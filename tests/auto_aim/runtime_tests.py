"""使用生成的视频和伪终端云台验证正式入口的退出、停控及回放跳帧。"""
import math
import os
import pathlib
import pty
import select
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tty


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


def run(scenario, repo, build):
    repo, build = pathlib.Path(repo).resolve(), pathlib.Path(build).resolve()
    with tempfile.TemporaryDirectory(prefix="robocore-runtime-") as directory:
        directory = pathlib.Path(directory)
        prefix = directory / "recording"
        positive = scenario in {"positive_target_recovery", "pose_wait_shutdown"}
        subprocess.run([build / "fixture_generator", prefix, *(["positive"] if positive else [])], check=True)
        if scenario == "replay_seek":
            env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
            result = subprocess.run(
                [build / "auto_aim_test", "--config-path=config/auto_aim.toml",
                 "--start-index=10", "--end-index=10", "--headless=true", "--device=CPU", prefix],
                cwd=repo, env=env, capture_output=True, text=True, timeout=25)
            assert result.returncode == 0, result.stdout + result.stderr
            assert "frame=10" in result.stdout, result.stdout
            print("PASS replay frame/pose seek at a timestamp beyond 35 minutes")
            return

        done = threading.Event()
        server = None
        network_peer = None
        if scenario == "network_camera_silence":
            server = socket.socket()
            server.bind(("127.0.0.1", 0))
            server.listen()
            server.settimeout(0.1)

            def silent_server():
                clients = []
                try:
                    while not done.is_set():
                        try:
                            client, _ = server.accept()
                            clients.append(client)
                        except TimeoutError:
                            pass
                finally:
                    for client in clients:
                        client.close()

            network_peer = threading.Thread(target=silent_server)
            network_peer.start()
        master, slave = pty.openpty()
        tty.setraw(slave)
        os.set_blocking(master, False)
        if positive:
            from pipeline_runtime_tests import positive_config, replace_setting
            config = replace_setting(positive_config(repo), "camera", "fps", "30.0")
            if scenario == "pose_wait_shutdown":
                config = replace_setting(config, "gimbal", "feedback_timeout_ms", "5000")
        else:
            config = (repo / "config/auto_aim.toml").read_text()
        camera = prefix.with_suffix(".avi") if scenario == "signal_shutdown" or positive else directory / "missing.avi"
        if server is not None:
            camera = f"http://127.0.0.1:{server.getsockname()[1]}/stream"
        config = config.replace("source = 0", 'source = "' + str(camera) + '"')
        config = config.replace('com_port = "/dev/ttyUSB0"', 'com_port = "' + os.ttyname(slave) + '"')
        path = directory / "config.toml"
        path.write_text(config)
        feedback_enabled = threading.Event()
        feedback_enabled.set()
        feedback_mode = [1]

        def send_feedback():
            while not done.wait(0.005):
                if not feedback_enabled.is_set():
                    continue
                feedback = struct.pack("<2sB4f5fH", b"SP", feedback_mode[0], 1, 0, 0, 0,
                                       0, 0, 0, 0, 22, 0)
                feedback += struct.pack("<H", crc16(feedback))
                try:
                    os.write(master, feedback)
                except BlockingIOError:
                    pass

        peer = threading.Thread(target=send_feedback)
        peer.start()
        log_path = directory / "sentry.log"
        process = None
        received = bytearray()
        packets = []

        def receive_available(timeout):
            if select.select([master], [], [], timeout)[0]:
                received.extend(os.read(master, 4096))
            fresh = []
            while len(received) >= 29:
                offset = received.find(b"SP")
                if offset < 0:
                    received.clear()
                    break
                del received[:offset]
                if len(received) < 29:
                    break
                packet = bytes(received[:29])
                del received[:29]
                assert crc16(packet[:-2]) == struct.unpack("<H", packet[-2:])[0], "Bad outgoing CRC"
                assert all(math.isfinite(value) for value in struct.unpack("<6f", packet[3:27])), "Nonfinite wire command"
                packets.append(packet)
                fresh.append(packet)
            return fresh

        def wait_mode(expected, timeout=4):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()
                if any(packet[2] == expected for packet in receive_available(0.02)):
                    return
            raise AssertionError(f"No wire mode {expected} before deadline\n" + log_path.read_text())

        try:
            with log_path.open("w") as log:
                process = subprocess.Popen(
                    ["stdbuf", "-oL", "-eL", build / "auto_aim_sentry", path],
                    cwd=repo, stdout=log, stderr=subprocess.STDOUT)
                deadline = time.monotonic() + 20
                while "[MultiThreadYOLO] started." not in log_path.read_text():
                    assert process.poll() is None, log_path.read_text()
                    assert time.monotonic() < deadline, "Startup timed out\n" + log_path.read_text()
                    time.sleep(0.02)
                if scenario == "pose_wait_shutdown":
                    wait_mode(1, timeout=6)
                    # 工作链先确认目标，再停 IMU。视频继续产生后续帧，跟踪会等待新的右侧位姿。
                    # 只等 250 ms，远短于配置的 5 s 超时；退出必须通过停止令牌唤醒等待。
                    feedback_enabled.clear()
                    deadline = time.monotonic() + 0.25
                    while time.monotonic() < deadline:
                        receive_available(0.02)
                elif positive:
                    wait_mode(1, timeout=6)
                    # 先建立真正的目标控制，再停反馈；恢复反馈后必须重新确认目标。
                    feedback_enabled.clear()
                    wait_mode(0)
                    deadline = time.monotonic() + 0.2
                    while time.monotonic() < deadline:
                        assert all(packet[2] == 0 for packet in receive_available(0.02)), "Feedback outage retained target control"
                    feedback_enabled.set()
                    wait_mode(1)
                    feedback_mode[0] = 0
                    wait_mode(0)
                    deadline = time.monotonic() + 0.15
                    while time.monotonic() < deadline:
                        assert all(packet[2] == 0 for packet in receive_available(0.02)), "IDLE mode retained target control"
                    feedback_mode[0] = 1
                    wait_mode(1)
                else:
                    deadline = time.monotonic() + 0.3
                    while time.monotonic() < deadline:
                        receive_available(0.02)
                started = time.monotonic()
                process.send_signal(signal.SIGTERM)
                assert process.wait(timeout=3) == 0, log_path.read_text()
                elapsed = time.monotonic() - started
                if scenario == "pose_wait_shutdown":
                    assert elapsed < 1, f"Shutdown waited for the configured pose timeout: {elapsed:.3f}s"
                while select.select([master], [], [], 0.02)[0]:
                    receive_available(0)
                assert len(packets) >= 2, "No control heartbeat received\n" + log_path.read_text()
                assert packets[-1][2] == 0, "Exit did not send a stop command"
                assert all(packet[2] != 2 for packet in packets), "Disabled auto-fire reached the wire"
                if positive:
                    assert any(packet[2] == 1 for packet in packets), "Positive video never controlled the gimbal"
                print(f"PASS {scenario}: {len(packets)} commands, safe stop, exit in {elapsed:.3f}s")
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait()
            done.set()
            peer.join()
            if network_peer is not None:
                network_peer.join()
                server.close()
            os.close(master)
            os.close(slave)


if __name__ == "__main__":
    run(*sys.argv[1:])
