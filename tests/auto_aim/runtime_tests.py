"""Run the real sentry/replay binaries with generated video and a PTY gimbal."""
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
        subprocess.run([build / "fixture_generator", prefix], check=True)
        if scenario == "replay_seek":
            env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
            result = subprocess.run(
                [build / "auto_aim_test", "--config-path=config/auto_aim.toml",
                 "--start-index=10", "--end-index=10", prefix],
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
        config = (repo / "config/auto_aim.toml").read_text()
        camera = prefix.with_suffix(".avi") if scenario == "signal_shutdown" else directory / "missing.avi"
        if server is not None:
            camera = f"http://127.0.0.1:{server.getsockname()[1]}/stream"
        config = config.replace("source = 0", 'source = "' + str(camera) + '"')
        config = config.replace('com_port = "/dev/ttyUSB0"', 'com_port = "' + os.ttyname(slave) + '"')
        path = directory / "config.toml"
        path.write_text(config)
        feedback = struct.pack("<2sB4f5fH", b"SP", 1, 1, 0, 0, 0, 0, 0, 0, 0, 22, 0)
        feedback += struct.pack("<H", crc16(feedback))

        def send_feedback():
            while not done.wait(0.005):
                try:
                    os.write(master, feedback)
                except BlockingIOError:
                    pass

        peer = threading.Thread(target=send_feedback)
        peer.start()
        log_path = directory / "sentry.log"
        process = None
        received = bytearray()
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
                deadline = time.monotonic() + 0.3
                while time.monotonic() < deadline:
                    if select.select([master], [], [], 0.02)[0]:
                        received.extend(os.read(master, 4096))
                started = time.monotonic()
                process.send_signal(signal.SIGTERM)
                assert process.wait(timeout=3) == 0, log_path.read_text()
                elapsed = time.monotonic() - started
                while select.select([master], [], [], 0.02)[0]:
                    received.extend(os.read(master, 4096))
                packets = []
                while len(received) >= 29:
                    offset = received.find(b"SP")
                    if offset < 0:
                        break
                    del received[:offset]
                    if len(received) < 29:
                        break
                    packet = received[:29]
                    del received[:29]
                    assert crc16(packet[:-2]) == struct.unpack("<H", packet[-2:])[0], "Bad outgoing CRC"
                    packets.append(packet)
                assert len(packets) >= 2, "No control heartbeat received\n" + log_path.read_text()
                assert packets[-1][2] == 0, "Exit did not send a stop command"
                assert all(packet[2] != 2 for packet in packets), "Blank scene requested fire"
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
