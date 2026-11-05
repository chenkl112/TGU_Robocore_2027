"""使用真实模型和有目标视频验证同步回放、帧索引及失败边界。"""

import csv
import json
import math
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile


def replace_setting(config, section, key, value):
    """只替换指定小节的字段，保留夹具之外的生产配置和默认门限。"""
    section_pattern = rf"(?ms)(^\[{re.escape(section)}\]\s*\n)(.*?)(?=^\[|\Z)"
    match = re.search(section_pattern, config)
    assert match, f"Missing config section: {section}"
    body, count = re.subn(
        rf"(?ms)^{re.escape(key)}\s*=\s*(?:\[[^\]]*\]|[^\n]*)",
        f"{key} = {value}", match.group(2), count=1)
    assert count == 1, f"Missing config field: {section}.{key}"
    return config[:match.start(2)] + body + config[match.end(2):]


def positive_config(repo):
    config = (repo / "config/auto_aim.toml").read_text()
    # 合成图相机中心在 640×480 图像中心，坐标系与正式流程相同。
    values = {
        "R_gimbal2imubody": "[1.,0.,0.,0.,1.,0.,0.,0.,1.]",
        "R_camera2gimbal": "[0.,0.,1.,-1.,0.,0.,0.,-1.,0.]",
        "t_camera2gimbal": "[0.,0.,0.]",
        "camera_matrix": "[800.,0.,320.,0.,800.,240.,0.,0.,1.]",
        "distort_coeffs": "[0.,0.,0.,0.,0.]",
    }
    # TOML 浮点字面量的小数点后必须有数字。
    for key, value in values.items():
        config = replace_setting(config, "camera", key, re.sub(r"\.(?=[,\]])", ".0", value))
    return replace_setting(config, "yolo", "device", '"CPU"')


def headless_environment():
    env = dict(os.environ)
    env.pop("DISPLAY", None)
    env.pop("WAYLAND_DISPLAY", None)
    # 若回放误调用 GUI，缺失的平台插件会让测试失败，不能用 offscreen 掩盖。
    env["QT_QPA_PLATFORM"] = "robocore-headless-must-not-load"
    return env


def replay(repo, build, prefix, config, output, *options):
    return subprocess.run(
        [build / "auto_aim_test", f"--config-path={config}", "--headless=true",
         "--device=CPU", f"--output={output}", *options, prefix],
        cwd=repo, env=headless_environment(), capture_output=True, text=True, timeout=45)


def rows(path):
    with path.open(newline="") as source:
        result = list(csv.DictReader(source))
    assert result, "Replay did not emit any CSV rows"
    return result


def require_active_target(records, first_frame=0):
    assert [int(row["frame"]) for row in records] == list(range(first_frame, first_frame + len(records)))
    for offset, row in enumerate(records):
        assert math.isclose(float(row["time_seconds"]), 3600 + (first_frame + offset) * 0.01, abs_tol=1e-10)
        assert row["detections"] == "1", row
        assert row["fire"] == "0", "Replay without measured feedback requested fire"
        if offset < 4:
            assert row["tracker_state"] == "detecting" and row["control"] == "0", row
            assert row["target_present"] == "0" and row["stop_reason"] == "no_target", row
        else:
            assert row["tracker_state"] == "tracking" and row["control"] == "1", row
            assert row["target_present"] == "1" and row["target_name"] == "one", row
            assert row["armor_count"] == "4" and row["observed"] == "1", row
            assert row["stop_reason"] == "none", row
            for field in ("target_yaw", "target_pitch", "yaw", "yaw_vel", "yaw_acc", "pitch", "pitch_vel", "pitch_acc"):
                assert math.isfinite(float(row[field])), (field, row)
    assert any(row["control"] == "1" for row in records), "Only blank/no-target frames were exercised"


def verify_output_safety(repo, build, prefix, directory):
    # 只操作临时模型副本，任何拒绝路径失效都不能修改仓库中的真实模型。
    models = directory / "models"
    models.mkdir()
    for filename in ("yolov8_640x480.xml", "yolov8_640x480.bin", "tiny_resnet.onnx"):
        shutil.copyfile(repo / "assets" / filename, models / filename)
    content = replace_setting(positive_config(repo), "yolo", "yolo_model", json.dumps(str(models / "yolov8_640x480.xml")))
    content = replace_setting(content, "classifier", "classify_model", json.dumps(str(models / "tiny_resnet.onnx")))
    config = directory / "copied-models.toml"
    config.write_text(content)
    protected = [prefix.with_suffix(".txt"), prefix.with_suffix(".avi"), config,
                 models / "yolov8_640x480.xml", models / "yolov8_640x480.bin", models / "tiny_resnet.onnx"]
    originals = {path: path.read_bytes() for path in protected}
    for index, source in enumerate(protected):
        alias = directory / f"alias-{index}.csv"
        os.link(source, alias)
        for destination in (source, alias):
            completed = replay(repo, build, prefix, config, destination, "--end-index=5")
            assert completed.returncode != 0 and "must not overwrite" in completed.stdout + completed.stderr, completed.stdout + completed.stderr
            assert source.read_bytes() == originals[source], f"Output validation damaged {source.name}"
        alias.unlink()

    saved = directory / "existing.csv"
    sentinel = b"previous valid result\n"
    saved.write_bytes(sentinel)
    config.write_text(replace_setting(content, "yolo", "yolo_model", json.dumps(str(models / "missing.xml"))))
    completed = replay(repo, build, prefix, config, saved, "--end-index=5")
    assert completed.returncode != 0 and saved.read_bytes() == sentinel, "Model failure damaged existing CSV"
    config.write_text(content)
    completed = replay(repo, build, prefix, config, directory / "absent" / "result.csv", "--end-index=5")
    assert completed.returncode != 0, "Unwritable output path was accepted"
    assert not list(directory.rglob("*.tmp.*")), "Failed replay leaked a temporary CSV"
    for source, original in originals.items():
        assert source.read_bytes() == original, f"A failed output path changed input {source.name}"


def run(scenario, repo, build):
    repo, build = pathlib.Path(repo).resolve(), pathlib.Path(build).resolve()
    with tempfile.TemporaryDirectory(prefix="robocore-pipeline-") as temporary:
        directory = pathlib.Path(temporary)
        prefix = directory / "positive"
        subprocess.run([build / "fixture_generator", prefix, "positive"], check=True, timeout=20)
        config = directory / "config.toml"
        config.write_text(positive_config(repo))
        first, second = directory / "first.csv", directory / "second.csv"
        pose_path = prefix.with_suffix(".txt")
        original_poses = pose_path.read_text()
        if scenario == "replay_deterministic":
            for output in (first, second):
                completed = replay(repo, build, prefix, config, output, "--end-index=19")
                assert completed.returncode == 0, completed.stdout + completed.stderr
                require_active_target(rows(output))
            assert first.read_bytes() == second.read_bytes(), "Repeated positive replay changed its per-frame CSV"
            print("PASS repeated positive replay: 20 identical CSV rows, confirmed control, headless, no fire")
        elif scenario == "replay_positive_seek":
            lines = original_poses.splitlines()
            lines[:10] = [line.split()[0] + " 0 0 0 0" for line in lines[:10]]
            pose_path.write_text("\n".join(lines) + "\n")
            completed = replay(repo, build, prefix, config, first, "--start-index=10", "--end-index=19")
            assert completed.returncode == 0, completed.stdout + completed.stderr
            records = rows(first)
            assert len(records) == 10, records
            require_active_target(records, first_frame=10)
            print("PASS positive replay seek: skipped invalid pre-roll poses, confirmed at requested frame + 4")
        elif scenario == "replay_invalid_records":
            source = original_poses.splitlines()
            cases = []
            for label, index, replacement, expected in (
                ("duplicate", 2, source[1], "strictly increasing"),
                ("backward", 2, "3599 1 0 0 0", "strictly increasing"),
                ("negative", 0, "-1 1 0 0 0", "Invalid recording timestamp"),
                ("overflow", 0, "1e20 1 0 0 0", "Invalid recording timestamp"),
                ("nan", 0, "nan 1 0 0 0", "Invalid pose record"),
                ("invalid_quaternion", 0, "3600 0 0 0 0", "invalid gimbal quaternion"),
                ("missing_field", 0, "3600 1 0 0", "Invalid pose record"),
                ("extra_field", 0, "3600 1 0 0 0 extra", "Invalid pose record"),
            ):
                modified = source.copy()
                modified[index] = replacement
                cases.append((label, "\n".join(modified) + "\n", expected))
            cases.append(("short_stream", "\n".join(source[:2]) + "\n", "Pose stream ended"))
            sentinel = b"previous valid result\n"
            first.write_bytes(sentinel)
            for label, text, expected in cases:
                pose_path.write_text(text)
                completed = replay(repo, build, prefix, config, first, "--end-index=5")
                details = completed.stdout + completed.stderr
                assert completed.returncode != 0 and expected in details, (label, details)
                assert first.read_bytes() == sentinel, f"Failed {label} replay damaged the previous CSV"
            pose_path.write_text(original_poses)
            completed = replay(repo, build, prefix, config, pose_path, "--end-index=5")
            assert completed.returncode != 0 and "must not overwrite" in completed.stdout + completed.stderr
            assert pose_path.read_text() == original_poses, "Output path validation damaged the pose stream"
            completed = replay(repo, build, prefix, config, first, "--start-index=5", "--end-index=4")
            assert completed.returncode != 0 and "Invalid replay options" in completed.stdout + completed.stderr
            verify_output_safety(repo, build, prefix, directory)
            print(f"PASS {len(cases)} invalid records, input/model hardlink protection and atomic CSV output")
        else:
            raise AssertionError(f"Unknown pipeline scenario: {scenario}")


if __name__ == "__main__":
    run(*sys.argv[1:])
