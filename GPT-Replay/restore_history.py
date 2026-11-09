"""一次性恢复历史文件；所有写入仅限本备份目录，已存在文件拒绝覆盖。"""
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
APP = ROOT / "maixcam2_rect_center"


def blob(rev, name):
    return subprocess.check_output(["git", "show", rev + ":maixcam2_rect_center/" + name], cwd=REPO)


def write_new(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(data)


def main():
    for name in ("config.py", "vision/__init__.py", "vision/rectangle_center.py", "assets/icon.png"):
        data = blob("fe530c8", name)
        if name == "vision/rectangle_center.py":
            text = data.decode("utf-8")
            # 本对话 10 月 6 日的最终代码：全宽从严判黑带、Hough 默认关闭。
            text = text.replace("[0.25, 0.50, 0.75, 1.22, 1.45]", "[0.20, 0.40, 0.60, 1.22, 1.45]")
            text = text.replace("dark = np.median(samples[:, :, :3], axis=2)",
                                "dark = np.max(samples[:, :, :3], axis=2)")
            anchor = "        line_edges = cv2.Canny(cv2.GaussianBlur(small, (5, 5), 0),"
            assert text.count(anchor) == 1
            text = text.replace(anchor, '        if not getattr(self.cfg, "ENABLE_LINE_RECOVERY", False):\n            return None\n' + anchor)
            data = text.encode("utf-8")
        write_new(APP / name, data)
    write_new(APP / "tools/check_app.py", blob("d2f36e7", "tools/check_app.py"))
    write_new(APP / "tools/test_performance.py", blob("d2f36e7", "tools/test_performance.py"))
    # 设备桩来自最早的同仓工程，使备份可独立在 PC 运行，不导入现行应用。
    paths = subprocess.check_output(["git", "ls-tree", "-r", "--name-only", "bfff85a", "--", "tools/maix_stub"], cwd=REPO).decode().splitlines()
    assert paths, "未找到历史 Maix 桩"
    for name in paths:
        if not name.endswith(".py"):
            continue
        data = subprocess.check_output(["git", "show", "bfff85a:" + name], cwd=REPO)
        write_new(APP / name, data)
    print("历史算法、图标、门禁、测试及 Maix 桩已恢复到", APP)


if __name__ == "__main__":
    main()
