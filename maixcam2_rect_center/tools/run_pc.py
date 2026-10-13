"""PC（Windows）上的黑框识别运行器：图片 / 视频 / 摄像头，全部走**真实检测算法**。

为什么需要它：`main.py` 是板端入口（第一步就 `from maix import ...`），在 Windows 上
跑不起来；而板端调试成本高（要连网、传文件、看串口）。本工具把**同一套** `vision/`
检测器搬到 PC 上，用本地图片/摄像头直接看识别效果，是调参和复现问题最快的一条路。

用法（本机 Python 3.11 + numpy + opencv，**用 `D:/Python311/python.exe`**）：

    # ① 跑一个目录/通配符下的所有图片（最常用：把实拍图丢进一个目录）
    "D:/Python311/python.exe" -B tools/run_pc.py --images 测试图/*.png

    # ② 跑一段录屏/视频
    "D:/Python311/python.exe" -B tools/run_pc.py --video 现场录像.mp4

    # ③ 跑本机摄像头（需插 USB 摄像头）
    "D:/Python311/python.exe" -B tools/run_pc.py --camera 0

    # ④ 不要窗口、只出标注图和结论（无头/批量）
    "D:/Python311/python.exe" -B tools/run_pc.py --images 测试图/*.png --no-window --out 输出

    # ⑤ 临时改参数验证（不用改 config.py）
    "D:/Python311/python.exe" -B tools/run_pc.py --images t/*.png --set MIN_AREA_RATIO=0.01

窗口快捷键：`q`/`ESC` 退出，`空格` 暂停/继续，`n` 下一张（图片模式），
`s` 保存当前标注图，`p` 打印当前帧诊断。

⚠ 路径编码：本机工作区路径含中文，`cv2.imread/imwrite` 在 Windows 上会失败，
本工具一律走 `np.fromfile + imdecode` / `imencode + tofile`。
"""
import argparse
import glob
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import Config
from vision.rectangle_center import RectangleCenterDetector, draw_result


def imread_unicode(path):
    """读图（兼容中文/非 ASCII 路径；`cv2.imread` 在 Windows 上会返回 None）。"""
    try:
        buf = np.fromfile(str(path), dtype=np.uint8)
    except OSError as exc:
        print("[warn] 读不到 %s: %s" % (path, exc))
        return None
    if buf.size == 0:
        return None
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def imwrite_unicode(path, image):
    """写图（同样兼容非 ASCII 路径）。"""
    ok, buf = cv2.imencode(Path(path).suffix or ".png", image)
    if not ok:
        return False
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    buf.tofile(str(path))
    return True


def apply_overrides(cfg, pairs):
    """`--set KEY=VALUE` 临时覆盖 Config 属性（只在本次进程内生效，不改文件）。"""
    for item in pairs:
        if "=" not in item:
            raise SystemExit("--set 需要 KEY=VALUE 形式，收到: %s" % item)
        key, raw = item.split("=", 1)
        key = key.strip()
        if not hasattr(cfg, key):
            raise SystemExit("Config 里没有参数 %s" % key)
        old = getattr(cfg, key)
        text = raw.strip()
        if text.lower() in ("true", "false"):
            value = text.lower() == "true"
        elif text.startswith("(") and text.endswith(")"):        # 元组值（保留通用支持）
            value = tuple(int(x) for x in text[1:-1].split(",") if x.strip())
        else:
            try:
                value = int(text)
            except ValueError:
                try:
                    value = float(text)
                except ValueError:
                    value = text
        setattr(cfg, key, value)
        print("[cfg] %s: %r -> %r" % (key, old, value))


def frames_from_images(patterns):
    """把一组通配符/目录展开成图片路径列表（保持输入顺序）。"""
    paths = []
    for pattern in patterns:
        p = Path(pattern)
        if p.is_dir():
            hits = sorted(str(x) for x in p.iterdir()
                          if x.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp", ".webp"))
        else:
            hits = sorted(glob.glob(pattern))
        if not hits:
            print("[warn] 没有匹配到图片: %s" % pattern)
        paths.extend(hits)
    return paths


def camera_source(index, width, height):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise SystemExit("打不开摄像头 %d。请改用 --images / --video，或确认摄像头未被占用。"
                         % index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def video_source(path):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit("打不开视频: %s" % path)
    return cap


def describe(result):
    return ("stage=%-12s valid=%-5s proposals=%2d thresholds=%d detect=%6.2fms%s"
            % (result.get("stage"), result.get("valid"), result.get("proposals", 0),
               result.get("thresholds", 0), result.get("detector_ms", 0.0),
               "" if not result.get("valid") else
               "  center=(%.1f,%.1f)" % (result["u"], result["v"])))


def overlay(canvas, text):
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 26), (0, 0, 0), -1)
    cv2.putText(canvas, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
                cv2.LINE_AA)
    return canvas


def process_one(detector, gray, show_width, save_path=None):
    result = detector.process(gray)
    canvas = draw_result(gray, result, show_width)
    overlay(canvas, "PC runner  " + describe(result))
    if save_path:
        imwrite_unicode(save_path, canvas)
    return result, canvas


def run_images(detector, paths, args):
    hits = 0
    for i, path in enumerate(paths):
        bgr = imread_unicode(path)
        if bgr is None:
            print("[%d/%d] %s  <- 读不出，跳过" % (i + 1, len(paths), path))
            continue
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        save_path = None
        if args.out:
            save_path = Path(args.out) / (Path(path).stem + "_annotated.png")
        result, canvas = process_one(detector, gray, args.show_width, save_path)
        hits += bool(result.get("valid"))
        print("[%d/%d] %-44s %s" % (i + 1, len(paths), Path(path).name, describe(result)))
        if not args.no_window:
            cv2.imshow("run_pc", canvas)
            key = cv2.waitKey(0 if args.step else 1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                imwrite_unicode(Path(args.out or ".") / (Path(path).stem + "_annotated.png"),
                                canvas)
                print("      已保存标注图")
    print("\n图片模式小结：%d/%d 张检出（阈值/参数可用 --set 现场试）" % (hits, len(paths)))
    return hits, len(paths)


def run_stream(detector, cap, args, label):
    frames = hits = 0
    fps_times = []
    paused = False
    last_canvas = None
    while True:
        if not paused:
            ok, bgr = cap.read()
            if not ok:
                break
            if bgr.ndim == 3:
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            else:
                gray = bgr
            t0 = time.perf_counter()
            result, canvas = process_one(detector, gray, args.show_width,
                                         Path(args.out) / ("%s_%05d.png" % (label, frames))
                                         if args.out and frames % max(1, args.save_every) == 0
                                         else None)
            fps_times.append(time.perf_counter() - t0)
            if len(fps_times) > 30:
                fps_times.pop(0)
            frames += 1
            hits += bool(result.get("valid"))
            mean = float(np.mean(fps_times)) if fps_times else 0.0
            text = "fps=%.1f  %s" % (1.0 / mean if mean else 0.0, describe(result))
            overlay(canvas, text)
            last_canvas = canvas
            if frames % 30 == 0 or frames <= 5:
                print("[%s] frame %5d  %s" % (label, frames, describe(result)))
            if args.max_frames and frames >= args.max_frames:
                break
        if args.no_window:
            continue
        cv2.imshow("run_pc", last_canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord(" "):
            paused = not paused
        if key == ord("p") and last_canvas is not None:
            print("      当前帧: %s" % describe(detector.last or {}))
        if key == ord("s") and last_canvas is not None:
            out = Path(args.out or ".") / ("%s_manual_%05d.png" % (label, frames))
            imwrite_unicode(out, last_canvas)
            print("      已保存 %s" % out)
    print("\n%s 小结：%d 帧，%d 帧检出（%.0f%%）"
          % (label, frames, hits, 100.0 * hits / max(1, frames)))
    return hits, frames


def main():
    ap = argparse.ArgumentParser(description="PC 端黑框识别运行器（Windows 可用）")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--images", nargs="+", metavar="PATH", help="图片文件/通配符/目录（可多个）")
    src.add_argument("--video", metavar="FILE", help="视频文件")
    src.add_argument("--camera", nargs="?", const=0, type=int, metavar="INDEX",
                     help="本机摄像头序号（默认 0）")
    ap.add_argument("--out", metavar="DIR", help="把标注图写到该目录（可配合 --no-window）")
    ap.add_argument("--no-window", action="store_true", help="不弹窗（无头/批量）")
    ap.add_argument("--step", action="store_true", help="图片模式逐张等待按键")
    ap.add_argument("--show-width", type=int, default=640, help="预览画布宽度，默认 640")
    ap.add_argument("--max-frames", type=int, default=0, help="流模式最多处理多少帧（0=不限）")
    ap.add_argument("--save-every", type=int, default=30, help="配合 --out：每 N 帧存一张")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="临时覆盖 Config 参数（可重复，不改文件）")
    args = ap.parse_args()

    cfg = type("PcConfig", (Config,), {})
    apply_overrides(cfg, args.set)
    cv2.setNumThreads(max(1, cfg.OPENCV_THREADS))
    detector = RectangleCenterDetector(cfg)
    print("[pc] OpenCV %s  SEARCH_WIDTH=%d  CANNY=%d/%d  EDGE_CLOSE_PX=%d"
          % (cv2.__version__, cfg.SEARCH_WIDTH, cfg.CANNY_LOW, cfg.CANNY_HIGH, cfg.EDGE_CLOSE_PX))
    print("[pc] 校验门 MIN_EDGE_COVERAGE=%.2f WEAK=%.2f STRONG=%.2f"
          % (cfg.MIN_EDGE_COVERAGE, cfg.WEAK_EDGE_COVERAGE, cfg.STRONG_EDGE_COVERAGE))

    if args.images:
        paths = frames_from_images(args.images)
        if not paths:
            raise SystemExit("没有可处理的图片")
        run_images(detector, paths, args)
    else:
        cap = camera_source(args.camera, cfg.CAMERA_W, cfg.CAMERA_H) if args.camera is not None \
            else video_source(args.video)
        try:
            run_stream(detector, cap, args, "camera" if args.camera is not None else "video")
        finally:
            cap.release()
    if not args.no_window:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
