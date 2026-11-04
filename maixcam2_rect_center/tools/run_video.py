"""Run the rectangle detector on a screen recording of the MaixVision camera pane.

The recording is a desktop capture: `--pane maixvision` crops the camera preview
panel (the top-right "图像" pane) in relative coordinates, so it works for any
recording size.  Use `--crop x0,y0,x1,y1` for arbitrary pixel rectangles.

Outputs next to --out (default tools/video_report/<stem>/):
    det.mp4       annotated video (green = full lock, yellow = partial lock)
    frames.json   per-frame result (valid/stage/u/v/partial/refined_edges/ms)
    contact.png   12-frame contact sheet
    summary.txt   aggregate metrics

This is a PC-only evaluation tool; it is excluded from the app package.

Examples
--------
    python -B tools/run_video.py --video ../vest视频/maixvision.mp4 --pane maixvision
    python -B tools/run_video.py --video clip.mp4 --crop 0,0,640,480 --size 640x480
    python -B tools/run_video.py --video clip.mp4 --pane maixvision --no-track --stride 2
"""
import argparse
import json
import os
import shutil
import sys
import tempfile
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import Config  # noqa: E402
from vision.rectangle_center import RectangleCenterDetector, draw_result  # noqa: E402

# relative (x0, y0, x1, y1) of the MaixVision camera pane in a 2560x1440 capture
PANES = {"maixvision": (2070 / 2560, 280 / 1440, 2520 / 2560, 618 / 1440)}
CONTACT_COLS, CONTACT_ROWS = 4, 3
CONTACT_W, CONTACT_H = 320, 240


def parse_crop(text):
    parts = [float(v) for v in text.replace(" ", "").split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("crop must be x0,y0,x1,y1")
    return tuple(parts)


def parse_size(text):
    w, h = text.lower().split("x")
    return int(w), int(h)


def crop_rect(frame, args):
    h, w = frame.shape[:2]
    if args.crop is not None:
        x0, y0, x1, y1 = args.crop
    else:
        fx0, fy0, fx1, fy1 = PANES[args.pane]
        x0, y0, x1, y1 = fx0 * w, fy0 * h, fx1 * w, fy1 * h
    x0, y0 = max(0, int(round(x0))), max(0, int(round(y0)))
    x1, y1 = min(w, int(round(x1))), min(h, int(round(y1)))
    if x1 - x0 < 8 or y1 - y0 < 8:
        raise ValueError("crop rectangle is empty")
    return x0, y0, x1, y1


def annotate(result, gray, index, t, size):
    canvas = draw_result(gray, result, size[0])
    valid = bool(result.get("valid"))
    partial = bool(result.get("partial"))
    if valid:
        color = (0, 220, 255) if partial else (0, 255, 0)      # BGR: yellow / green
        tag = "PARTIAL %d/4 edges" % result.get("refined_edges", 4) if partial else "LOCK"
    else:
        color, tag = (0, 0, 255), "NO LOCK"
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1] - 1, canvas.shape[0] - 1), color, 2)
    cv2.putText(canvas, "#%04d t=%.1fs  %s  stage=%s  %s  %.1fms"
                % (index, t, tag, result.get("stage", "?"),
                   "clipped=%d" % result.get("clipped_edges", 0), result.get("detector_ms", 0.)),
                (8, canvas.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, .42, color, 1, cv2.LINE_AA)
    return canvas


def contact_sheet(samples, size):
    sheet = np.full((CONTACT_ROWS * CONTACT_H, CONTACT_COLS * CONTACT_W, 3), 24, np.uint8)
    for k, (index, t, canvas, result) in enumerate(samples):
        row, col = divmod(k, CONTACT_COLS)
        tile = cv2.resize(canvas, (CONTACT_W, CONTACT_H), interpolation=cv2.INTER_AREA)
        sheet[row * CONTACT_H:(row + 1) * CONTACT_H,
              col * CONTACT_W:(col + 1) * CONTACT_W] = tile
    return sheet


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--video", required=True)
    parser.add_argument("--pane", choices=sorted(PANES), default="maixvision",
                        help="named crop preset (ignored when --crop is given)")
    parser.add_argument("--crop", type=parse_crop, default=None,
                        help="pixel crop in the source frame: x0,y0,x1,y1")
    parser.add_argument("--size", type=parse_size, default=(640, 480),
                        help="detector input size (default 640x480)")
    parser.add_argument("--out", default=None, help="output directory")
    parser.add_argument("--stride", type=int, default=1, help="process every Nth frame")
    parser.add_argument("--start", type=float, default=0.0, help="start time in seconds")
    parser.add_argument("--end", type=float, default=0.0, help="end time in seconds (0 = EOF)")
    parser.add_argument("--limit", type=int, default=0, help="max processed frames (0 = all)")
    parser.add_argument("--no-track", action="store_true",
                        help="fresh detector per frame (cold acquisition instead of tracking)")
    parser.add_argument("--no-video", action="store_true", help="skip the annotated mp4")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    src = Path(args.video)
    if not src.is_file():
        raise SystemExit("video not found: %s" % src)
    out_dir = Path(args.out) if args.out else ROOT / "tools" / "video_report" / src.stem
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(src))
    if not cap.isOpened():
        raise SystemExit("cannot open video: %s" % src)
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if args.start > 0:
        cap.set(cv2.CAP_PROP_POS_MSEC, args.start * 1000.0)
    # determine the crop from a real frame (cheap: read one and put it back)
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    ok, probe = cap.read()
    if not ok:
        raise SystemExit("cannot read the first frame")
    x0, y0, x1, y1 = crop_rect(probe, args)
    if args.start > 0:
        cap.set(cv2.CAP_PROP_POS_MSEC, args.start * 1000.0)
    else:
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    writer = None
    video_tmp = None
    if not args.no_video:
        # 本机路径含中文：cv2.VideoWriter 对 Unicode 路径会静默失败，先写临时 ASCII 再搬。
        fd, video_tmp = tempfile.mkstemp(suffix=".mp4")
        os.close(fd)
        writer = cv2.VideoWriter(video_tmp, cv2.VideoWriter_fourcc(*"mp4v"), fps, args.size)
        if not writer.isOpened():
            print("[video] WARN: mp4 writer unavailable, continuing without video")
            writer = None
            try:
                os.unlink(video_tmp)
            except OSError:
                pass
            video_tmp = None

    detector = None if args.no_track else RectangleCenterDetector(Config)
    records, samples = [], []
    stages = Counter()
    processed = valid = partial = 0
    ms_values = []
    loss_run = longest_loss = 0
    relock_frames = []
    first_valid = last_valid = None
    sample_every = max(1, (total or 1) // 12) if total else 60
    frame_index = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = frame_index / fps
        frame_index += 1
        if args.end > 0 and t > args.end:
            break
        if (frame_index - 1) % max(1, args.stride):
            continue
        pane = frame[y0:y1, x0:x1]
        gray = cv2.cvtColor(pane, cv2.COLOR_BGR2GRAY)
        if gray.shape[::-1] != args.size:
            gray = cv2.resize(gray, args.size, interpolation=cv2.INTER_CUBIC)
        if args.no_track:
            detector = RectangleCenterDetector(Config)
        result = detector.process(gray, timestamp=t)
        processed += 1
        stages[result.get("stage", "?")] += 1
        ms_values.append(result.get("detector_ms", 0.0))
        if result.get("valid"):
            valid += 1
            partial += bool(result.get("partial"))
            last_valid = t
            if first_valid is None:
                first_valid = t
            if loss_run:
                relock_frames.append(loss_run)
            loss_run = 0
        else:
            loss_run += 1
            longest_loss = max(longest_loss, loss_run)
        records.append(dict(i=frame_index - 1, t=round(t, 3),
                            valid=bool(result.get("valid")),
                            partial=bool(result.get("partial")),
                            refined_edges=result.get("refined_edges", 0),
                            clipped_edges=result.get("clipped_edges", 0),
                            stage=result.get("stage"),
                            u=result.get("u"), v=result.get("v"),
                            ms=round(result.get("detector_ms", 0.0), 2)))
        if writer is not None:
            writer.write(annotate(result, gray, frame_index - 1, t, args.size))
        if len(samples) < CONTACT_COLS * CONTACT_ROWS and processed % sample_every == 0:
            samples.append((frame_index - 1, t,
                            annotate(result, gray, frame_index - 1, t, args.size), result))
        if not args.quiet and processed % 200 == 0:
            print("  %d frames, valid=%d, partial=%d" % (processed, valid, partial),
                  flush=True)
        if args.limit and processed >= args.limit:
            break

    cap.release()
    if writer is not None:
        writer.release()
        shutil.move(video_tmp, str(out_dir / "det.mp4"))
    longest_loss = max(longest_loss, loss_run)
    ms = np.asarray(ms_values, dtype=float) if ms_values else np.zeros(1)
    summary = dict(
        video=str(src), crop=[x0, y0, x1, y1], size=list(args.size),
        source_fps=round(fps, 3), processed=processed,
        valid=valid, valid_rate=round(valid / max(1, processed), 4),
        partial=partial,
        stages=dict(stages),
        longest_loss_run=longest_loss,
        relock_runs=relock_frames[:50],
        first_valid_s=first_valid, last_valid_s=last_valid,
        ms_median=round(float(np.median(ms)), 2),
        ms_p95=round(float(np.percentile(ms, 95)), 2),
        ms_max=round(float(ms.max()), 2),
    )
    (out_dir / "frames.json").write_text(
        json.dumps(dict(summary=summary, frames=records), ensure_ascii=False, indent=1),
        encoding="utf-8")
    (out_dir / "summary.txt").write_text(
        "\n".join("%s: %s" % (k, summary[k]) for k in summary) + "\n", encoding="utf-8")
    if samples:
        sheet = contact_sheet(samples, args.size)
        ok, buf = cv2.imencode(".png", sheet)
        if ok:
            buf.tofile(str(out_dir / "contact.png"))
        else:
            print("[video] WARN: failed to encode the contact sheet")
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    print("outputs -> %s" % out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
