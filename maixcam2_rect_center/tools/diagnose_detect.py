"""逐层打印淘汰计数：定位"没检出"卡在面积/四边形/长宽比哪一层。

用法：
    diagnose_detect.py <图片> [--save <目录>]

输出四段，对应流水线四步：
    1. 二值化统计（白像素占比）——判断自适应阈值是否把胶带分成连通白环
    2. 轮廓总数             ——判断噪点量
    3. 三层淘汰计数         ——判断死在哪一层
    4. 通过三层的候选清单   ——判断"选最大"是否会选错

`--save` 会把 blur / binary / result 三张图写到目录，用来看二值图是否闭合。
"""
import argparse
import json
from pathlib import Path

from run_pc import imread_unicode, imwrite_unicode, load_config
from vision.rectangle_center import (RectangleCenterDetector, draw_result,
                                     blur_frame, binarize_frame, find_quad_candidates,
                                     filter_band)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image")
    parser.add_argument("--save", type=Path, help="导出 blur/binary/result 到该目录")
    parser.add_argument("--top", type=int, default=8, help="打印前 N 个候选")
    args = parser.parse_args()

    gray = imread_unicode(args.image)
    if gray is None:
        raise SystemExit("Cannot read image: " + args.image)

    cfg = load_config(RECT_KEEP_STAGES=True)
    detector = RectangleCenterDetector(cfg)
    gray = gray if gray.ndim == 2 else gray[:, :, 0]

    blur = blur_frame(gray, cfg.RECT_GAUSS_KERNEL)
    binary = binarize_frame(blur, cfg.RECT_BLOCK_SIZE, cfg.RECT_C)
    white = float((binary > 0).mean())
    print("image %dx%d white_ratio=%.3f block=%d c=%.0f"
          % (gray.shape[1], gray.shape[0], white, cfg.RECT_BLOCK_SIZE, cfg.RECT_C))

    candidates, stats = find_quad_candidates(
        binary, cfg.RECT_MIN_AREA, cfg.RECT_MAX_AREA_RATIO, cfg.RECT_APPROX_EPS,
        cfg.RECT_MIN_ASPECT, cfg.RECT_MAX_ASPECT, cfg.RECT_MAX_CANDIDATES)
    print("contours=%d  reject: area=%d quad=%d aspect=%d  passed=%d"
          % (stats["contours"], stats["area"], stats["quad"], stats["aspect"],
             stats["candidates"]))
    for i, cand in enumerate(candidates[:max(0, args.top)]):
        x, y, w, h = cand["bbox"]
        score = filter_band(gray, [cand], cfg.RECT_MIN_BAND_SCORE,
                            cfg.RECT_BAND_OFFSET_PX, cfg.RECT_BAND_CONTRAST,
                            cfg.RECT_MIN_BAND_RATIO, cfg.RECT_BAND_DARK_RATIO,
                            cfg.RECT_BAND_PROBE_RATIO)[0]
        print("  #%d area=%.0f bbox=(%d,%d,%d,%d) aspect=%.2f band=%s"
              % (i, cand["area"], x, y, w, h, cand["aspect"],
                 "%.2f ratio=%.3f" % (score[0]["band"], score[0].get("band_ratio", 0.0))
                 if score else "-"))

    result = detector.process(gray)
    print("result valid=%s stage=%s reason=%s ms=%.2f"
          % (result["valid"], result["stage"], result.get("fail_reason", "-"),
             result["detector_ms"]))
    if result["valid"]:
        print(json.dumps(dict(center=(round(result["u"], 1), round(result["v"], 1)),
                              quad=[[int(round(v)) for v in p] for p in result["quad"]],
                              area=int(result.get("area", 0)),
                              aspect=round(float(result.get("aspect", 0.)), 3)),
                         ensure_ascii=False))

    if args.save:
        imwrite_unicode(args.save / "01_blur.png", blur)
        imwrite_unicode(args.save / "02_binary.png", binary)
        imwrite_unicode(args.save / "03_result.png",
                        draw_result(gray, result, gray.shape[1]))
        print("saved ->", args.save)


if __name__ == "__main__":
    main()
