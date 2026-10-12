"""诊断工具：把**真实画面**喂给 RectangleCenterDetector，打印每一级为什么拒绝。

用法：
    "D:/Python311/python.exe" -B tools/diagnose_detect.py <图片路径> [--out 结果图]

为什么需要它：离线基线用的是合成靶纸（29/30），但实机是"偶尔锁上、很快丢"。
本工具不猜原因，而是把 `_verify`（四边暗带校验）与 `_refine`（窄带精修）内部的
逐边数字全部打出来 —— 覆盖率 / 对比度 / 暗带与亮面灰度 —— 直接看是哪一项卡住。

`--overlay` 可把预览画布上的叠加层（按钮、左上角面板、底部状态串）涂平，
还原"检测器真正看到的相机灰度帧"（板上检测跑在原始帧上，不含任何叠加）。
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config import Config
import vision.rectangle_center as rc
from vision.rectangle_center import (_homography, _rectangle, _sample,
                                     project_points, RectangleCenterDetector)


def flatten_overlays(bgr):
    """把预览叠加层（按钮/面板/状态条）涂成中性灰，近似还原检测器看到的原始帧。"""
    out = bgr.copy()
    h, w = out.shape[:2]
    # 取画面中部一块干净区域的灰度当填充值（通常是靶纸周围背景）。
    fill = int(np.median(cv2.cvtColor(out[h // 3:2 * h // 3, w // 3:2 * w // 3],
                                      cv2.COLOR_BGR2GRAY)))
    for y0, y1, x0, x1 in ((0, int(.20 * h), 0, int(.55 * w)),      # 左上激光面板 + 红字
                           (0, int(.45 * h), int(.62 * w), w),        # 右上按钮
                           (int(.88 * h), h, 0, w)):                  # 底部状态串
        out[y0:y1, x0:x1] = fill
    return out, fill


class VerboseDetector(RectangleCenterDetector):
    """把 _verify 的逐边数字与拒绝原因全部打出来。"""

    def __init__(self, config):
        super().__init__(config)
        self.rejects = []
        self.accepts = []

    def _verify(self, gray, q, kind):
        result = super()._verify(gray, q, kind)
        self._report(gray, q, kind, result is None)
        return result

    def _report(self, gray, q, kind, rejected):
        cfg = self.cfg
        q = np.asarray(q, np.float32)
        item = {"kind": kind, "rejected": rejected}
        if not self._valid_quad(q, gray.shape):
            item["reason"] = "valid_quad(越界/非凸/边<12px/尖角)"
            self.rejects.append(item)
            return
        w, h, physical = self._layout(q, kind)
        tape = float(cfg.BLACK_FRAME_MM)
        try:
            hg = _homography(physical, q)
            t = self._t
            offsets = tape * np.array([0.20, 0.40, 0.60, 1.22, 1.45])
            p = np.empty((4, len(t), 5, 2), np.float64)
            p[0, :, :, 0] = (tape + t * (w - 2 * tape))[:, None]
            p[0, :, :, 1] = offsets
            p[1, :, :, 0] = w - offsets
            p[1, :, :, 1] = (tape + t * (h - 2 * tape))[:, None]
            p[2, :, :, 0] = (tape + (1 - t) * (w - 2 * tape))[:, None]
            p[2, :, :, 1] = h - offsets
            p[3, :, :, 0] = offsets
            p[3, :, :, 1] = (tape + (1 - t) * (h - 2 * tape))[:, None]
            mapped = project_points(hg, p.reshape(-1, 2)).reshape(p.shape)
        except (ValueError, np.linalg.LinAlgError) as exc:
            item["reason"] = "homography 退化: %s" % exc
            self.rejects.append(item)
            return
        gh, gw = gray.shape
        if (mapped[..., 0].min() < 0 or mapped[..., 1].min() < 0 or
                mapped[..., 0].max() >= gw - 1 or mapped[..., 1].max() >= gh - 1):
            item["reason"] = "采样点落到图像外"
            item["quad"] = np.rint(q).astype(int).tolist()
            self.rejects.append(item)
            return
        samples = _sample(gray, mapped).astype(np.float32)
        dark = np.max(samples[:, :, :3], axis=2)
        bright = np.median(samples[:, :, 3:], axis=2)
        delta = bright - dark
        coverage = ((delta >= cfg.MIN_EDGE_CONTRAST) &
                    (dark <= bright * cfg.MAX_DARK_BRIGHT_RATIO)).mean(axis=1)
        edge_contrast = np.median(delta, axis=1)
        item.update(quad=np.rint(q).astype(int).tolist(), mm=(w, h),
                    edge_contrast=[round(float(x), 1) for x in edge_contrast],
                    coverage=[round(float(x), 2) for x in coverage],
                    dark=[round(float(np.median(dark[i])), 1) for i in range(4)],
                    bright=[round(float(np.median(bright[i])), 1) for i in range(4)])
        if np.count_nonzero(coverage >= cfg.MIN_EDGE_COVERAGE) < 3:
            item["reason"] = "覆盖率达标边数 <3（需 >=%.2f）" % cfg.MIN_EDGE_COVERAGE
        elif coverage.min() < cfg.WEAK_EDGE_COVERAGE:
            item["reason"] = "最弱边覆盖率 <%.2f" % cfg.WEAK_EDGE_COVERAGE
        elif float(np.median(edge_contrast)) < cfg.MIN_EDGE_CONTRAST:
            item["reason"] = "中位对比度 <%.1f" % cfg.MIN_EDGE_CONTRAST
        if rejected:
            self.rejects.append(item)
        else:
            self.accepts.append(item)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--out", default="")
    ap.add_argument("--overlay", action="store_true", help="涂平预览叠加层")
    ap.add_argument("--crop", default="", help="x0,y0,x1,y1 先裁剪")
    ap.add_argument("--scale", type=float, default=1.0, help="裁剪后再缩放（还原相机分辨率）")
    ap.add_argument("--top", type=int, default=6, help="打印前 N 个被拒候选")
    args = ap.parse_args()

    cfg = Config
    bgr = cv2.imread(args.image)
    if bgr is None:
        print("读不到图片: %s" % args.image)
        return 1
    print("原图 %dx%d" % (bgr.shape[1], bgr.shape[0]))
    if args.overlay:
        bgr, fill = flatten_overlays(bgr)
        print("已涂平叠加层（填充灰度 %d）" % fill)
    if args.crop:
        x0, y0, x1, y1 = (int(v) for v in args.crop.split(","))
        bgr = bgr[y0:y1, x0:x1]
        print("裁剪 -> %dx%d" % (bgr.shape[1], bgr.shape[0]))
    if args.scale != 1.0:
        bgr = cv2.resize(bgr, None, fx=args.scale, fy=args.scale,
                         interpolation=cv2.INTER_LINEAR)
        print("缩放 x%.2f -> %dx%d" % (args.scale, bgr.shape[1], bgr.shape[0]))

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    print("灰度 min=%d max=%d mean=%.1f  p5=%.0f p50=%.0f p95=%.0f"
          % (gray.min(), gray.max(), gray.mean(),
             *np.percentile(gray, [5, 50, 95])))
    print("阈值组: DARK_THRESHOLDS=%s CANNY=%s/%s SEARCH_WIDTH=%d"
          % (cfg.DARK_THRESHOLDS, cfg.CANNY_LOW, cfg.CANNY_HIGH, cfg.SEARCH_WIDTH))
    print("校验门: MIN_EDGE_CONTRAST=%.0f MAX_DARK_BRIGHT_RATIO=%.2f "
          "MIN_EDGE_COVERAGE=%.2f WEAK=%.2f"
          % (cfg.MIN_EDGE_CONTRAST, cfg.MAX_DARK_BRIGHT_RATIO,
             cfg.MIN_EDGE_COVERAGE, cfg.WEAK_EDGE_COVERAGE))

    detector = VerboseDetector(cfg)
    result = detector.process(gray)
    print("\n=== process() -> stage=%s valid=%s ===" % (result["stage"], result.get("valid")))
    if result.get("valid"):
        print("  靶心 (%.1f, %.1f)  外框=%s"
              % (result["u"], result["v"], np.rint(result["quad"]).astype(int).tolist()))
    print("  候选验证: 通过 %d / 拒绝 %d" % (len(detector.accepts), len(detector.rejects)))

    if detector.accepts:
        print("\n--- 通过的候选（余量越小越「边缘」，光照/角度稍变就会掉）---")
        for item in detector.accepts:
            print("kind=%s quad=%s mm=%s" % (item["kind"], item.get("quad"), item.get("mm")))
            print("   coverage=%s (门槛 %.2f；缺一条边需另三边 >=%.2f)"
                  % (item["coverage"], cfg.MIN_EDGE_COVERAGE, cfg.STRONG_EDGE_COVERAGE))
            print("   contrast=%s (需中位 >=%.0f)" % (item["edge_contrast"], cfg.MIN_EDGE_CONTRAST))
            print("   dark=%s bright=%s" % (item["dark"], item["bright"]))

    print("\n--- 被拒候选（按面积从大到小，前 %d 个）---" % args.top)
    def area(item):
        q = item.get("quad")
        return -cv2.contourArea(np.asarray(q, np.float32)) if q else 0.0
    for item in sorted(detector.rejects, key=area)[:args.top]:
        print("kind=%s reason=%s" % (item["kind"], item.get("reason")))
        if "coverage" in item:
            print("   quad=%s mm=%s" % (item["quad"], item["mm"]))
            print("   coverage=%s (需>=%.2f)" % (item["coverage"], cfg.MIN_EDGE_COVERAGE))
            print("   contrast=%s (需中位>=%.0f)" % (item["edge_contrast"], cfg.MIN_EDGE_CONTRAST))
            print("   dark=%s bright=%s" % (item["dark"], item["bright"]))

    if args.out:
        canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if result.get("valid"):
            cv2.polylines(canvas, [np.rint(result["quad"]).astype(np.int32)], True, (0, 255, 0), 2)
        for item in detector.rejects[:40]:
            if item.get("quad"):
                cv2.polylines(canvas, [np.asarray(item["quad"], np.int32)], True, (0, 0, 255), 1)
        cv2.imwrite(args.out, canvas)
        print("\n结果图 -> %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
