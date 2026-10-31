"""把录像回放里的失败帧归因到 搜索 / 验证 / 精修 三层，给调参提供方向。

run_video.py 只报 valid 与 stage，但 process() 在任何失败分支都写 `no_black_frame`，
看不出"到底搜过没搜过、卡在哪一条判据"。本脚本**不改被测代码**，只做两件事：

1) 包住 locate/_verify/_refine 计数，判断本次失败是"没提案"还是"提案被拒"；
2) 对失败帧做尸检：拿上一帧的锁定框重跑一次窄带精修 + 带 diagnostic 的验证，
   把四边覆盖率/对比度/是否裁边打出来，指出死在哪个门。

归因口径：
    cold_no_proposal   无锁定，contour_locator 一个候选都没给
    cold_verify        无锁定，给了候选但验证不过
    track_refine       有锁定，窄带精修凑不出 >=3 条实测边
    track_coverage     精修出框，但有 >1 条边覆盖率不达标（手挡/贴边）
    track_contrast     覆盖率够，但暗带对比度不够（曝光/虚焦）
    track_u_shape      画面内某条边几乎没有证据（U 形/窗框）
    track_geometry     四边形体检不过（出画/退化）
    track_area         强证据通过，但复锁面积门（REACQUIRE_AREA_RATIO）拒了

用法：
    python -B tools/diagnose_video.py --video ../vest视频/maixvision.mp4
    python -B tools/diagnose_video.py --video clip.mp4 --crop 0,0,640,480
"""
import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REPORTS = ROOT.parent / "reports"   # 产物落在应用目录外，避免 MaixVision 打包超 30MB
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2  # noqa: E402

from config import Config  # noqa: E402
from vision.rectangle_center import RectangleCenterDetector  # noqa: E402
from run_video import PANES, crop_rect, parse_crop, parse_size  # noqa: E402


def classify_gate(diag, cfg):
    """把一次 diagnostic 验证的结果翻译成"死在哪个门"。"""
    cov = diag.get("coverage")
    if cov is None:
        return "track_geometry", {}
    cov = np.asarray(cov, float)
    in_frame = np.asarray(diag.get("in_frame", [True] * 4), bool)
    exempt = np.asarray(diag.get("exempt", [False] * 4), bool)
    usable = in_frame & ~exempt
    contrast = np.asarray(diag.get("contrast", [0.] * 4), float)
    need = float(cfg.MIN_EDGE_COVERAGE)
    strong = int(np.count_nonzero(in_frame & (cov >= need)))
    info = dict(cov=[round(float(c), 3) for c in cov],
                contrast=[round(float(c), 1) for c in contrast],
                strong=strong)
    if np.count_nonzero(~in_frame) > int(getattr(cfg, "MAX_CLIPPED_EDGES", 1)):
        return "track_geometry", info
    if strong < 3:
        return "track_coverage", info
    if usable.any() and float(np.median(contrast[usable])) < cfg.MIN_EDGE_CONTRAST:
        return "track_contrast", info
    if np.any(usable & (cov < getattr(cfg, "MIN_OBSERVED_EDGE_COVERAGE", .125))):
        return "track_u_shape", info
    return "track_other", info


def postmortem(det, gray, stage_ms):
    """上一帧的锁定框还在，为什么这一帧没锁上？"""
    track = det._track
    if track is None:
        return None, {}
    kind = track["kind"]
    q = det._refine(gray, track["quad"], kind, det.cfg.TRACK_RADIUS, require_all=False)
    radius = det.cfg.TRACK_RADIUS
    if q is None:
        wide = getattr(det.cfg, "REFINE_RADIUS_WIDE", 20)
        q = det._refine(gray, track["quad"], kind, wide, require_all=False)
        radius = wide
    if q is None:
        LF = getattr(det, "last_refine", None) or {}
        return "track_refine", dict(radius=radius, refined=int(det._refined_edges),
                                    weak=int(det._weak_edges), good=LF.get("good"),
                                    measured=LF.get("measured"))
    det._verify(gray, q, kind, diagnostic=True, exempt_edges=det._fallback_edges)
    diag = getattr(det, "last_diagnostic", {}) or {}
    reason, info = classify_gate(diag, det.cfg)
    info["refined"] = int(det._refined_edges)
    info["weak"] = int(det._weak_edges)
    return reason, info


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--video", required=True)
    ap.add_argument("--pane", choices=sorted(PANES), default="maixvision")
    ap.add_argument("--crop", type=parse_crop, default=None)
    ap.add_argument("--size", type=parse_size, default=(640, 0),
                   help="WxH；H=0 按裁切框比例推算")
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    src = Path(args.video)
    if not src.is_file():
        raise SystemExit("video not found: %s" % src)
    cap = cv2.VideoCapture(str(src))
    if not cap.isOpened():
        raise SystemExit("cannot open video: %s" % src)
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    ok, probe = cap.read()
    if not ok:
        raise SystemExit("cannot read the first frame")
    x0, y0, x1, y1 = crop_rect(probe, args)
    if args.size[1] <= 0:
        w = args.size[0]
        args.size = (w, max(2, int(round(w * (y1 - y0) / float(x1 - x0)))))
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    det = RectangleCenterDetector(Config)
    reasons = Counter()
    builtin = Counter()   # 检测器自己给的 fail_stage/fail_reason，板端日志同口径
    losses, run_start, run_len = [], None, 0
    samples, gate_rows = [], []
    frame = processed = valid = 0
    while True:
        ok, raw = cap.read()
        if not ok:
            break
        t = frame / fps
        frame += 1
        if (frame - 1) % max(1, args.stride):
            continue
        pane = raw[y0:y1, x0:x1]
        gray = cv2.cvtColor(pane, cv2.COLOR_BGR2GRAY)
        if gray.shape[::-1] != args.size:
            gray = cv2.resize(gray, args.size, interpolation=cv2.INTER_CUBIC)
        cold = det._track is None
        result = det.process(gray, timestamp=t)
        processed += 1
        if result.get("valid"):
            valid += 1
            if run_len:
                losses.append((run_start, run_len))
            run_start, run_len = None, 0
            continue
        if run_start is None:
            run_start, run_len = t, 0
        run_len += 1
        builtin['%s/%s' % (result.get('fail_stage'), result.get('fail_reason'))] += 1
        reason, info = postmortem(det, gray, result.get("detector_ms", 0.))
        if reason is None:
            reason = "cold_no_proposal" if cold else "cold_verify"
        elif cold:
            reason = "cold_" + reason.split("_", 1)[1] if "_" in reason else "cold_" + reason
        reasons[reason] += 1
        if info and len(gate_rows) < 600:
            gate_rows.append(dict(t=round(t, 2), reason=reason, **info))
        if len(samples) < 12 and processed % max(1, (1022 if not args.limit else args.limit) // 12) == 0:
            samples.append(dict(t=round(t, 2), reason=reason,
                                detector_ms=round(result.get("detector_ms", 0.), 2)))
    if run_len:
        losses.append((run_start, run_len))
    cap.release()

    print("帧数 %d  有效 %d (%.1f%%)  失败 %d"
          % (processed, valid, 100.0 * valid / max(1, processed), processed - valid))
    print("\n内置归因（result.fail_stage/fail_reason，与板端 [perf] det_fail 同口径）：")
    for key, n in builtin.most_common():
        print("  %-28s %5d" % (key, n))
    print("\n尸检归因（再跑一次精修，带边级细节）：")
    for key, n in reasons.most_common():
        print("  %-20s %5d  (%5.1f%% of failures)" % (key, n, 100.0 * n / max(1, processed - valid)))
    if losses:
        losses.sort(key=lambda r: -r[1])
        print("\n最长失锁段（起点 s / 帧数）：%s"
              % ", ".join("%.2f / %d" % r for r in losses[:6]))
    cov_rows = [r for r in gate_rows if r.get("strong") is not None]
    if cov_rows:
        print("\n达标边数分布（需 >=3；样本 %d）：%s"
              % (len(cov_rows), dict(sorted(Counter(r["strong"] for r in cov_rows).items()))))
        near = [r for r in cov_rows if r["strong"] == 2]
        if near:
            print("  只差一条边的帧 %d 个，示例：" % len(near))
            for r in near[:6]:
                print("    t=%6.2fs cov=%s refined=%d weak=%d"
                      % (r["t"], r["cov"], r.get("refined_edges", -1), r.get("weak_edges", -1)))

    out = Path(args.out) if args.out else REPORTS / "video" / (src.stem + "-diag")
    out.mkdir(parents=True, exist_ok=True)
    (out / "diagnose.json").write_text(json.dumps(dict(
        video=str(src), processed=processed, valid=valid,
        reasons=dict(reasons), builtin=dict(builtin), losses=[dict(start=r[0], frames=r[1]) for r in losses],
        gates=gate_rows), ensure_ascii=False, indent=1), encoding="utf-8")
    print("\n明细 -> %s" % (out / "diagnose.json"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
