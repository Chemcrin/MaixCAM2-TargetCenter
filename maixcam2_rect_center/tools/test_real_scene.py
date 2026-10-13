"""真实场景回归：**角落胶带缺失的黑框**必须仍能被检出（对应实机"基本识别不到"）。

背景（2026-10-13 实机排查）：用户实拍画面里黑框清晰可见，程序却一直显示
`NO BLACK RECTANGLE`。逐级诊断（`tools/diagnose_detect.py`）结论是：

  1. 画面中**黑胶带环的右上角是断的**（上边缘约缺 12%、右边缘约缺 53%），
     环不闭合 ⇒ `findContours(RETR_TREE)` 里**没有"亮孔"子轮廓**，且 U 形轮廓的
     `approxPolyDP` 得到 8 个点；
  2. 旧候选生成要求"恰好 4 点凸四边形 + outer 必须有亮孔" ⇒ **一个候选都产不出来**
     （表现为 `_verify` 的调用次数为 0）；
  3. 即使产出正确四边形，旧校验门要求四边覆盖率都 ≥ `WEAK_EDGE_COVERAGE` ⇒
     缺失的那条边（覆盖率仅 0.21）会把唯一正确候选拒掉。

本用例用这张**真实灰度帧**锁住修复结果。对应改动：
  · `vision/rectangle_center.py::_quad_from_contour` —— epsilon 扫描 + 凸包兜底（候选提议）；
  · `_verify` —— 另三条边都达 `STRONG_EDGE_COVERAGE` 时允许一条边缺失；
  · `_refine` —— 某条边无证据时沿用输入四边形的边，而不是整框放弃。

⚠ 不要"简化"上述任何一处来让本用例通过：那不是冗余，正是实机可用性的来源。
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from config import Config                                    # noqa: E402
from vision.rectangle_center import RectangleCenterDetector  # noqa: E402
from run_pc import imread_unicode                            # noqa: E402

FIXTURE = ROOT / "tools" / "real_scene_corner_gap.png"
# 回归锁定值：人工核对确认落在靶纸同心圆圆心（射影不变量的对角线交点）。
EXPECTED_CENTER = (245.1, 161.6)
TOLERANCE_PX = 8.0

# ★ 第二张固件：**板端真机帧**（从 MaixVision 副屏抓取，已抹掉叠加层）。
#   它锁定 2026-10-13 在实机上定位到的真因：掩膜给出的候选四边形**比胶带外沿低约 14px**，
#   `_verify` 固定只从边向里采 0.2~0.6×BLACK_FRAME_MM（≈2~6px）⇒ 采到纸面当"暗带"而误拒；
#   窄半径精修（6/12px）纠不动 ⇒ 画面里黑框清清楚楚却永远 no_black_frame（板端 detect_fps=3.1、
#   结果 age>200ms 全被判 stale）。修复 = 候选被拒时先用 REFINE_RADIUS_WIDE 宽半径精修再验。
BOARD_FIXTURE = ROOT / "tools" / "real_board_frame.png"
BOARD_EXPECTED_CENTER = (208.1, 188.0)
BOARD_TOLERANCE_PX = 8.0


def main():
    bgr = imread_unicode(FIXTURE)
    assert bgr is not None, "缺少回归图 %s" % FIXTURE
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    detector = RectangleCenterDetector(Config)

    first = detector.process(gray)
    assert first.get("valid"), (
        "真实场景（角落胶带缺失）必须检出黑框；stage=%s。"
        "若这里失败，先跑 tools/diagnose_detect.py %s 看 _verify 的逐边覆盖率。"
        % (first.get("stage"), FIXTURE.name))
    error = float(np.linalg.norm([first["u"] - EXPECTED_CENTER[0],
                                  first["v"] - EXPECTED_CENTER[1]]))
    assert error <= TOLERANCE_PX, \
        "中心偏离回归锁定值 %.1fpx（容差 %.1f）：(%.1f, %.1f)" % (
            error, TOLERANCE_PX, first["u"], first["v"])
    print("PASS: 真实场景首帧检出，中心 (%.1f, %.1f)，偏差 %.2fpx，stage=%s"
          % (first["u"], first["v"], error, first["stage"]))

    second = detector.process(gray)
    assert second.get("valid") and second.get("stage") == "track", \
        "第二帧应走窄带跟踪（说明缺一条边时精修没有整框放弃）：stage=%s" % second.get("stage")
    drift = float(np.linalg.norm([second["u"] - first["u"], second["v"] - first["v"]]))
    assert drift < 1.0, "跟踪帧与首帧中心漂移过大：%.2fpx" % drift
    print("PASS: 第二帧走 track，中心漂移 %.2fpx" % drift)

    # ---- 抖动/移动序列：锁上之后**不能很快掉回搜索**（对应实机"偶尔锁上但很快丢"）----
    detector.reset()
    detector.process(gray)
    tracked = 0
    frames = 24
    for i in range(1, frames + 1):
        dx = 3.0 * i                            # 每帧平移 3px，模拟车体/云台抖动
        dy = 1.5 * (i % 3) - 1.5
        matrix = np.float32([[1, 0, dx], [0, 1, dy]])
        moved = cv2.warpAffine(gray, matrix, (gray.shape[1], gray.shape[0]),
                               flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        out = detector.process(moved)
        assert out.get("valid"), "第 %d 帧丢锁（stage=%s）—— 平移 3px/帧不该掉出跟踪" % (
            i, out.get("stage"))
        tracked += out.get("stage") == "track"
    assert tracked >= frames - 2, "平移序列中 track 帧数偏少：%d/%d" % (tracked, frames)
    print("PASS: 平移序列 %d 帧全部锁定（其中 track %d 帧）" % (frames, tracked))

    # ---- 板端真机帧：候选偏离真边时，宽半径兜底必须把它救回来 ----
    board = imread_unicode(BOARD_FIXTURE)
    assert board is not None, "缺少板端固件 %s" % BOARD_FIXTURE
    board_gray = cv2.cvtColor(board, cv2.COLOR_BGR2GRAY)
    detector.reset()
    out = detector.process(board_gray)
    assert out.get("valid"), (
        "板端真机帧必须检出；stage=%s。这张帧的候选四边形比胶带外沿低约 14px，"
        "若这里失败，说明 REFINE_RADIUS_WIDE 的宽半径兜底被去掉了（窄半径纠不动）。"
        % out.get("stage"))
    board_error = float(np.linalg.norm([out["u"] - BOARD_EXPECTED_CENTER[0],
                                        out["v"] - BOARD_EXPECTED_CENTER[1]]))
    assert board_error <= BOARD_TOLERANCE_PX, \
        "板端帧中心偏离锁定值 %.1fpx：(%.1f, %.1f)" % (board_error, out["u"], out["v"])
    print("PASS: 板端真机帧检出，中心 (%.1f, %.1f)，偏差 %.2fpx" % (out["u"], out["v"], board_error))

    # ---- 正视图：透视变换方向必须自洽（方向反了会得到"一整片背景"，肉眼还未必立刻发现）----
    # 契约：正视图 = 物理靶纸本身，所以**四周应当是胶带暗带、中间应当是纸面亮区**；
    # 剖面法（灰度对称性）算出的中心必须与解析解（四角几何）接近 —— 两法是独立估计。
    detector.reset()
    out = detector.process(board_gray)
    assert out.get("valid"), "正视图用例需要先检出板端帧"
    view, _, _ = detector.front_view(board_gray, out, short_px=256)
    assert view is not None, "正视图必须能生成（warp 矩阵退化会让现场看不到校验图）"
    border = float(np.median(np.concatenate([view[0, :], view[-1, :], view[:, 0], view[:, -1]])))
    middle = float(np.median(view[view.shape[0] // 4:3 * view.shape[0] // 4,
                                  view.shape[1] // 4:3 * view.shape[1] // 4]))
    assert border < middle - 40, (
        "正视图四周必须比中间暗（四周=胶带、中间=纸面）：border=%.0f middle=%.0f。"
        "若 border≈middle≈背景灰度，说明 warp_front_view 的单应方向写反了。" % (border, middle))
    alt = detector.front_view_center(board_gray, out, short_px=256)
    assert alt is not None, "剖面法中心必须可用（正视图里应能看到两侧胶带暗带）"
    agree = float(np.hypot(alt[0] - out["u"], alt[1] - out["v"]))
    assert agree < 6.0, ("剖面法与解析解的中心偏差 %.2fpx 过大（两者独立，"
                         "超限说明四角几何或光照有问题）" % agree)
    print("PASS: 正视图自洽（周边暗 %.0f < 中间 %.0f），剖面中心与解析解差 %.2fpx"
          % (border, middle, agree))

    print("real scene regression: PASS (5)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
