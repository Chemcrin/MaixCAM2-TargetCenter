"""周期日志组装（从 main.py 抽出，v0.4.6 模块化）。

只做字符串拼装，不碰任何硬件/时间源 —— 便于单独审阅"日志口径"是否与代码一致。
★ 日志文案逐字冻结（tools/test_main_self_test.test_perf_log_branch_formatting 会校验字段），
  改这里等于改板端日志格式，务必同步 README 的读法说明。
"""
# 主循环单轮超过该毫秒数（不含 show）即判为"慢"，打一次归因提示。
MAIN_LOOP_SLOW_MS = 66.0


def format_perf_line(build_id, capture_fps, display_fps, display_low1, vision_fps,
                     result, read_avg, draw_avg, loop_avg,
                     show_avg, show_p95, show_max, show_n,
                     age_s, shown, drop_show, drop_vision, detector=None):
    """拼装 [perf] 行。

    读法：[perf] 里 capture / disp / detect_fps 是三路**不同**口径 ——
      "眼睛看到的刷新率"看 disp，"识别吞吐"看 detect_fps，别互相换算。
      read/draw/loop 属于**主循环**，show 属于**显示线程** —— 分开看就能一眼判定
      "到底谁在吃时间"（这正是 1.5fps 归因的关键）。

    ★ `det_*` 是**检测器自己的**结果，与前面的 valid/stage **不是一回事**：
      valid/stage 取自显示口径，结果过期时会被 `display_result` 整体替换成
      `{valid:false, stage:"stale"}` —— 于是 `stage=stale` 会掩盖检测器真实的
      track/roi_search/full_search/no_black_frame。2026-10-13 板端排查时正是被这一点
      挡住：分不清"检测成功但太慢"（改性能/新鲜度）与"检测失败"（改算法），而两者修法相反。
      det_stage / det_valid / det_ms 三个字段就是那次诊断盲区的对症修复。
    """
    detector = detector or {}
    return ("[perf] %s capture=%.1f disp=%.1f low1=%.1f detect_fps=%.1f "
            "valid=%s stage=%s det_valid=%s det_stage=%s det_ms=%.1f | "
            "read=%.1f draw=%.1f loop=%.1f ms | "
            "show avg=%.1f p95=%.1f max=%.1fms n=%d | "
            "age=%.0fms shown=%d drop(show)=%d drop(vision)=%d" %
            (build_id, capture_fps, display_fps, display_low1, vision_fps,
             result.get("valid", False), result.get("stage"),
             detector.get("valid", False), detector.get("stage"),
             float(detector.get("detector_ms", 0.0)),
             read_avg, draw_avg, loop_avg,
             show_avg, show_p95, show_max, show_n,
             age_s * 1000, shown, drop_show, drop_vision))


def format_slow_loop(loop_avg):
    """主循环偏慢（且此数**不含 show**）时的归因提示。"""
    return ("[preview] main loop slow: loop=%.0fms（不含 show）；看 read= 是否偏大；"
            "若 show 也恒定 ~505ms 则是图传忙等抢 CPU" % loop_avg)
