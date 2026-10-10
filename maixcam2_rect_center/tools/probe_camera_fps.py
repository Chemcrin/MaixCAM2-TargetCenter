"""相机真实帧率探针 —— 只测「取帧」与「显示」两步，不含任何识别/跟踪/云台。

目的：把"帧率低"的归属**一次性钉死**，不再靠推测。它会分两段测量：

    阶段 A：只 `cam.read()`（**完全不显示**）      -> 相机与取帧的真实帧率
    阶段 B：`cam.read()` + 每帧 `Display.show()`   -> 加上显示后的帧率，并单列 show 耗时

判读：
    * A 快（~30fps）而 B 慢          -> 瓶颈在 **Display.show**（显示/上位机链路），与相机无关；
    * A 本身就慢（1~2fps）           -> 瓶颈在 **相机取帧**（曝光/分辨率模式/驱动），与显示无关；
    * A、B 都快                       -> 瓶颈在主循环里的**其它环节**（识别/云台/UART）。

用法（在 MaixVision 里打开本文件 → 「运行当前文件」即可；默认测 60 帧）：
    直接运行即可；想改帧数就把下面 N_FRAMES 改掉。

注意：本文件是**诊断工具**，不是应用的一部分（`tools/` 已在 app.yaml 的 exclude 里，
不会进应用包）。
"""

import time

N_FRAMES = 60          # 每段测量的帧数
CAM_W, CAM_H = 640, 480
CAM_FPS = 30           # 与主工程 Config.CAMERA_FPS 一致
CAM_BUFF = 2
SKIP = 10


def _new_cam(camera, image):
    cam = camera.Camera(CAM_W, CAM_H, image.Format.FMT_GRAYSCALE,
                        fps=CAM_FPS, buff_num=CAM_BUFF)
    cam.skip_frames(SKIP)
    return cam


def _measure(cam, n, on_frame=None):
    """返回 (frames, total_s, read_ms_list, interval_ms_list, extra_ms_list)。"""
    reads, intervals, extras = [], [], []
    t_prev = None
    t0 = time.perf_counter()
    frames = 0
    while frames < n:
        ta = time.perf_counter()
        img = cam.read()
        tb = time.perf_counter()
        if img is None:
            continue
        reads.append((tb - ta) * 1000.0)
        if t_prev is not None:
            intervals.append((tb - t_prev) * 1000.0)
        t_prev = tb
        if on_frame is not None:
            te0 = time.perf_counter()
            on_frame(img)
            extras.append((time.perf_counter() - te0) * 1000.0)
        frames += 1
    return frames, time.perf_counter() - t0, reads, intervals, extras


def _report(tag, frames, total, reads, intervals, extras=None):
    def avg(v):
        return sum(v) / len(v) if v else 0.0

    def mx(v):
        return max(v) if v else 0.0

    fps = frames / total if total > 0 else 0.0
    print("[probe] %-22s frames=%d 用时=%.2fs  实测帧率=%.1ffps" % (tag, frames, total, fps))
    print("[probe]   read  ms: 均=%.1f 最大=%.1f" % (avg(reads), mx(reads)))
    print("[probe]   帧间隔 ms: 均=%.1f 最大=%.1f" % (avg(intervals), mx(intervals)))
    if extras is not None:
        print("[probe]   附加步 ms: 均=%.1f 最大=%.1f" % (avg(extras), mx(extras)))
    return fps


def main():
    from maix import camera, image

    print("=" * 66)
    print("[probe] 相机 %dx%d 灰度 @%d fps（与主工程同参数）；每段 %d 帧"
          % (CAM_W, CAM_H, CAM_FPS, N_FRAMES))
    print("=" * 66)

    # ---------- 阶段 A：只取帧，不做任何显示 ----------
    cam = _new_cam(camera, image)
    _measure(cam, 10)                      # 预热
    frames_a, total_a, reads_a, ints_a, _ = _measure(cam, N_FRAMES)
    cam.close()
    fps_a = _report("A) 只取帧（无显示）", frames_a, total_a, reads_a, ints_a)

    # ---------- 阶段 B：取帧 + 每帧显示 ----------
    cam = _new_cam(camera, image)
    _measure(cam, 10)                      # 预热
    disp = None
    disp_err = None
    try:
        from maix import display
        disp = display.Display()
        show = lambda img: disp.show(img)      # noqa: E731
    except Exception as exc:               # 显示不可用则跳过阶段 B
        disp_err = exc
        show = None

    if show is None:
        cam.close()
        print("[probe] 阶段 B 跳过：Display 初始化失败（%s）" % disp_err)
    else:
        try:
            frames_b, total_b, reads_b, ints_b, extras_b = _measure(cam, N_FRAMES, on_frame=show)
            fps_b = _report("B) 取帧 + 每帧显示", frames_b, total_b, reads_b, ints_b, extras_b)
        finally:
            try:
                disp.close()
            except Exception:
                pass
            cam.close()

        print("-" * 66)
        print("[probe] 结论：")
        if fps_a > fps_b * 2 and fps_b < 15:
            print("[probe]   A(%.1ffps) 远快于 B(%.1ffps) -> **瓶颈在 Display.show**"
                  "（相机没问题，显示/上位机链路把主循环拖慢）" % (fps_a, fps_b))
        elif fps_a < 15:
            print("[probe]   A 本身只有 %.1ffps -> **瓶颈在相机取帧**"
                  "（查曝光/分辨率模式/驱动；显示无关）" % fps_a)
        else:
            print("[probe]   A、B 都不慢（A=%.1ffps B=%.1ffps）-> 瓶颈在识别/云台/UART 等其它环节，"
                  "请把本输出发回分析" % (fps_a, fps_b))
        print("[probe]   （若你在用 MaixVision，阶段 B 的 show 会包含上位机链路开销；"
              "不接 MaixVision 再跑一次可对比）")


if __name__ == "__main__":
    main()
