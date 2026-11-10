"""MaixCAM2 entry point for perspective robust black rectangle center detection."""

import os
import sys
import time

import cv2
import numpy as np


def _root():
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except NameError:
        return os.getcwd()


ROOT = _root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import Config
from vision.rectangle_center import RectangleCenterDetector, draw_result
from vision.aiming import LaserSpotDetector, add_aiming, draw_aiming, pack_aiming

BUILD_ID = "2026-11-10-fast-perspective-v1"


def _to_gray(img, image_module):
    a = image_module.image2cv(img, ensure_bgr=False, copy=False)
    if a.ndim == 2:
        return a
    if a.ndim == 3 and a.shape[2] == 1:
        return a[:, :, 0]
    if a.ndim == 3 and a.shape[2] == 3:
        return cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
    if a.ndim == 3 and a.shape[2] == 4:
        return cv2.cvtColor(a, cv2.COLOR_RGBA2GRAY)
    b = image_module.image2cv(img, ensure_bgr=True, copy=True)
    if b.ndim == 2:
        return b
    return cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)


def _send_uart(uart, result):
    if uart is None:
        return
    uart.write(pack_aiming(result, Config.UART_PROTOCOL))


def main():
    from maix import app, camera, display, image

    cv2.setNumThreads(Config.OPENCV_THREADS)
    cam = camera.Camera(Config.CAMERA_W, Config.CAMERA_H,
                        image.Format.FMT_GRAYSCALE,
                        fps=Config.CAMERA_FPS,
                        buff_num=Config.CAMERA_BUFFERS)
    cam.skip_frames(Config.SKIP_FRAMES)
    disp = display.Display() if Config.SHOW else None
    stream = None                          # MaixVision 图传（独立进程，首帧时按实际画布尺寸拉起）
    uart = None
    if Config.ENABLE_UART:
        try:
            from maix import uart as maix_uart
            uart = maix_uart.UART(Config.UART_PORT, Config.UART_BAUD)
        except Exception as exc:
            print("[uart] open failed: %s" % exc)
    detector = RectangleCenterDetector(Config)
    laser_detector = LaserSpotDetector(Config)
    last = time.perf_counter()
    last_log = last
    last_show = -1e10
    lens_enabled = Config.LENS_CORR
    totals = dict(read=0.0, lens=0.0, bridge=0.0, detect=0.0, uart=0.0, show=0.0, loop=0.0)
    count, searches = 0, 0
    fps = 0.0
    mean_dt = None
    print("[rect-center] %s search_width=%d display_width=%d display_fps=%d" %
          (BUILD_ID, Config.SEARCH_WIDTH, Config.SHOW_WIDTH, Config.SHOW_FPS))
    print("[rect-center] 低分辨率Otsu/自适应轮廓 / ROI跟踪 / 四边精修 / 单应性中心")
    print("[aim] 激光参考=%s 标定图=%s calibrated=%s UART=%s; dx向右为正 dy向下为正" %
          (Config.LASER_PIXEL, Config.LASER_REFERENCE_SIZE,
           Config.LASER_CALIBRATED, Config.UART_PROTOCOL))
    while not app.need_exit():
        start = time.perf_counter()
        img = cam.read()
        if img is None:
            continue
        after_read = time.perf_counter()
        if lens_enabled:
            try:
                corrected = img.lens_corr(strength=Config.LENS_STRENGTH)
                if corrected is not None:
                    img = corrected
            except Exception as exc:
                lens_enabled = False
                print("[lens] 本次会话停用不支持的 lens_corr: %s" % exc)
        after_lens = time.perf_counter()
        gray = _to_gray(img, image)
        after_bridge = time.perf_counter()
        result = detector.process(gray)
        spot = laser_detector.process(gray) if Config.LASER_MODE == "detect" else None
        add_aiming(result, gray.shape, Config, spot)
        after_detect = time.perf_counter()
        _send_uart(uart, result)
        after_uart = time.perf_counter()
        if disp is not None and (after_uart - last_show >= 1.0 / max(1, Config.SHOW_FPS)):
            last_show = after_uart
            view = draw_result(gray, result, Config.SHOW_WIDTH)
            draw_aiming(view, result, gray.shape)
            cv2.putText(view, "FPS %.1f" % fps, (12, view.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            disp.show(image.cv2image(view, bgr=True, copy=True))
            if getattr(Config, "SHOW_STREAM", False):
                if stream is None:
                    try:
                        from vision.maixvision_stream import MaixVisionStream
                        stream = MaixVisionStream(Config, view.shape[1], view.shape[0], clock=time)
                        print("[stream] MaixVision 图传已启动（独立进程，%dx%d）%s"
                              % (view.shape[1], view.shape[0],
                                 "" if getattr(stream, "enabled", False) else " —— 未启用"))
                    except Exception as exc:
                        stream = None
                        print("[stream] 图传启动失败（只影响 MaixVision 画面）: %s" % exc)
                if stream is not None:
                    try:
                        stream.submit(view)
                    except Exception as exc:
                        print("[stream] 投递失败，停用图传: %s" % exc)
                        try:
                            stream.close()
                        except Exception:
                            pass
                        stream = None
        now = time.perf_counter()
        dt = now - last
        last = now
        if dt > 1e-4:
            mean_dt = dt if mean_dt is None else 0.9 * mean_dt + 0.1 * dt
            fps = 1.0 / mean_dt
        phases = (after_read - start, after_lens - after_read,
                  after_bridge - after_lens, after_detect - after_bridge,
                  after_uart - after_detect, now - after_uart, now - start)
        for key, elapsed in zip(totals, phases):
            totals[key] += elapsed * 1000.0
        count += 1
        searches += result.get("search_count", 0)
        if now - last_log > Config.LOG_INTERVAL:
            window_fps = count / max(1e-6, now - last_log)
            last_log = now
            print("[perf] %s valid=%s FPS=%.1f stage=%s frames=%d searches=%d "
                  "read=%.1f lens=%.1f bridge=%.1f detect=%.1f uart=%.1f show=%.1f loop=%.1f ms" %
                  ((BUILD_ID, result["valid"], window_fps, result["stage"], count, searches) +
                   tuple(totals[k] / count for k in totals)))
            if result["aim_valid"]:
                print("[aim] L=(%.1f,%.1f) T=(%.1f,%.1f) dx=%+.1f dy=%+.1f px calibrated=%s" %
                      (result["laser_u"], result["laser_v"], result["u"], result["v"],
                       result["dx"], result["dy"], result["laser_calibrated"]))
            elif result["laser_valid"]:
                print("[aim] L=(%.1f,%.1f) no_target dx=-- dy=-- calibrated=%s" %
                      (result["laser_u"], result["laser_v"], result["laser_calibrated"]))
            else:
                print("[aim] laser=%s target_valid=%s dx=-- dy=--" %
                      (result["laser_source"], result["valid"]))
            count, searches = 0, 0
            for key in totals:
                totals[key] = 0.0
    try:
        cam.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
