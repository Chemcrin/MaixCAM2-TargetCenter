"""MaixCAM2实测30+fps：实际相机/检测线程/本地屏/图传，无电机动作。

关闭正在占用相机的main.py后，在MaixVision运行此文件（需要整个项目在板上）。
python tools/benchmark_board.py --duration 20 --output /tmp/board_report.json
默认同时验收图传；仅本地屏可显式指定--local-only。PC不会伪造硬件结果。
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import cv2
import numpy as np
from target_api import load_config
from vision.camera_source import to_gray
from vision.aim_control import VisionControl
from vision.vision_worker import LatestFrameWorker
from vision.preview import Preview, display_result, aligned_frame
from vision.app_runtime import _draw_preview


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--min-fps',type=float,default=30.)
    parser.add_argument('--local-only',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('/tmp/board_report.json'))
    args = parser.parse_args()
    if args.duration < 5: parser.error('duration must be >=5s')
    try:
        from maix import app,camera,image
    except ImportError:
        parser.exit(2,'MaixPy SDK unavailable: run this benchmark on MaixCAM2, not PC.\n')
    cfg = load_config(ENABLE_GIMBAL=False,ENABLE_UART=False,SHOW=True)
    cv2.setNumThreads(cfg.OPENCV_THREADS)
    cam = worker = preview = None
    try:
        cam = camera.Camera(cfg.CAMERA_W,cfg.CAMERA_H,image.Format.FMT_GRAYSCALE,
                            fps=cfg.CAMERA_FPS,buff_num=cfg.CAMERA_BUFFERS)
        cam.skip_frames(cfg.SKIP_FRAMES)
        control = VisionControl(cfg,None,None)
        worker = LatestFrameWorker(control.process,control.idle,True)
        preview = Preview(cfg)
        warmup_until = time.perf_counter()+2.
        started = None
        captures,ages,worker_ms = 0,[],[]
        observed = set()
        while not app.need_exit():
            img = cam.read()
            now = time.perf_counter()
            if worker.error is not None: raise RuntimeError(worker.error)
            if img is None: continue
            if cfg.LENS_CORR: img.lens_corr(strength=cfg.LENS_STRENGTH)
            gray = to_gray(img,image)
            worker.submit(gray,now)
            snapshot,fps,_ = worker.snapshot(time.perf_counter())
            result,laser,age = display_result(snapshot,time.perf_counter(),cfg.RESULT_MAX_AGE_S)
            if preview.due(now):
                _draw_preview(cfg,preview,aligned_frame(snapshot,gray,result),result,laser,age,fps,'BENCHMARK',now)
            if started is None:
                if now < warmup_until: continue
                started = now
                first_valid,first_done,first_show = worker.valid_rate.total,worker.rate.total,preview.shown
                first_stream = preview.stream.metrics()['sent'] if preview.stream else 0
                first_dropped = worker.dropped
            captures += 1
            if snapshot is not None and snapshot['captured_at'] not in observed:
                observed.add(snapshot['captured_at'])
                ages.append((snapshot['completed_at']-snapshot['captured_at'])*1000)
                worker_ms.append(snapshot['detect_ms'])
            if now-started >= args.duration: break
        if started is None: raise RuntimeError('benchmark cancelled during warmup')
        elapsed = time.perf_counter()-started
        stream = preview.stream.metrics() if preview.stream else None
        rates = dict(capture=captures/elapsed,processed=(worker.rate.total-first_done)/elapsed,
                     valid_detection=(worker.valid_rate.total-first_valid)/elapsed,
                     display=(preview.shown-first_show)/elapsed,
                     stream=(stream['sent']-first_stream)/elapsed if stream else None)
        checks = dict(capture=rates['capture']>args.min_fps,
                      valid_detection=rates['valid_detection']>args.min_fps,
                      display=rates['display']>args.min_fps)
        if not args.local_only: checks['stream'] = rates['stream'] is not None and rates['stream']>args.min_fps
        report = dict(hardware=True,seconds=elapsed,requested_camera_fps=cfg.CAMERA_FPS,
                      fps=rates,checks=checks,passed=all(checks.values()),
                      unique_results=len(observed),dropped_vision=worker.dropped-first_dropped,
                      latency_p95_ms=float(np.percentile(ages,95)) if ages else None,
                      detector_p95_ms=float(np.percentile(worker_ms,95)) if worker_ms else None,
                      stream_required=not args.local_only)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
        return 0 if report['passed'] else 1
    finally:
        if worker is not None: worker.close()
        if preview is not None: preview.close()
        if cam is not None: cam.close()


if __name__=='__main__': sys.exit(main())
