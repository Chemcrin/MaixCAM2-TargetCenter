"""同进程交替执行改前/改后窄带跟踪，减少主机负载变化对A/B计时的干扰。"""
import argparse
import json
import subprocess
import sys
import time
import types
from pathlib import Path
import cv2
import numpy as np
from test_support import ROOT, load_config
from test_performance import scene
from vision.rectangle_center import RectangleCenterDetector


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ref',default='efe5a61',help='本次原始源码快照，或其它基线Git提交')
    parser.add_argument('--output',type=Path,default=ROOT/'tools'/'precision_report'/'ab_timing.json')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    for name,path in (('geometry','rect_geometry.py'),('locator','contour_locator.py'),('detector','rectangle_center.py')):
        source = subprocess.check_output(['git','show',args.ref+':maixcam2_rect_center/vision/'+path],cwd=ROOT).decode('utf-8')
        source = source.replace('from .rect_geometry import','from ._review_geometry import')
        source = source.replace('from .contour_locator import','from ._review_locator import')
        module = types.ModuleType('vision._review_'+name)
        sys.modules[module.__name__] = module
        exec(compile(source,path,'exec'),module.__dict__)
    cfg = load_config()
    old = sys.modules['vision._review_detector'].RectangleCenterDetector(cfg)
    new = RectangleCenterDetector(cfg)
    image,_ = scene([[180,70],[445,80],[475,400],[150,420]],noise=True)
    old.process(image); new.process(image)
    values = [[],[]]
    for i in range(400):
        for k,detector in (((0,old),(1,new)) if i%2 else ((1,new),(0,old))):
            started = time.perf_counter()
            result = detector.process(image,timestamp=1+i/40)
            elapsed = (time.perf_counter()-started)*1000
            assert result['valid'] and result['stage']=='track'
            if i>20: values[k].append(elapsed)
    report = {name:dict(median_ms=float(np.median(v)),p95_ms=float(np.percentile(v,95)))
              for name,v in zip(('before','after'),values)}
    report.update(kind='same-process interleaved PC timing; not board FPS',baseline_ref=args.ref)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__': main()
