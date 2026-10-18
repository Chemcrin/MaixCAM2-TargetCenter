"""独立生成抗锯齿靶图，逐帧验证中心/固定激光偏差，并导出可复查报告。

python -B tools/verify_precision.py --output tools/precision_report
不重试漏检帧；真值只用 OpenCV 四点投影，不调用被测几何代码。
"""
import argparse
import html
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import Config
from target_api import CenterTracker
from vision.rectangle_center import draw_result


def render(quad, variant='clean', seed=0):
    """4×面积积分近似像素曝光；靶纸边界是物理0..210、0..297mm。"""
    ss = 4
    matrix = cv2.getPerspectiveTransform(np.float32([[0,0],[210,0],[210,297],[0,297]]), np.float32(quad))
    target = np.full((297*ss,210*ss), 20, np.uint8)
    target[18*ss:-18*ss,18*ss:-18*ss] = 220
    cv2.circle(target, (105*ss,round(148.5*ss)),38*ss,160,ss)
    if variant == 'glare':
        cv2.ellipse(target,(90*ss,11*ss),(22*ss,13*ss),0,0,360,240,-1)
    bg = 20 if variant == 'dark_bg' else 175
    to_mm = np.array([[1/ss,0,.5/ss],[0,1/ss,.5/ss],[0,0,1.]])
    to_high = np.array([[ss,0,(ss-1)/2],[0,ss,(ss-1)/2],[0,0,1.]])
    high = cv2.warpPerspective(target, to_high@matrix@to_mm, (640*ss,480*ss),
                               flags=cv2.INTER_NEAREST,borderValue=bg)
    image = cv2.resize(high,(640,480),interpolation=cv2.INTER_AREA)
    if variant == 'blur': image = cv2.GaussianBlur(image,(7,7),1.2)
    if variant == 'motion_blur': image = cv2.filter2D(image,-1,np.ones((1,5),np.float32)/5)
    if variant == 'low_contrast': image = np.rint(image*.30+85).astype(np.uint8)
    if variant == 'gradient':
        image = np.clip(image.astype(float)*np.linspace(.60,1.,640)[None,:],0,255).astype(np.uint8)
    noise = np.random.default_rng(seed).normal(0,1.,image.shape)
    image = np.clip(image.astype(float)+noise,0,255).astype(np.uint8)
    truth = cv2.perspectiveTransform(np.float64([[[105,148.5]]]),matrix).reshape(2)
    return image, truth


def cases():
    variants = ('clean','dark_bg','blur','motion_blur','low_contrast','gradient','glare')
    for i in range(56):
        angle = np.deg2rad((0,17,42,45,71,90,135,178)[i%8])
        rot = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        width = (115,155,195,225)[(i//8)%4]
        quad = np.array([[-.5,-.5],[.45,-.44],[.54,.5],[-.58,.56]])*[width,width*297/210]
        quad = quad@rot.T+[320+.37*(i%3),240+.23*(i%5)]
        yield '%02d_%s'%(i,variants[i%7]), render(quad,variants[i%7],i)
    for i in range(16):
        width = (48,60,75,90)[i//4]
        angle = np.deg2rad((0,25,45,90)[i%4])
        rot = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        quad = np.array([[-.5,-.5],[.45,-.44],[.54,.5],[-.58,.56]])*[width,width*297/210]
        yield '%02d_far_%d'%(56+i,width), render(quad@rot.T+[320.3,240.1],'blur',i)
    for i,quad in enumerate(([[280,120],[360,150],[500,390],[120,410]],
                             [[120,190],[500,150],[400,320],[210,340]],
                             [[35,35],[200,60],[210,290],[25,300]],
                             [[410,185],[605,170],[610,450],[400,435]])):
        yield '%02d_perspective'% (72+i), render(quad,'gradient',i)


def stats(values):
    a = np.asarray(values,float)
    return dict(median_ms=round(float(np.median(a)),3),p95_ms=round(float(np.percentile(a,95)),3),
                max_ms=round(float(a.max()),3)) if len(a) else {}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'tools'/'precision_report')
    parser.add_argument('--record-only',action='store_true',help='记录改动前基线，不执行验收断言')
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(Config.OPENCV_THREADS)
    rows, tiles = [], []
    for name,(image,truth) in cases():
        tracker = CenterTracker(with_laser=True)
        result = tracker.track(image,1.)
        error = float(np.hypot(result.raw_u-truth[0],result.raw_v-truth[1])) if result else None
        dv_error = float(np.linalg.norm(np.array([result.dx,result.dy])-(truth-[Config.LASER_U_NOMINAL,Config.LASER_V_NOMINAL]))) if result else None
        row = dict(case=name,valid=bool(result),error_px=error,deviation_error_px=dv_error,
                   stage=result.stage,ms=result.ms,masks=result.get('masks',0))
        rows.append(row)
        view = draw_result(image,result,640)
        cv2.drawMarker(view,tuple(np.rint(truth).astype(int)),(255,255,255),cv2.MARKER_TILTED_CROSS,14,1)
        cv2.putText(view,name+' err='+('MISS' if error is None else '%.3fpx'%error),(8,458),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,200,255),1)
        cv2.imencode('.png',image)[1].tofile(str(args.output/(name+'_input.png')))
        cv2.imencode('.png',view)[1].tofile(str(args.output/(name+'_result.png')))
        tiles.append(cv2.resize(view,(320,240)))

    tracker = CenterTracker(with_laser=True)
    movement, times = [], []
    base = np.array([[205.,135.],[335.,125.],[355.,310.],[195.,320.]])
    for i in range(120):
        dx = 90*np.sin(i*.12)
        dy = 18*np.sin(i*.09)
        image,truth = render(base+[dx,dy],'blur',i)
        start = time.perf_counter()
        r = tracker.track(image,2+i/40)
        times.append((time.perf_counter()-start)*1000)
        movement.append(dict(valid=bool(r),stage=r.stage,masks=r.get('masks',0),
                             error_px=float(np.hypot(r.raw_u-truth[0],r.raw_v-truth[1])) if r else None))

    # 一个真实像素画面只测一次；包括真实负样本、实心块、开口U形、椭圆和纯噪声。
    negatives = [np.full((480,640),v,np.uint8) for v in (20,120,220)]
    for kind in ('solid','u_shape','ellipse','noise'):
        im = np.full((480,640),180,np.uint8)
        if kind == 'solid': cv2.rectangle(im,(150,70),(450,410),20,-1)
        elif kind == 'u_shape':
            cv2.rectangle(im,(180,70),(400,390),20,18)
            cv2.rectangle(im,(150,40),(430,100),180,-1)
        elif kind == 'ellipse': cv2.ellipse(im,(320,240),(100,150),20,0,360,20,18)
        else: im = np.random.default_rng(42).integers(0,256,im.shape,dtype=np.uint8)
        negatives.append(im)
    for name in ('negative_e421.png','negative_e880.png'):
        negatives.append(cv2.imdecode(np.fromfile(str(ROOT/'tools'/name),np.uint8),cv2.IMREAD_GRAYSCALE))
    negative_rows = []
    for i,im in enumerate(negatives):
        r = CenterTracker().track(im)
        name = 'negative_%02d_%s' % (i,('flat20','flat120','flat220','solid','u_shape','ellipse','noise','photo_e421','photo_e880')[i])
        view = draw_result(im,r,640)
        cv2.imencode('.png',im)[1].tofile(str(args.output/(name+'_input.png')))
        cv2.imencode('.png',view)[1].tofile(str(args.output/(name+'_result.png')))
        negative_rows.append(dict(case=name,accepted=bool(r),masks=r.get('masks',0)))
    false_positives = sum(r['accepted'] for r in negative_rows)
    errors = [r['error_px'] for r in rows if r['valid']]
    motion_errors = [r['error_px'] for r in movement if r['valid']]
    summary = dict(kind='PC synthetic; hardware FPS not measured',single_frame_cases=len(rows),
                   detected=len(errors),max_error_px=max(errors,default=999.),
                   p95_error_px=float(np.percentile(errors,95)),negative_cases=len(negatives),false_positives=false_positives,
                   motion_frames=len(movement),motion_detected=len(motion_errors),motion_max_error_px=max(motion_errors,default=999.),
                   motion_tracks=sum(r['stage']=='track' for r in movement),motion_timing=stats(times),
                   max_search_masks=max(r['masks'] for r in rows),max_motion_masks=max(r['masks'] for r in movement),
                   search_timing=stats([r['ms'] for r in rows]))
    (args.output/'results.json').write_text(json.dumps(dict(summary=summary,cases=rows,movement=movement,negatives=negative_rows),ensure_ascii=False,indent=2),encoding='utf-8')
    montage = np.vstack([np.hstack(tiles[i:i+4]) for i in range(0,len(tiles),4)])
    cv2.imencode('.jpg',montage,[cv2.IMWRITE_JPEG_QUALITY,85])[1].tofile(str(args.output/'overview.jpg'))
    body = ''.join('<tr><td>%s</td><td>%s</td><td>%s</td><td>%.2f</td><td><a href="%s_result.png">查看</a></td></tr>' %
                   (r['case'],r['valid'],'MISS' if r['error_px'] is None else '%.3f'%r['error_px'],r['ms'],r['case']) for r in rows)
    neg_body = ''.join('<tr><td>%s</td><td>%s</td><td><a href="%s_result.png">查看</a></td></tr>' %
                       (r['case'],'误检' if r['accepted'] else '正确拒绝',r['case']) for r in negative_rows)
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>矩形中心验证</title><style>body{font:16px system-ui;max-width:1200px;margin:30px auto;background:#fafafa;color:#123}table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccd;padding:8px}img{width:100%}pre{white-space:pre-wrap}</style><h1>矩形中心与固定激光偏差验证</h1><p>独立4倍采样生成输入；每张图只检测一帧。白色叉号为真值，蓝色十字为测量中心。以下耗时来自PC，不代表MaixCAM2帧率。</p><pre>'+html.escape(json.dumps(summary,ensure_ascii=False,indent=2))+'</pre><table><tr><th>场景</th><th>检出</th><th>误差px</th><th>耗时ms</th><th>标注图</th></tr>'+body+'</table><h2>负样本</h2><table><tr><th>场景</th><th>判定</th><th>标注图</th></tr>'+neg_body+'</table><img src="overview.jpg">',encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2))
    if not args.record_only:
        assert len(errors)==len(rows),'single-frame misses'
        assert max(errors)<=.7,'static precision >0.7px'
        assert false_positives==0,'negative falsely accepted'
        assert len(motion_errors)==len(movement),'motion misses'
        assert max(motion_errors)<=1.,'motion precision >1px'
        assert summary['motion_tracks']>=110,'excessive full searches during motion'
        assert summary['max_search_masks']<=2 and summary['max_motion_masks']<=4,'search mask bound exceeded'


if __name__=='__main__': main()
