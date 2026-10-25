"""Print proposals and the accepted candidate's per-side tape evidence."""
import argparse
import json
from run_pc import imread_unicode, load_config
from vision.rectangle_center import RectangleCenterDetector


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image')
    args = parser.parse_args()
    gray = imread_unicode(args.image)
    if gray is None:
        raise SystemExit('Cannot read image: ' + args.image)
    detector = RectangleCenterDetector(load_config())
    for level in (0, 1):
        proposals = detector.locator.locate(gray, level=level)
        print('level=%d candidates=%d stats=%s' % (level, len(proposals), detector.locator.last_stats))
    result = detector.process(gray)
    print('result valid=%s stage=%s masks=%d proposals=%d ms=%.2f' % (
        result['valid'], result['stage'], result['masks'], result['proposals'], result['detector_ms']))
    if result['valid']:
        print(json.dumps(dict(center=[result['u'], result['v']],
                              raw=[result['raw_u'], result['raw_v']],
                              kind=result['kind'], aspect=detector._aspect(result['quad']),
                              evidence=[{k: round(v, 2) for k, v in row.items()}
                                        for row in result['edge_profile']]), ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
