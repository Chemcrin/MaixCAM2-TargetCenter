"""One-command PC checks for the v1.3.0 four-layer build.

Covers packaging (check_app) and the single-file build (build_single), plus the
synthetic-image smoke regression. Board-only behaviour (camera, display, UART)
cannot run here; missing hardware stays explicit, never a fake pass.

    python -B tools/check_pc.py            # full gate
    python -B tools/check_pc.py --quick    # same set (kept for muscle memory)
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHECKS = (
    ['tools/check_app.py', '--manifest'],
    ['tools/build_single.py', '--check'],
    ['tools/build_single.py', '--smoke'],
    ['tools/test_smoke.py'],
)


def main():
    # Windows pipes default to GBK; child checks print Chinese and Unicode maths.
    os.environ['PYTHONIOENCODING'] = 'utf-8'
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--quick', action='store_true',
                        help='Accepted for compatibility; this gate has no long suite to skip')
    parser.parse_args()
    failed = []
    for command in CHECKS:
        print('\nCHECK ' + ' '.join(command), flush=True)
        try:
            result = subprocess.run([sys.executable, '-B', *command], cwd=ROOT, timeout=180)
        except subprocess.TimeoutExpired:
            failed.append(command)
            print('FAIL timeout', flush=True)
            continue
        if result.returncode:
            failed.append(command)
    print('\nPC checks: %d passed / %d failed' % (len(CHECKS) - len(failed), len(failed)))
    for command in failed:
        print('FAIL ' + ' '.join(command))
    return bool(failed)


if __name__ == '__main__':
    sys.exit(main())
