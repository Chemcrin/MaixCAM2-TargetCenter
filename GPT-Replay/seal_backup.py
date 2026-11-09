"""验证并封存备份。首次直接运行；之后用 --check 核验，拒绝覆盖现有封存。"""
import argparse
import ast
import hashlib
import json
import os
import subprocess
import sys
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent
APP = ROOT / 'maixcam2_rect_center'
ARCHIVE = ROOT / 'GPT-Replay_2026-10-06-laser-spot-v1.zip'
MANIFEST = ROOT / 'SHA256SUMS.txt'
SIDECAR = ARCHIVE.with_suffix('.zip.sha256')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def payload():
    skip = {MANIFEST.name, ARCHIVE.name, SIDECAR.name}
    return sorted(p for p in ROOT.rglob('*') if p.is_file()
                  and '__pycache__' not in p.parts and p.suffix != '.pyc'
                  and p.name not in skip and p.suffix != '.zip')


def verify():
    entries = {}
    for line in MANIFEST.read_text(encoding='utf-8').splitlines():
        checksum, name = line.split('  ', 1)
        entries[name] = checksum
    actual = {p.relative_to(ROOT).as_posix(): digest(p.read_bytes()) for p in payload()}
    if actual != entries:
        raise RuntimeError('文件被增删或校验值变化：' + str(sorted(
            name for name in actual.keys() | entries.keys() if actual.get(name) != entries.get(name))))
    checksum, filename = SIDECAR.read_text(encoding='utf-8').strip().split('  ', 1)
    if filename != ARCHIVE.name or digest(ARCHIVE.read_bytes()) != checksum:
        raise RuntimeError('ZIP 校验失败')
    with zipfile.ZipFile(ARCHIVE) as archive:
        expected = {'GPT-Replay/' + name: value for name, value in entries.items()}
        expected['GPT-Replay/' + MANIFEST.name] = digest(MANIFEST.read_bytes())
        actual_zip = {name: digest(archive.read(name)) for name in archive.namelist()}
        if expected != actual_zip or archive.testzip() is not None:
            raise RuntimeError('ZIP 内容与文件清单不一致')
    print('SHA256/ZIP verified: %d payload files; %d archive entries' % (len(entries), len(entries) + 1))
    print('Archive SHA256:', checksum)


def create():
    outputs = (ROOT / 'validation.txt', ROOT / 'provenance.json', MANIFEST, ARCHIVE, SIDECAR)
    if any(p.exists() for p in outputs):
        raise RuntimeError('已有封存文件，拒绝覆盖；请使用 --check')
    for p in ROOT.rglob('*.py'):
        ast.parse(p.read_text(encoding='utf-8'), filename=str(p))
    env = dict(os.environ, PYTHONIOENCODING='utf-8', PYTHONDONTWRITEBYTECODE='1')
    report = ['GPT-Replay 历史恢复版验证，2026-11-09',
              'AST: all Python sources parse successfully',
              'Runtime: ' + sys.version, 'Executable: ' + sys.executable]
    commands = [('tools/check_app.py', '--manifest', '--strict'),
                ('tools/test_aiming.py',), ('tools/test_replay.py',),
                ('tools/test_performance.py',)]
    for command in commands:
        result = subprocess.run([sys.executable, '-B', *command], cwd=APP, env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                encoding='utf-8', timeout=120)
        report.extend(['', '$ python -B ' + ' '.join(command), result.stdout,
                       'exit_code=%d' % result.returncode])
        if result.returncode:
            print(result.stdout)
            raise RuntimeError('验证失败，未封存：' + command[0])
    (ROOT / 'validation.txt').write_text('\n'.join(report) + '\n', encoding='utf-8', newline='\n')
    commits = {}
    for short in ('fe530c8', 'd2f36e7', 'bfff85a'):
        commits[short] = subprocess.check_output(['git', 'rev-parse', short], cwd=ROOT.parent).decode().strip()
    provenance = dict(restored_on='2026-11-09', build_id='2026-10-06-laser-spot-v1',
                      kind='historical_git_plus_conversation_reconstruction',
                      byte_identical_original_snapshot=False, current_application_modified=False,
                      source_commits=commits,
                      reconstructed_from_conversation=['main.py', 'vision/aiming.py',
                                                       'tools/test_aiming.py', 'laser configuration'],
                      historical_baseline={'positive_cases': 30, 'detected': 29,
                                           'known_miss_index': 2, 'max_error_px': 0.803})
    (ROOT / 'provenance.json').write_text(json.dumps(provenance, ensure_ascii=False, indent=2) + '\n',
                                        encoding='utf-8', newline='\n')
    files = payload()
    MANIFEST.write_text(''.join(digest(p.read_bytes()) + '  ' + p.relative_to(ROOT).as_posix() + '\n'
                               for p in files), encoding='utf-8', newline='\n')
    with zipfile.ZipFile(ARCHIVE, 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in files + [MANIFEST]:
            archive.write(p, 'GPT-Replay/' + p.relative_to(ROOT).as_posix())
    SIDECAR.write_text(digest(ARCHIVE.read_bytes()) + '  ' + ARCHIVE.name + '\n',
                       encoding='utf-8', newline='\n')
    verify()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    verify() if args.check else create()
