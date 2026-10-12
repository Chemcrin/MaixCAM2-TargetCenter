"""PC（Windows）一条命令跑完本目录**所有可离线运行**的检查，并明确标出哪些只能在板子上跑。

用途：把"这个目录在本机能不能用"变成一个确定性的答案，而不是逐个脚本试错。
板端专属的东西（MaixPy SDK、串口、图传、显示）在 PC 上永远跑不了 —— 本工具把它们
标成 SKIP 并写明原因，**不让它们伪装成失败**，也**不让真正的失败被跳过掩盖**。

用法：
    "D:/Python311/python.exe" -B tools/check_pc.py            # 全部
    "D:/Python311/python.exe" -B tools/check_pc.py --quick    # 跳过较慢的视觉回归
    "D:/Python311/python.exe" -B tools/check_pc.py -v         # 打印每个脚本的输出

退出码：0 = 没有失败；1 = 有脚本失败（可接 CI / 提交前钩子）。
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"

# 慢用例：跑起来要几秒到十几秒，--quick 时跳过。
SLOW = {"test_performance.py", "test_images_e2e.py", "test_imu_turn_tracking.py",
        "test_gimbal_tracking.py", "test_aimloop_tracking.py", "test_runtime_performance.py"}

# 需要命令行参数、不能裸跑的脚本（由别的用例/人工调用）。
NEEDS_ARGS = {"diagnose_detect.py", "run_pc.py", "check_pc.py", "example_api_usage.py",
              "uart_probe.py", "git_round.py", "clean_artifacts.py",
              "apply_gimbal_track_patch.py", "compare_guoyi_scheme.py"}

SKIP_MARKERS = ("[skip]", "SKIP")

# 已知的非"PC 运行"问题：打包身份/产物一致性等，需人工决定，不计入 PC 可用性失败，
# 但仍逐条列出来（不静默、不掩盖）。
KNOWN_ISSUES = {
    "check_app.py": "C1：app.yaml 的 id 与目录名不一致（打包身份问题，需人工决定是否改名，"
                    "与 PC 能否运行无关）",
}


def scripts(quick):
    out = []
    for path in sorted(TOOLS.glob("*.py")):
        name = path.name
        if name in NEEDS_ARGS or not (name.startswith("test_") or name in
                                      ("build_single.py", "check_app.py")):
            continue
        if quick and name in SLOW:
            continue
        out.append(path)
    return out


def run_one(path, verbose):
    cmd = [sys.executable, "-B", str(path)]
    if path.name == "build_single.py":
        cmd.append("--check")
    elif path.name == "check_app.py":
        cmd.append("--manifest")
    t0 = time.perf_counter()
    proc = subprocess.run(cmd, cwd=str(ROOT), stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True,
                          encoding="utf-8", errors="replace")
    elapsed = time.perf_counter() - t0
    out = proc.stdout or ""
    if verbose:
        print("-" * 78)
        print("$ %s" % " ".join(cmd))
        print(out.rstrip())
    if proc.returncode != 0:
        status = "FAIL"
    elif any(mark in out for mark in SKIP_MARKERS):
        status = "SKIP"
    else:
        status = "PASS"
    return status, elapsed, out


def summarize(out, status):
    """从脚本输出里挑一行最有信息量的结论行。"""
    lines = [ln.strip() for ln in out.splitlines() if ln.strip()]
    for ln in reversed(lines):
        if any(k in ln for k in ("PASS", "FAIL", "通过", "失败", "skip", "OK=", "汇总")):
            return ln[:96]
    return lines[-1][:96] if lines else ""


def main():
    ap = argparse.ArgumentParser(description="PC 端可运行性总检")
    ap.add_argument("--quick", action="store_true", help="跳过较慢的视觉/跟踪回归")
    ap.add_argument("-v", "--verbose", action="store_true", help="打印每个脚本的完整输出")
    args = ap.parse_args()

    print("PC 环境：Python %s (%s)" % (sys.version.split()[0], sys.executable))
    try:
        import cv2
        import numpy as np
        print("依赖：numpy %s / OpenCV %s" % (np.__version__, cv2.__version__))
    except ImportError as exc:
        raise SystemExit("缺少依赖 %s；本机请用 D:/Python311/python.exe 运行。" % exc)

    todo = scripts(args.quick)
    results = []
    for path in todo:
        status, elapsed, out = run_one(path, args.verbose)
        results.append((path.name, status, elapsed, out))
        print("%-4s %-30s %6.1fs  %s" % (status, path.name, elapsed,
                                         summarize(out, status) if status != "PASS"
                                         else ""))

    fails = [r for r in results if r[1] == "FAIL"]
    skips = [r for r in results if r[1] == "SKIP"]
    known = [r for r in fails if r[0] in KNOWN_ISSUES]
    real_fails = [r for r in fails if r[0] not in KNOWN_ISSUES]
    print("\n汇总：PASS=%d  SKIP=%d  KNOWN=%d  FAIL=%d（共 %d 个脚本）"
          % (len(results) - len(fails) - len(skips), len(skips), len(known),
             len(real_fails), len(results)))
    if skips:
        print("SKIP（只能在 MaixCAM2 板子上跑，PC 不可用属预期）：%s"
              % ", ".join(r[0] for r in skips))
    for name, _, _, _ in known:
        print("KNOWN（不影响 PC 可用性，但需人工处理）：%s -> %s"
              % (name, KNOWN_ISSUES[name]))
    if real_fails:
        print("\n失败脚本（完整输出用 -v 重跑）：%s" % ", ".join(r[0] for r in real_fails))
        for name, _, _, out in real_fails:
            print("\n== %s ==\n%s" % (name, out.rstrip()[-1200:]))
        return 1
    print("PC 侧全部通过。板端专属链路（显示/串口/图传/云台）仍需上板验证。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
