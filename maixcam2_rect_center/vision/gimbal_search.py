"""按钮启动的非阻塞双轴寻靶：相对按钮时姿态，各轴覆盖正负 45°。

这里记录的是已请求的相对位置，不是编码器实测角度。只允许使用实测
脉冲/度或已知减速比；途中识别到黑框、取消或故障时立即停车，不猜测
剩余脉冲，也不自动回零。UART 与本状态机均由控制线程独占。

★ v0.6.3 两处修复（用户实报"按 SEARCH 和没按一样，且没有和陀螺仪结合查找"）：
  ① **未标定脉冲/度时不再一动不动**：旧行为是直接进 `UNCALIBRATED` 并 return，
     一条指令都不发（实机实测 `YAW/PITCH_PULSES_PER_DEG=0`、`GIMBAL_GEAR_RATIO=0`）。
     现按 `SEARCH_PULSES` 的**固定脉冲数**扫描 —— 与 `SELF_TEST_PULSES` 已有的口径一致。
     置 `SEARCH_PULSES=0` 可显式退回旧行为。
  ② **IMU 辅助收段**：给了 `heading` 且其相对朝向可用时，每段按**陀螺仪实测角度**提前
     收段（达到目标角度即停），因此 ±45° 不再依赖脉冲/度标定；未标定/无样本时静默退回
     脉冲口径。段与段之间用**相对脉冲增量**表达，所以某段提前收段也不会把后续端点带偏。
"""
import math

from .stepper_link import estimate_move_seconds


class SearchSweep:
    """每帧/idle 调用 update；构造不发运动，终态后不会再发运动指令。

    调度器启动前应停止其它运动。ACQUIRED/DONE/CANCELLED 的 stopped_at
    是停机帧成功写出的时刻：切回追踪必须等待该时刻之后采集的新画面。
    stop 写失败则为 FAULT，不能将 stop_attempted_at 当成成功停机。
    """

    TERMINAL = ("DONE", "ACQUIRED", "CANCELLED", "UNCALIBRATED", "FAULT")

    def __init__(self, cfg, link, now=0.0, heading=None):
        self.cfg, self.link, self.heading = cfg, link, heading
        self.state, self.reason = "RUNNING", None
        self.index, self.sequence = 0, []
        self.active_axis = None
        self.stopped_at = self.stop_attempted_at = None
        self.position_known = True
        self.commanded_pulses = {"yaw": 0, "pitch": 0}
        self.pulses_per_degree = {}
        self.limit_pulses = {}
        # 每段**相对起始位置**的脉冲增量与目标角度增量（IMU 收段用）。
        self.segment_deltas = []
        self.segment_angles = []
        self.imu_active = False
        self.imu_note = None
        self.calibration_report = {}
        self._measured_slots = set()
        self._reported_calibration = False
        self._seg_angles_at_start = {"yaw": 0.0, "pitch": 0.0}
        self.started_at = self.deadline = self._last_now = float(now)
        self._last_capture = -math.inf
        self._prepare()

    @property
    def done(self):
        return self.state in self.TERMINAL

    @property
    def positions_deg(self):
        """累计请求端点；中途停车后不代表实际姿态（见 position_known）。

        未标定脉冲/度（值为 None）的轴不参与换算 —— 它只有脉冲口径，没有角度口径。
        """
        return {axis: self.commanded_pulses[axis] / ppd
                for axis, ppd in self.pulses_per_degree.items() if ppd}

    def _prepare(self):
        try:
            if not math.isfinite(self.started_at):
                raise ValueError("invalid_timestamp")
            self.rpm = int(getattr(self.cfg, "SEARCH_RPM", 60))
            self.acc = int(getattr(self.cfg, "SEARCH_ACC", 60))
            self.settle_s = float(getattr(self.cfg, "SEARCH_SETTLE_S", 0.10))
            self.timeout_s = float(getattr(self.cfg, "SEARCH_TIMEOUT_S", 60.0))
            self.max_age_s = float(getattr(self.cfg, "RESULT_MAX_AGE_S", 0.20))
            self.per_rev = float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200))
            numbers = (self.settle_s, self.timeout_s, self.max_age_s, self.per_rev)
            if not all(math.isfinite(value) for value in numbers):
                raise ValueError("invalid_config")
            if (not 1 <= self.rpm <= 5000 or not 0 <= self.acc <= 255 or
                    self.settle_s < 0 or self.timeout_s <= 0 or
                    self.max_age_s <= 0 or self.per_rev <= 0):
                raise ValueError("invalid_config")
            ratio = float(getattr(self.cfg, "GIMBAL_GEAR_RATIO", 0.0))
            fallback = int(getattr(self.cfg, "SEARCH_PULSES", 0))
            for axis in ("yaw", "pitch"):
                angle = float(getattr(self.cfg, "SEARCH_" + axis.upper() + "_DEG", 45.0))
                if not math.isfinite(angle) or not 0 < angle <= 45.0:
                    raise ValueError("search_angle_out_of_range")
                ppd = float(getattr(self.cfg, axis.upper() + "_PULSES_PER_DEG", 0.0))
                if ppd <= 0 and math.isfinite(ratio) and ratio > 0:
                    ppd = self.per_rev * ratio / 360.0
                if not math.isfinite(ppd) or ppd <= 0:
                    # ★ 未标定脉冲/度：按固定脉冲数扫描（不再是"一动不动"）。
                    if fallback <= 0:
                        self.state, self.reason = "UNCALIBRATED", "pulse_per_degree_not_calibrated"
                        return
                    limit = fallback
                    known_ppd = None
                else:
                    # 端点先量化、再作差；不会因逐段取整把累计端点推到 ±45° 之外。
                    limit = int(math.floor(angle * ppd))
                    known_ppd = ppd
                if limit <= 0 or 2 * limit > 0xFFFFFFFF:
                    raise ValueError("invalid_pulse_count")
                self.pulses_per_degree[axis] = known_ppd
                self.limit_pulses[axis] = limit
                # 序列写成**相对增量**：+limit → −2·limit → +limit（等价于端点 +limit/−limit/0），
                # 这样某段被 IMU 提前收段时，后续段仍以"相对当前姿态"下发，不会累积错位。
                for delta, target_angle in ((limit, angle), (-2 * limit, -angle), (limit, 0.0)):
                    self.sequence.append((axis, delta))
                    self.segment_deltas.append((axis, delta))
                    self.segment_angles.append((axis, target_angle))
            self._activate_imu()
            self.timeout_at = self.started_at + self.timeout_s
        except (TypeError, ValueError, OverflowError) as exc:
            self.state, self.reason = "FAULT", str(exc) or "invalid_config"

    def _activate_imu(self):
        """尝试用 IMU 相对朝向做角度收段；任何不可用都静默退回脉冲口径。"""
        self.imu_active = False
        self.imu_note = "imu_off"
        if self.heading is None or not bool(getattr(self.cfg, "SEARCH_USE_IMU", True)):
            return
        try:
            if not self.heading.lock_reference(self.started_at):
                self.imu_note = "imu_reference_unavailable"
                return
        except Exception as exc:                                  # 传感器/线程异常都不致命
            self.imu_note = "imu_error:%s" % exc
            return
        self.imu_active = True
        self.imu_note = "imu_angle_terminated"

    def _measure_segment(self, now):
        """段结束时记录「指令脉冲 vs IMU 实测转角」，一次 SEARCH 即可反推脉冲/度。

        价值：现场不必专门跑开环标定 —— 点一次 SEARCH，日志里的
        `≈X 脉冲/度` 就是可以直接填回 `YAW/PITCH_PULSES_PER_DEG` 的标定值。
        只在 IMU 可用时统计；采样不可用则跳过（该项缺失不影响搜索本身）。
        """
        slot = self.index - 1
        if not (self.imu_active and 0 <= slot < len(self.segment_deltas)):
            return
        if slot in self._measured_slots:
            return                                                   # 同段只统计一次
        self._measured_slots.add(slot)
        axis, delta = self.segment_deltas[slot]
        angles = self._imu_angles(now)
        if angles is None:
            return
        moved = angles[axis] - self._seg_angles_at_start[axis]
        record = self.calibration_report.setdefault(axis, dict(pulses=0, deg=0.0))
        record["pulses"] += abs(delta)
        record["deg"] += moved

    def calibration_summary(self):
        """把累计的「脉冲↔实测角度」换算成脉冲/度；无有效样本返回 None。"""
        out = {}
        for axis, record in self.calibration_report.items():
            if abs(record["deg"]) < 1.0 or record["pulses"] <= 0:
                continue
            out[axis] = abs(record["pulses"] / record["deg"])       # 脉冲/度
        return out or None

    def _imu_angles(self, now):
        """当前相对扫描起点的 yaw/pitch 角度；不可用返回 None。"""
        try:
            state = self.heading.sample(now)
        except Exception:
            return None
        if not state or not state.get("valid"):
            return None
        try:
            return {"yaw": float(state["yaw_deg"]), "pitch": float(state["pitch_deg"])}
        except (KeyError, TypeError, ValueError, OverflowError):
            return None

    def _mark_segment_start(self, now):
        angles = self._imu_angles(now) if self.imu_active else None
        self._seg_angles_at_start = angles or {"yaw": 0.0, "pitch": 0.0}

    def _imu_angle_reached(self, now):
        """本段是否已转到目标角度（IMU 实测），用于提前收段。"""
        slot = self.index - 1
        if not 0 <= slot < len(self.segment_angles):
            return False
        axis, target_angle = self.segment_angles[slot]
        if abs(target_angle) <= 0:
            return False                                          # 回零段不靠角度收，走脉冲时长
        margin = float(getattr(self.cfg, "SEARCH_IMU_MARGIN_DEG", 3.0))
        angles = self._imu_angles(now)
        if angles is None:
            return False
        moved = angles[axis] - self._seg_angles_at_start[axis]
        # 目标角度的符号 = 期望转向；只看同向推进量，避免反向抖动误判到点。
        return moved * target_angle > 0 and abs(moved) >= abs(target_angle) - margin

    def _target_is_fresh(self, result, now):
        if not result or not result.get("valid") or result.get("held") or result.get("predicted"):
            return False
        try:
            captured = float(result["captured_at"])
            coordinates = (float(result["u"]), float(result["v"]))
        except (KeyError, TypeError, ValueError, OverflowError):
            return False
        if (not all(math.isfinite(value) for value in (captured,) + coordinates) or
                captured < self.started_at or captured <= self._last_capture or
                not 0 <= now - captured <= self.max_age_s):
            return False
        self._last_capture = captured
        return True

    def _finish(self, state, reason, now):
        self._measure_segment(now)
        summary = self.calibration_summary()
        if summary and not self._reported_calibration:
            self._reported_calibration = True
            print("[ui] SEARCH 实测脉冲/度：%s（可直接填回 config 的 %s_PULSES_PER_DEG）"
                  % ("、".join("%s≈%.2f" % (axis.upper(), value)
                              for axis, value in sorted(summary.items())),
                     "/".join(axis.upper() for axis in sorted(summary))))
        moving = self.active_axis is not None and now < self.deadline
        self.position_known = self.position_known and not moving
        self.stop_attempted_at = now
        try:
            accepted = self.link.stop(now=now)
        except Exception:
            accepted = False
        self.active_axis = None
        if accepted is False:
            self.state, self.reason = "FAULT", "stop_failed:%s" % reason
            self.position_known = False
        else:
            self.state, self.reason, self.stopped_at = state, reason, now
        return self.state

    def cancel(self, now):
        """按钮再次点击取消；禁止按已请求端点倒算回起点。"""
        if self.done:
            return self.state
        try:
            now = float(now)
        except (TypeError, ValueError, OverflowError):
            now = self._last_now
        if not math.isfinite(now):
            now = self._last_now
        now = max(now, self._last_now)
        return self._finish("CANCELLED", "cancelled", now)

    def fail(self, reason, now):
        """控制器发现链路故障时调用；失败后的 update 不会再下发运动。"""
        if self.done:
            return self.state
        try:
            now = float(now)
        except (TypeError, ValueError, OverflowError):
            now = self._last_now
        if not math.isfinite(now):
            now = self._last_now
        now = max(now, self._last_now)
        return self._finish("FAULT", str(reason), now)

    def update(self, now, result=None):
        """最多发一段位置指令；视觉识别在运动未完成时也能立即停车。"""
        if self.done:
            return self.state
        try:
            now = float(now)
        except (TypeError, ValueError, OverflowError):
            return self.fail("invalid_timestamp", self._last_now)
        if not math.isfinite(now) or now < self._last_now:
            return self.fail("invalid_timestamp", self._last_now)
        self._last_now = now
        fault = getattr(self.link, "fault_reason", None)
        if fault:
            return self.fail("link:%s" % fault, now)
        if self._target_is_fresh(result, now):
            return self._finish("ACQUIRED", "black_frame_found", now)
        if now >= self.timeout_at:
            return self._finish("DONE", "timeout", now)
        if now < self.deadline:
            if not (self.imu_active and self._imu_angle_reached(now)):
                return self.state
            # ★ IMU 实测角度已到目标 → 立刻收段：比"估算脉冲时长"更准，且**不依赖脉冲/度标定**。
            if self.link.stop(axis=self.active_axis, now=now) is False:
                self.position_known = False
                return self.fail("stop_failed:imu_angle", now)
            self.deadline = now + self.settle_s
            self.active_axis = None
            return self.state
        self.active_axis = None
        if self.index >= len(self.sequence):
            return self._finish("DONE", "sweep_complete", now)
        axis, delta = self.sequence[self.index]
        pulses = abs(delta)
        positive_ccw = bool(getattr(self.cfg, axis.upper() + "_POSITIVE_CCW", True))
        ccw = positive_ccw if delta > 0 else not positive_ccw
        try:
            duration = estimate_move_seconds(pulses, self.rpm, self.acc, self.per_rev)
            accepted = self.link.move(axis, ccw, pulses, self.rpm, self.acc, now=now)
        except Exception:
            accepted = False
        if accepted is False:
            self.position_known = False
            return self.fail("uart_write_failed:%s" % axis, now)
        self.commanded_pulses[axis] += delta
        self.active_axis = axis
        self.deadline = now + duration + self.settle_s
        self._measure_segment(now)                  # 先结算刚结束的那一段（IMU 实测）
        self._mark_segment_start(now)               # IMU 角度基线（本段起点）
        self.index += 1
        return self.state
