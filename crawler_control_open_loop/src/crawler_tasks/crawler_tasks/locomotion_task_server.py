#!/usr/bin/env python3
import time
import math
import traceback
from typing import Dict, Any, List, Optional, Set, Iterable, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import UInt8MultiArray
from crawler_msgs.action import Locomotion
from crawler_msgs.msg import JointSetpoint


def to_idx(name: str) -> Optional[int]:
    if name and name[0].upper() == 'J':
        try:
            v = int(name[1:]) - 1
            return v if 0 <= v < 7 else None
        except Exception:
            return None
    return None


def phases_as_list(phases_field: Any) -> List[Dict[str, Any]]:
    if isinstance(phases_field, list):
        return [p for p in phases_field if isinstance(p, dict)]
    if isinstance(phases_field, dict):
        # preserve p1,p2,... order
        keys = sorted(phases_field.keys(), key=lambda k: (len(k), k))
        return [phases_field[k] for k in keys if isinstance(phases_field[k], dict)]
    return []


class LocomotionTaskServer(Node):
    def __init__(self):
        super().__init__(
            'locomotion_task_server',
            automatically_declare_parameters_from_overrides=True
        )

        # ---------- param helpers (Jazzy-safe; never re-declare if YAML provided) ----------
        def p_str(name: str, default: str) -> str:
            if self.has_parameter(name):
                return str(self.get_parameter(name).value)
            return str(self.declare_parameter(name, default).value)

        def p_int(name: str, default: int) -> int:
            if self.has_parameter(name):
                return int(self.get_parameter(name).value)
            return int(self.declare_parameter(name, default).value)

        def p_float(name: str, default: float) -> float:
            if self.has_parameter(name):
                return float(self.get_parameter(name).value)
            return float(self.declare_parameter(name, default).value)

        def p_bool(name: str, default: bool) -> bool:
            if self.has_parameter(name):
                return bool(self.get_parameter(name).value)
            return bool(self.declare_parameter(name, default).value)

        def p_float2(name: str, default: Tuple[float, float]) -> Tuple[float, float]:
            if self.has_parameter(name):
                v = self.get_parameter(name).value
            else:
                v = self.declare_parameter(name, list(default)).value
            try:
                return (float(v[0]), float(v[1]))
            except Exception:
                return default

        # ---------- node-level params ----------
        self.out_topic             = p_str('out_topic', '/joint_goal')
        self.rate_hz               = p_float('rate_hz', 100.0)
        self.sleep_step_s          = p_float('sleep_step_s', 0.005)

        self.settle_consecutive    = p_int('settle_consecutive', 2)
        self.min_dwell_s           = p_float('min_dwell_s', 0.05)

        self.global_timeout_s      = p_float('global_timeout_s', 20.0)
        self.phase_timeout_floor_s = p_float('phase_timeout_s', 6.0)
        self.timeout_safety        = p_float('timeout_safety_factor', 1.5)

        # dwell controls
        self.inter_cycle_wait_s    = p_float('inter_cycle_wait_s', 0.20)  # between cycles
        self.hold_on_wait          = p_bool('hold_on_wait', True)          # publish "hold" before waiting

        # J1/J2 scale (turns compensation)
        self.j12_scale             = p_float2('j12_pos_cmd_scale', (10.0, 10.0))

        # tolerances (kept for compatibility)
        tol_param = self.get_parameters_by_prefix('tol')
        self.tol: Dict[str, float] = {k: float(p.value) for k, p in tol_param.items()}

        # default require list
        req_param = self.get_parameters_by_prefix('default_require')
        if req_param:
            root = list(req_param.values())[0].value
            if isinstance(root, Iterable) and not isinstance(root, (str, bytes)):
                self.default_require: Set[str] = set(str(x) for x in root)
            else:
                self.default_require = set()
        else:
            self.default_require = set([f'J{i+1}' for i in range(7)])

        # gaits tree (phases)
        self.gaits = self._nest_from_prefix_map(self.get_parameters_by_prefix('gaits'))
        if not self.gaits:
            self.get_logger().fatal('No gaits.* phases loaded. Pass gaits_phased.yaml via --params-file.')
            raise SystemExit(1)

        # pub/sub
        self.pub = self.create_publisher(JointSetpoint, self.out_topic, 10)
        self.reached_sub = self.create_subscription(UInt8MultiArray, '/maestro/reached', self.on_reached, 10)
        self.reached_latest: List[int] = [0] * 7

        # action server
        self.server = ActionServer(
            self, Locomotion, '/locomotion/move',
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            execute_callback=self.execute_cb
        )

        self.get_logger().info(f'phased server ready; gaits: {sorted(self.gaits.keys())}')

    # ---------- utils ----------
    def _nest_from_prefix_map(self, prefix_map: Dict[str, Any]) -> Dict[str, Any]:
        root: Dict[str, Any] = {}
        for dotted, param in prefix_map.items():
            val = param.value if hasattr(param, 'value') else param
            keys = dotted.split('.') if dotted else []
            node = root
            for k in keys[:-1]:
                node = node.setdefault(k, {})
            if keys:
                node[keys[-1]] = val
        return root.get('gaits', root)

    def _uniform_wait(self, seconds: float, why: str, last_cmd: Optional[JointSetpoint] = None):
        if seconds <= 0.0:
            return
        # publish a hold if requested (speeds → 0; positions unchanged)
        if self.hold_on_wait and last_cmd is not None:
            hold = JointSetpoint()
            hold.header.stamp = self.get_clock().now().to_msg()
            hold.position = list(last_cmd.position)
            hold.speed = [0.0] * 7
            hold.pumps_front_on = last_cmd.pumps_front_on
            hold.solenoids_front_on = last_cmd.solenoids_front_on
            hold.pumps_back_on = last_cmd.pumps_back_on
            hold.solenoids_back_on = last_cmd.solenoids_back_on
            self.pub.publish(hold)

        t0 = time.monotonic()
        step = max(0.001, float(self.sleep_step_s))
        while (time.monotonic() - t0) < seconds and rclpy.ok():
            time.sleep(step)
        dt = time.monotonic() - t0
        self.get_logger().info(f'{why}: waited {dt:.3f}s (target {seconds:.2f}s)')

    def _per_joint_trap_time(self, dist: float, vmax: float, amax: Optional[float]) -> float:
        dist = abs(dist)
        if vmax <= 0.0:
            return 0.0
        if amax is None or amax <= 0.0:
            return dist / vmax
        t_acc = vmax / amax
        d_acc = 0.5 * amax * t_acc * t_acc
        if dist >= 2.0 * d_acc:    # trapezoid
            d_cruise = dist - 2.0 * d_acc
            return 2.0 * t_acc + d_cruise / vmax
        else:                      # triangle
            return math.sqrt(2.0 * dist / amax)

    # ---------- callbacks ----------
    def on_reached(self, msg: UInt8MultiArray):
        arr = list(msg.data)
        if len(arr) >= 7:
            self.reached_latest = arr[:7]

    # ---------- action plumbing ----------
    def goal_cb(self, goal: Locomotion.Goal):
        if goal.gait not in self.gaits:
            self.get_logger().warn(f'Unknown gait "{goal.gait}"')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, _):
        return CancelResponse.ACCEPT

    # ---------- helpers ----------
    def _phase_require(self, ph: Dict[str, Any]) -> Set[str]:
        require_raw = ph.get('require', list(self.default_require))
        if isinstance(require_raw, (list, set, tuple)):
            return set(str(x) for x in require_raw)
        return {str(require_raw)} if require_raw else set()

    def _need_idxs(self, require: Set[str]) -> List[int]:
        idxs = [to_idx(n) for n in require if isinstance(n, str) and n.upper().startswith('J')]
        return [i for i in idxs if i is not None]

    def _compute_dynamic_timeout(self, ph: Dict[str, Any], action_speed_default: Optional[float]) -> float:
        # optional per-phase override
        ph_override = 0.0
        try:
            if 'timeout_s' in ph:
                ph_override = float(ph.get('timeout_s', 0.0))
        except Exception:
            ph_override = 0.0

        base = float(self.phase_timeout_floor_s)
        joints = ph.get('joints', {}) or {}
        longest = 0.0

        for jname, spec in joints.items():
            idx = to_idx(jname)
            if idx is None:
                continue

            mode = str(spec.get('mode', 'hold')).lower().strip()
            # resolve vmax: explicit → action default → skip
            vmax = None
            if 'vmax' in spec:
                try:
                    vmax = float(spec['vmax'])
                except Exception:
                    vmax = None
            if vmax is None and action_speed_default is not None:
                vmax = float(action_speed_default)
            if vmax is None or vmax <= 0.0:
                continue

            # amax (optional)
            amax = None
            if 'amax' in spec:
                try:
                    amax = float(spec['amax'])
                except Exception:
                    amax = None

            # distance in command units (apply J1/J2 scaling)
            d = 0.0
            if mode in ('move_rel', 'relative', 'rel'):
                try:
                    d = float(spec.get('dpos', 0.0))
                except Exception:
                    d = 0.0
            elif mode in ('move_abs', 'absolute', 'abs', 'move'):
                # cannot know dist for abs w/out current pos; skip
                continue
            else:
                continue

            if idx == 0:
                d *= float(self.j12_scale[0])
            elif idx == 1:
                d *= float(self.j12_scale[1])

            t_joint = self._per_joint_trap_time(d, vmax, amax)
            if t_joint > longest:
                longest = t_joint

        cushion = 0.25
        dyn = max(base, longest * self.timeout_safety + cushion)
        if ph_override > 0.0:
            dyn = max(dyn, ph_override)

        self.get_logger().info(
            f'Dynamic timeout for phase: longest={longest:.2f}s, safety={self.timeout_safety:.2f} → dyn={dyn:.2f}s '
            f'(floor {base:.2f}s, override {ph_override:.2f}s)'
        )
        return dyn

    # ---------- core execute ----------
    def execute_cb(self, gh):
        try:
            goal: Locomotion.Goal = gh.request
            gait_cfg = self.gaits[goal.gait]
            phases_list = phases_as_list(gait_cfg.get('phases', []))
            if not phases_list:
                gh.abort()
                return Locomotion.Result(success=False, message='No phases in gait')

            mps = float(gait_cfg.get('meters_per_cycle', 0.0) or 0.0)
            distance = float(getattr(goal, 'distance', 0.0) or 0.0)
            cycles = 1 if mps <= 0.0 else int(math.ceil(max(0.0, distance) / mps))
            cycles = max(1, cycles)

            # per-gait default between-phase wait (optional)
            gait_between_wait = 0.0
            try:
                if 'between_phase_wait_s' in gait_cfg:
                    gait_between_wait = float(gait_cfg.get('between_phase_wait_s', 0.0))
            except Exception:
                gait_between_wait = 0.0

            action_speed_default: Optional[float] = None
            try:
                if float(getattr(goal, 'speed', 0.0)) > 0.0:
                    action_speed_default = float(goal.speed)
            except Exception:
                action_speed_default = None

            self.get_logger().info(f'Action params → distance={distance:.4f} m, '
                                   f'meters_per_cycle={mps if mps>0 else float("nan")}, cycles={cycles}')

            last_cmd = JointSetpoint()
            last_cmd.position = [0.0] * 7
            last_cmd.speed = [0.0] * 7
            last_cmd.pumps_front_on = False
            last_cmd.solenoids_front_on = False
            last_cmd.pumps_back_on = False
            last_cmd.solenoids_back_on = False

            t_start = time.monotonic()
            global_deadline = t_start + float(self.global_timeout_s)

            for ci in range(cycles):
                if time.monotonic() > global_deadline:
                    gh.abort()
                    return Locomotion.Result(success=False, message='Global timeout')

                # Between-cycles dwell (except before first cycle)
                if ci > 0:
                    self._uniform_wait(self.inter_cycle_wait_s,
                                       f'Between-cycles dwell before cycle {ci+1}',
                                       last_cmd)

                self.get_logger().info(f'Cycle {ci+1}/{cycles} start')

                for pi, ph in enumerate(phases_list, start=1):
                    # --- build and publish the command for this phase ---
                    self.get_logger().info(f'Phase {pi}/{len(phases_list)} start')

                    cmd = JointSetpoint()
                    cmd.header.stamp = self.get_clock().now().to_msg()
                    cmd.position = list(last_cmd.position)
                    cmd.speed = list(last_cmd.speed)

                    joints: Dict[str, Any] = ph.get('joints', {}) or {}
                    for jname, spec in joints.items():
                        idx = to_idx(jname)
                        if idx is None:
                            continue
                        mode = str(spec.get('mode', 'hold')).lower().strip()

                        # resolve vmax: explicit → action default → current
                        vmax = None
                        if 'vmax' in spec:
                            try:
                                vmax = float(spec['vmax'])
                            except Exception:
                                vmax = None
                        if vmax is None and action_speed_default is not None:
                            vmax = float(action_speed_default)
                        if vmax is None:
                            vmax = cmd.speed[idx] or 0.0

                        if mode == 'hold':
                            cmd.speed[idx] = vmax
                        elif mode in ('move_rel', 'relative', 'rel'):
                            dpos = float(spec.get('dpos', 0.0))
                            if idx == 0:
                                dpos *= float(self.j12_scale[0])
                            elif idx == 1:
                                dpos *= float(self.j12_scale[1])
                            cmd.position[idx] = last_cmd.position[idx] + dpos
                            cmd.speed[idx] = vmax
                        elif mode in ('move_abs', 'absolute', 'abs', 'move'):
                            pos = float(spec.get('pos', last_cmd.position[idx]))
                            if idx == 0:
                                pos *= float(self.j12_scale[0])
                            elif idx == 1:
                                pos *= float(self.j12_scale[1])
                            cmd.position[idx] = pos
                            cmd.speed[idx] = vmax
                        else:
                            self.get_logger().warn(f'Unknown mode "{mode}" for {jname}; treating as hold')
                            cmd.speed[idx] = vmax

                    # Actuators
                    acts = ph.get('actuators', {}) or {}
                    cmd.solenoids_front_on = bool(acts.get('solenoids_front', last_cmd.solenoids_front_on))
                    cmd.pumps_front_on     = bool(acts.get('pumps_front',     last_cmd.pumps_front_on))
                    cmd.solenoids_back_on  = bool(acts.get('solenoids_back',  last_cmd.solenoids_back_on))
                    cmd.pumps_back_on      = bool(acts.get('pumps_back',      last_cmd.pumps_back_on))

                    # Publish the phase command
                    self.pub.publish(cmd)
                    last_cmd = cmd

                    # --- settle/wait for requirements ---
                    require_set = self._phase_require(ph)
                    need_idxs = self._need_idxs(require_set)
                    dyn_timeout = self._compute_dynamic_timeout(ph, action_speed_default)

                    t_phase0 = time.monotonic()
                    consec = 0

                    while rclpy.ok():
                        if gh.is_cancel_requested:
                            gh.canceled()
                            return Locomotion.Result(success=False, message='Canceled')

                        if (time.monotonic() - t_phase0) < self.min_dwell_s:
                            time.sleep(self.sleep_step_s)
                            continue

                        ok = True
                        for idx in need_idxs:
                            if idx < 0 or idx >= len(self.reached_latest) or self.reached_latest[idx] != 1:
                                ok = False
                                break

                        consec = consec + 1 if ok else 0
                        if consec >= self.settle_consecutive:
                            break

                        # timeouts
                        if (time.monotonic() - t_phase0) > dyn_timeout:
                            self.get_logger().warn(f'Phase {pi} timeout after {dyn_timeout:.2f}s; advancing')
                            break
                        if time.monotonic() > global_deadline:
                            gh.abort()
                            return Locomotion.Result(success=False, message='Global timeout')

                        # feedback
                        fb = Locomotion.Feedback()
                        fb.elapsed = float(time.monotonic() - t_start)
                        fb.cycles_done = ci * len(phases_list) + (pi - 1)
                        fb.progress = float(ci + (pi - 1) / max(1.0, float(len(phases_list)))) / float(cycles)
                        gh.publish_feedback(fb)

                        time.sleep(self.sleep_step_s)

                    # --- between-phase dwell (uniform semantics) ---
                    # priority: per-phase after_wait_s → per-gait between_phase_wait_s → 0.0
                    after_wait = 0.0
                    try:
                        if 'after_wait_s' in ph:
                            after_wait = float(ph.get('after_wait_s', 0.0))
                        elif gait_between_wait > 0.0:
                            after_wait = float(gait_between_wait)
                    except Exception:
                        after_wait = 0.0

                    self._uniform_wait(after_wait, f'Between-phase dwell after phase {pi}', last_cmd)

                # end phases
            # end cycles

            gh.succeed()
            return Locomotion.Result(success=True, message='All phases complete')

        except Exception as e:
            tb = traceback.format_exc()
            self.get_logger().error(f'Exception in execute_cb: {e}\n{tb}')
            try:
                gh.abort()
            except Exception:
                pass
            return Locomotion.Result(success=False, message=f'Exception: {e!r}')


def main():
    rclpy.init()
    node = LocomotionTaskServer()
    try:
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

