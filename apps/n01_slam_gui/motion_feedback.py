"""Execution feedback, independent of ROS and the browser event loop."""
import math


def command_conflicts(publishers):
    return sorted(p for p in publishers if p.rsplit('/', 1)[-1] not in
                  {'n01_slow_motion_executor', 'n01_bounded_motion_test'})


class MotionFeedback:
    def __init__(self):
        self.state = None
        self.last_progress = float('-inf')
        self.started = None
        self.last_rx = None
        self.terminal = False
        self.timeout_reported = False

    def feed(self, data, now):
        state = str(data.get('state', 'UNKNOWN'))
        changed = state != self.state
        self.state = state
        self.last_rx = now
        self.timeout_reported = False
        if self.started is None:
            self.started = now
        if self.terminal:
            return []
        if not changed and (state != 'MOVING' or now - self.last_progress < 5):
            return []
        self.last_progress = now
        reason = str(data.get('reason') or '')
        labels = {'MOVING': '正在沿动态更新路径行驶', 'ARRIVED': '到达目标成功',
                  'COMMAND_SOURCE_CONFLICT': '行驶受阻：速度指令源冲突',
                  'ESTOP': '行驶暂停：急停生效', 'SESSION_COMPLETE': '执行结束，未确认到达目标',
                  'DISABLED': '执行器未启用', 'DRY_RUN': '模拟执行，未输出底盘速度'}
        labels.update({'ROTATING':'正在原地对齐站点朝向','ROTATION_BLOCKED':'旋转暂停：扫掠空间或感知条件不满足','ROTATION_INPUT_STALE':'旋转暂停：里程计过期','ROTATION_POSE_MISMATCH':'旋转暂停：地图与里程计航向不一致','MANUAL_OVERRIDE': '手动控制已接管，导航已让出速度输出；结束手动操作后点击开始恢复导航',
                       'WAITING_COMMAND_OBSERVER': '等待速度指令来源监控就绪，暂不输出运动指令',
                       'NO_POSE': '行驶暂停：定位位姿过期或缺失',
                       'NO_ODOM': '行驶暂停：底盘里程计过期或缺失',
                       'NOT_READY': '行驶暂停：导航就绪条件不满足',
                       'BLOCKED': '行驶暂停：导航安全门控阻止运动',
                       'NO_PATH': '行驶暂停：缺少有效路径',
                       'REPLANNING_AFTER_ROTATION_BLOCKED':'旋转受阻，正在重新规划平移绕行',
                       'RECOVERY_FAILED':'行驶失败：旋转受阻且重新规划未找到可执行绕行',
                       'ROTATION_TIMEOUT':'行驶失败：旋转任务超时',
                       'ROTATION_DRIFT':'行驶失败：旋转中机身偏移超限',
                       'STOPPING_FOR_TURN':'正在停车，确认停稳后转向',
                       'NO_VALID_TRAJECTORY': '行驶暂停：局部轨迹尚未有效',
                       'VERIFYING_ARRIVAL': '正在确认到达，保持零速',
                       'WAITING_FOR_PATH_EXTENSION': '行驶暂停：等待局部有效路径延伸',
                       'WAITING_FOR_NEW_GOAL_PATH': '等待新目标路径，旧路径不可执行',
                       'CHASSIS_KINEMATICS_BLOCKED': '行驶暂停：底盘运动学约束不满足',
                       'CBF_SENSOR_STALE': '行驶暂停：CBF 所需传感器数据过期',
                       'CBF_INVALID_TRACK': '行驶暂停：CBF 障碍物跟踪数据无效',
                       'ALIGN': '正在调整朝向'})
        text = labels.get(state, '行驶暂停/等待：' + state)
        if reason:
            text += '；原因：' + reason
        if data.get('tracking_continuity')=='PREDICTED_TRACKING':
            text += ' · 里程计预测跟踪（等待全局定位修正）'
        if isinstance(data.get('cbf_enabled'), bool):
            text += ' · CBF ' + ('开启' if data['cbf_enabled'] else '关闭（仅 SCAN）')
        for key, label, unit in [('goal_distance_m', '距目标', 'm'),
                                 ('travelled_m', '累计位移', 'm'),
                                 ('speed_mps', '速度', 'm/s')]:
            value = data.get(key)
            if isinstance(value, (float, int)) and math.isfinite(value):
                text += f' · {label} {value:.3f} {unit}'
        text += f' · 观测用时 {now - self.started:.1f}s'
        self.terminal = state in {'ARRIVED', 'SESSION_COMPLETE', 'RECOVERY_FAILED', 'ROTATION_TIMEOUT', 'ROTATION_DRIFT'}
        level = 'ok' if state == 'ARRIVED' else ('err' if state in {'RECOVERY_FAILED','ROTATION_TIMEOUT','ROTATION_DRIFT'} else ('info' if state == 'MOVING' else 'warn'))
        return [dict(text=text, level=level, notify=self.terminal,
                     terminal=self.terminal)]

    def watchdog(self, now):
        limit = 10 if self.started is None else 3
        if self.terminal or self.last_rx is None or self.timeout_reported or now - self.last_rx <= limit:
            return []
        self.timeout_reported = True
        text = ('执行器启动超过 10 秒尚无反馈；请检查进程启动日志。' if self.started is None else
                '执行反馈中断超过 3 秒，无法确认运动状态或到达结果；请检查执行器和 ROS 连接。')
        return [dict(text=text,
                     level='err', notify=False, terminal=False)]
