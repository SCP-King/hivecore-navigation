"""GUI descriptions and read-only inspection of recorded sensor sources."""
from pathlib import Path
import math
import yaml


def is_realtime_backend(mode, running=True, map_mode=None):
    return bool(running and mode in ('mapping', 'fast_lio_realtime')
                and map_mode != 'offline')


def slam_selection(fast_lio, task, realtime):
    """Offline mapping/localization or one realtime SLAM session."""
    if task not in ('建图', '定位'):
        raise ValueError('未知建图定位任务')
    if realtime and task != '建图':
        raise ValueError('实时模式统一启动SLAM（建图与位姿估计），不单独启动定位')
    return dict(backend='fast_lio' if fast_lio else 'vendor', task=task,
                map_mode='realtime' if realtime else 'offline',
                engine_mode=('realtime' if fast_lio else 'mapping') if realtime
                            else ('mapping' if task == '建图' else 'loc'),
                record_bag=task == '建图', needs_pcd=not realtime and task == '定位',
                navigation_allowed=realtime or task == '定位')


def workflow_help(fast_lio, realtime):
    backend = '自部署 FAST-LIO2' if fast_lio else '甲方 SLAM3D Lite'
    if realtime:
        return dict(summary=f'{backend} · 实时SLAM（建图＋位姿估计）',
                    workflow='① 启动实时SLAM → 等待点云与位姿就绪、同步地面 → ② 手动启动导航 → 选点规划 → ③ 手动启动行驶。无需加载旧PCD或重定位。',
                    map_action='实时模式无需加载历史地图', needs_pcd=False)
    return dict(summary=f'{backend} · 离线地图流程',
                workflow='① 建图并结束保存PCD（已有地图可跳过）→ ② 加载地图定位并进入跟踪 → ③ 手动启动导航 → 选点规划后手动启动行驶。',
                map_action='选择/加载 PCD 地图', needs_pcd=True)


def mapping_ready(status, receive_age):
    try:
        source_age = float(status.get('source_status_age_s', 'inf'))
    except (AttributeError, TypeError, ValueError):
        return False
    return bool(isinstance(status, dict)
                and status.get('readiness_semantics') == 'mapping_session_and_fresh_pose'
                and status.get('session') == 'mapping'
                and status.get('status_mapping_valid') == 'true'
                and status.get('mapping_pose_available', status.get('mapping_pose_fresh')) == 'true'
                and -0.2 <= source_age <= 1.0
                and receive_age is not None and 0 <= receive_age <= 1.0)


def localization_help(fast_lio=False, realtime=False):
    if not fast_lio and realtime:
        return {
            'summary': '甲方 SLAM3D Lite · 实时建图导航，使用本次运行坐标系',
            'workflow': '启动建图 → 等待位姿与实时点云就绪 → 同步地面 → 启动导航 → 地图选点规划；无需加载旧 PCD 或重定位。',
            'map_action': '② 实时建图无需加载历史地图',
        }
    if fast_lio and realtime:
        return {
            'summary': '自部署 FAST-LIO2 · 实时建图定位，使用本次运行地图',
            'workflow': '启动实时建图定位 → 数据链就绪 → 启动导航；无需加载历史 PCD 或 bag。',
            'map_action': '② 历史地图不适用',
        }
    if fast_lio:
        return {
            'summary': '自部署 FAST-LIO2 · 离线地图定位，PCD + 当前雷达 / IMU',
            'workflow': '先选择 PCD → 启动定位 → RViz 设置初始位姿 → ICP 匹配后自动跟踪 → 启动导航。更换 PCD 需先停止后端。',
            'map_action': '② 选择参考地图（启动时加载）',
        }
    return {
        'summary': '甲方 SLAM3D Lite · 离线地图定位，PCD + 当前雷达 / IMU',
        'workflow': '启动定位 → 加载 PCD → RViz 设置初始位姿 → 匹配成功后进入跟踪 → 启动导航。',
        'map_action': '② 加载 PCD 地图',
    }


def inspect_bag(path):
    """Read metadata and file presence only; never replay or publish messages."""
    directory = Path(path).expanduser().resolve(strict=True)
    metadata = directory if directory.name == 'metadata.yaml' else directory / 'metadata.yaml'
    if not metadata.is_file():
        raise ValueError('请选择含 metadata.yaml 的 rosbag2 目录')
    directory = metadata.parent
    if metadata.stat().st_size > 4 * 1024 * 1024:
        raise ValueError('bag 元数据过大')
    try:
        with metadata.open() as stream:
            info = yaml.safe_load(stream)['rosbag2_bagfile_information']
    except (yaml.YAMLError, TypeError, KeyError) as exc:
        raise ValueError('bag 元数据格式无效') from exc
    files = info.get('relative_file_paths', [])
    if not files or any(not (directory / name).is_file() for name in files):
        raise ValueError('bag 数据文件缺失，不能用于完整回放')
    count = int(info['message_count'])
    duration = int(info['duration']['nanoseconds']) / 1e9
    if count <= 0 or duration <= 0:
        raise ValueError('bag 没有有效记录')
    topics = {t['topic_metadata']['name']: int(t['message_count'])
              for t in info['topics_with_message_count']}
    warnings = []
    if not topics.get('/livox/lidar') or not topics.get('/livox/imu'):
        warnings.append('原始雷达或 IMU 缺失，不能直接作为当前 FAST-LIO2 回放输入')
    elif topics.get('/imu/data', 0) > 2 * topics['/livox/imu']:
        warnings.append('原始 IMU 记录明显少于滤波 IMU，重建前需检查时间连续性')
    return {'path': str(directory), 'duration_s': duration, 'message_count': count,
            'topics': topics, 'warnings': warnings}


def route_summary(route):
    points=route['points']
    names=' → '.join(p['name'] for p in points[:5])
    if len(points)>5:names+=' …'
    lengths=[math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(points,points[1:])]
    distance=' / '.join(f'{d:.2f}m' for d in lengths[:4]) or '单站'
    waits=' / '.join(f'{p["wait_s"]:g}s' for p in points[:5])
    end='结束后再回首点' if route['return_to_start'] else '终点结束，不再返回'
    return f'先到 {points[0]["name"]}，再按 {names} 执行 · 站间 {distance} · 各站停留 {waits} · {end}'
