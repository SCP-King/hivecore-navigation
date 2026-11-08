"""Pure, testable geometry and data contracts for the N-01 live stack."""
import math
from bisect import bisect_left
import numpy as np


def localization_is_tracking(status):
    """Validate only fields actually represented by the external LocStatus."""
    required = ("map_loaded", "session", "reloc_ok", "reloc_fitness")
    if not isinstance(status, dict) or any(key not in status for key in required):
        return False, "localization_status_contract_mismatch"
    if str(status["map_loaded"]).lower() != "true":
        return False, "localization_map_not_loaded"
    if str(status["session"]).lower() != "tracking":
        return False, "localization_not_tracking"
    if str(status["reloc_ok"]).lower() != "true":
        return False, "localization_relocalization_not_ok"
    try:
        fitness = float(status["reloc_fitness"])
    except (TypeError, ValueError, OverflowError):
        return False, "localization_reloc_fitness_invalid"
    if not math.isfinite(fitness) or fitness < 0.0:
        return False, "localization_reloc_fitness_invalid"
    return True, "tracking_ready"


def select_latest_tf_stamp(requested_stamp_ns, tf_stamp_ns, max_lag_s):
    """Resolve a live pose to the actual latest TF sample without falsifying time.

    A zero TF stamp denotes a static transform and is valid at the requested
    odometry time. Dynamic TF may trail odometry by a bounded amount; callers
    must use odometry interpolated at the returned TF timestamp.
    """
    try:
        requested = int(requested_stamp_ns)
        transform = int(tf_stamp_ns)
        max_lag = float(max_lag_s)
    except (TypeError, ValueError, OverflowError):
        return None
    if (
        requested <= 0 or transform < 0
        or not math.isfinite(max_lag) or max_lag < 0.0
    ):
        return None
    if transform == 0:
        return requested
    if transform > requested:
        return None
    lag_s = (requested - transform) / 1e9
    if lag_s > max_lag:
        return None
    return transform


def select_pose_anchor(samples, stamp_ns, max_delta_s):
    """Select a nearby SLAM anchor; prefer a causal previous sample."""
    if not samples:
        return None
    try:
        target = int(stamp_ns)
        max_delta = float(max_delta_s)
    except (TypeError, ValueError, OverflowError):
        return None
    if target <= 0 or not math.isfinite(max_delta) or max_delta < 0.0:
        return None
    stamps = [int(sample[0]) for sample in samples]
    index = bisect_left(stamps, target)
    if index < len(samples) and stamps[index] == target:
        return samples[index]
    if index > 0:
        previous = samples[index - 1]
        if 0 <= (target - int(previous[0])) / 1e9 <= max_delta:
            return previous
    if index < len(samples):
        following = samples[index]
        if 0 <= (int(following[0]) - target) / 1e9 <= max_delta:
            return following
    return None

def rotate_translate(point, translation, quaternion):
    """Apply a normalized xyzw rigid transform to one XYZ point."""
    x, y, z = (float(v) for v in point)
    tx, ty, tz = (float(v) for v in translation)
    qx, qy, qz, qw = (float(v) for v in quaternion)
    qn = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(qn) or qn < 1e-9:
        raise ValueError("invalid_transform_quaternion")
    qx, qy, qz, qw = qx/qn, qy/qn, qz/qn, qw/qn
    ix = qw*x + qy*z - qz*y
    iy = qw*y + qz*x - qx*z
    iz = qw*z + qx*y - qy*x
    iw = -qx*x - qy*y - qz*z
    rx = ix*qw + iw*(-qx) + iy*(-qz) - iz*(-qy)
    ry = iy*qw + iw*(-qy) + iz*(-qx) - ix*(-qz)
    rz = iz*qw + iw*(-qz) + ix*(-qy) - iy*(-qx)
    out = (rx+tx, ry+ty, rz+tz)
    if not all(math.isfinite(v) for v in out):
        raise ValueError("nonfinite_transformed_point")
    return out


def decode_pointcloud_xyz(msg, max_points=100000, max_range_m=None):
    """Decode x/y/z FLOAT32 or FLOAT64 fields, honoring offsets, endian and row padding."""
    width, height = int(msg.width), int(msg.height)
    point_step, row_step = int(msg.point_step), int(msg.row_step)
    total = width * height
    if width <= 0 or height <= 0 or point_step <= 0 or total <= 0:
        raise ValueError("cloud_empty_or_invalid_dimensions")
    if total > int(max_points):
        raise ValueError("cloud_point_count_out_of_bounds")
    if row_step < width * point_step:
        raise ValueError("cloud_row_step_invalid")
    data = memoryview(msg.data)
    if len(data) < row_step * height:
        raise ValueError("cloud_data_layout_invalid")

    fields = {str(f.name): f for f in msg.fields}
    names, formats, offsets = [], [], []
    endian = ">" if bool(msg.is_bigendian) else "<"
    for name in ("x", "y", "z"):
        if name not in fields:
            raise ValueError("cloud_missing_xyz")
        field = fields[name]
        datatype = int(field.datatype)
        if int(field.count) != 1 or datatype not in (7, 8):
            raise ValueError("cloud_xyz_must_be_float")
        size = 4 if datatype == 7 else 8
        offset = int(field.offset)
        if offset < 0 or offset + size > point_step:
            raise ValueError("cloud_xyz_offset_out_of_record")
        names.append(name)
        formats.append(np.dtype(endian + ("f4" if size == 4 else "f8")))
        offsets.append(offset)

    dtype = np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": point_step,
    })
    try:
        records = np.ndarray(
            shape=(height, width),
            dtype=dtype,
            buffer=data,
            strides=(row_step, point_step),
        )
    except (TypeError, ValueError, BufferError) as exc:
        raise ValueError("cloud_record_layout_invalid") from exc

    points = np.column_stack((
        records["x"].reshape(-1),
        records["y"].reshape(-1),
        records["z"].reshape(-1),
    ))
    mask = np.isfinite(points).all(axis=1)
    if max_range_m is not None:
        max_range = float(max_range_m)
        if not math.isfinite(max_range) or max_range <= 0:
            raise ValueError("invalid_max_range")
        mask &= np.einsum("ij,ij->i", points, points) <= max_range * max_range
    points = np.asarray(points[mask], dtype=np.float32)
    if points.shape[0] == 0:
        raise ValueError("cloud_has_no_usable_points")
    return np.ascontiguousarray(points)


def transform_points(points, translation, quaternion):
    """Vectorized rigid transform for an N x 3 float array."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3 or not np.isfinite(pts).all():
        raise ValueError("invalid_xyz_array")
    t = np.asarray(translation, dtype=np.float64)
    q = np.asarray(quaternion, dtype=np.float64)
    if t.shape != (3,) or q.shape != (4,) or not np.isfinite(t).all():
        raise ValueError("invalid_transform")
    qn = float(np.linalg.norm(q))
    if not math.isfinite(qn) or qn < 1e-9:
        raise ValueError("invalid_transform_quaternion")
    q = q / qn
    qv, qw = q[:3], q[3]
    uv = np.cross(qv, pts)
    uuv = np.cross(qv, uv)
    out = pts + 2.0 * (qw * uv + uuv) + t
    if not np.isfinite(out).all():
        raise ValueError("nonfinite_transformed_points")
    return np.ascontiguousarray(out, dtype=np.float32)


def estimate_velocity(previous, current, max_speed=3.0):
    """Finite-difference map-frame velocity; zero and invalid on bad dt."""
    if previous is None:
        return (0.0, 0.0, 0.0), False
    p_xyz, p_stamp = previous
    c_xyz, c_stamp = current
    dt = float(c_stamp) - float(p_stamp)
    if not math.isfinite(dt) or dt < 0.02 or dt > 0.5:
        return (0.0, 0.0, 0.0), False
    vel = tuple((float(c_xyz[i])-float(p_xyz[i]))/dt for i in range(3))
    speed = math.sqrt(sum(v*v for v in vel))
    if not all(math.isfinite(v) for v in vel) or speed > max_speed:
        return (0.0, 0.0, 0.0), False
    return vel, True



def normalize_quaternion(quaternion):
    """Return a finite unit quaternion in xyzw order."""
    q = np.asarray(quaternion, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("invalid_quaternion")
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid_quaternion_norm")
    return tuple(float(v) for v in (q / norm))


def pose_samples_equivalent(existing, position, orientation,
                            position_tolerance_m=0.02,
                            orientation_tolerance_rad=0.035):
    """Compare same-stamp pose samples, treating q and -q as equivalent."""
    try:
        old_position = np.asarray(existing[1], dtype=np.float64)
        new_position = np.asarray(position, dtype=np.float64)
        old_orientation = np.asarray(
            normalize_quaternion(existing[2]), dtype=np.float64)
        new_orientation = np.asarray(
            normalize_quaternion(orientation), dtype=np.float64)
        position_tolerance = float(position_tolerance_m)
        orientation_tolerance = float(orientation_tolerance_rad)
    except (IndexError, TypeError, ValueError, OverflowError):
        return False
    if (
        old_position.shape != (3,) or new_position.shape != (3,)
        or not np.isfinite(old_position).all()
        or not np.isfinite(new_position).all()
        or not math.isfinite(position_tolerance) or position_tolerance < 0.0
        or not math.isfinite(orientation_tolerance) or orientation_tolerance < 0.0
    ):
        return False
    position_delta = float(np.linalg.norm(old_position - new_position))
    dot = abs(float(np.dot(old_orientation, new_orientation)))
    orientation_delta = 2.0 * math.acos(min(1.0, max(-1.0, dot)))
    return bool(
        position_delta <= position_tolerance
        and orientation_delta <= orientation_tolerance
    )


def quaternion_multiply(a, b):
    """Compose rotations a * b in xyzw order."""
    ax, ay, az, aw = normalize_quaternion(a)
    bx, by, bz, bw = normalize_quaternion(b)
    return normalize_quaternion((
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    ))


def quaternion_slerp(a, b, fraction):
    """Shortest-arc spherical interpolation between xyzw quaternions."""
    q0 = np.asarray(normalize_quaternion(a), dtype=np.float64)
    q1 = np.asarray(normalize_quaternion(b), dtype=np.float64)
    u = float(fraction)
    if not math.isfinite(u) or u < 0.0 or u > 1.0:
        raise ValueError("quaternion_interpolation_fraction_out_of_range")
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion(q0 + u * (q1 - q0))
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    if abs(sin_theta) < 1e-9:
        return normalize_quaternion(q0)
    return normalize_quaternion(
        (math.sin((1.0-u)*theta) / sin_theta) * q0
        + (math.sin(u*theta) / sin_theta) * q1
    )


def compose_transform(parent_from_mid_translation, parent_from_mid_quaternion,
                      mid_from_child_translation, mid_from_child_quaternion):
    """Compose parent<-mid and mid<-child into parent<-child."""
    t1 = tuple(float(v) for v in parent_from_mid_translation)
    t2 = tuple(float(v) for v in mid_from_child_translation)
    if len(t1) != 3 or len(t2) != 3 or not all(
        math.isfinite(v) for v in t1 + t2
    ):
        raise ValueError("invalid_transform_translation")
    q1 = normalize_quaternion(parent_from_mid_quaternion)
    q2 = normalize_quaternion(mid_from_child_quaternion)
    translation = rotate_translate(t2, t1, q1)
    rotation = quaternion_multiply(q1, q2)
    return translation, rotation



def invert_transform(translation, quaternion):
    """Invert a parent<-child rigid transform (translation, xyzw quaternion)."""
    t = tuple(float(v) for v in translation)
    if len(t) != 3 or not all(math.isfinite(v) for v in t):
        raise ValueError("invalid_transform_translation")
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    inverse_q = (-qx, -qy, -qz, qw)
    inverse_t = rotate_translate(tuple(-v for v in t), (0.0, 0.0, 0.0), inverse_q)
    return inverse_t, inverse_q


def motion_compensated_pose(map_anchor, odom_anchor, odom_target):
    """Propagate a map-frame body pose anchor using relative odometry motion.

    All three poses must describe the same rigid body frame. The odometry
    parent frame cancels in inv(odom_anchor) * odom_target, so no map<->odom
    alignment or absolute odometry origin is assumed.
    """
    map_t, map_q = map_anchor
    anchor_t, anchor_q = odom_anchor
    target_t, target_q = odom_target
    inverse_anchor = invert_transform(anchor_t, anchor_q)
    relative_motion = compose_transform(
        inverse_anchor[0], inverse_anchor[1], target_t, target_q)
    return compose_transform(map_t, map_q, relative_motion[0], relative_motion[1])

def interpolate_pose(samples, stamp_ns, max_gap_s=0.5):
    """Interpolate a stamped pose at a requested timestamp; never extrapolate.

    Each sample is (stamp_ns, xyz_tuple, quaternion_xyzw_tuple). A single exact
    timestamp is accepted; otherwise two bracketing samples are required.
    """
    target = int(stamp_ns)
    gap_limit = float(max_gap_s)
    if target <= 0 or not math.isfinite(gap_limit) or gap_limit <= 0.0:
        raise ValueError("invalid_pose_interpolation_request")
    ordered = sorted(samples, key=lambda sample: int(sample[0]))
    if not ordered:
        return None
    for sample_stamp, xyz, quaternion in ordered:
        if int(sample_stamp) == target:
            position = tuple(float(v) for v in xyz)
            if len(position) != 3 or not all(math.isfinite(v) for v in position):
                raise ValueError("invalid_pose_position")
            return position, normalize_quaternion(quaternion)
    for left, right in zip(ordered[:-1], ordered[1:]):
        t0, p0, q0 = left
        t1, p1, q1 = right
        t0, t1 = int(t0), int(t1)
        if t0 < target < t1:
            gap = (t1 - t0) / 1e9
            if gap <= 0.0 or gap > gap_limit:
                return None
            alpha = (target - t0) / float(t1 - t0)
            p0 = np.asarray(p0, dtype=np.float64)
            p1 = np.asarray(p1, dtype=np.float64)
            if p0.shape != (3,) or p1.shape != (3,) or not (
                np.isfinite(p0).all() and np.isfinite(p1).all()
            ):
                raise ValueError("invalid_pose_position")
            position = tuple(float(v) for v in (p0 + alpha * (p1 - p0)))
            orientation = quaternion_slerp(q0, q1, alpha)
            return position, orientation
    return None


def cluster_xy_points(points, cell_size=0.30, min_points=6, max_clusters=64):
    """Connected components over occupied XY cells; returns arrays of real points."""
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.size == 0:
        return []
    cell = float(cell_size)
    if not math.isfinite(cell) or cell <= 0:
        raise ValueError("invalid_cluster_cell_size")
    cells = {}
    keys = np.floor(pts[:, :2] / cell).astype(np.int64)
    for index, key in enumerate(keys):
        cells.setdefault((int(key[0]), int(key[1])), []).append(index)

    visited = set()
    clusters = []
    for seed in cells:
        if seed in visited:
            continue
        stack = [seed]
        visited.add(seed)
        component = []
        while stack:
            current = stack.pop()
            component.extend(cells[current])
            cx, cy = current
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbor = (cx + dx, cy + dy)
                    if neighbor in cells and neighbor not in visited:
                        visited.add(neighbor)
                        stack.append(neighbor)
        if len(component) >= int(min_points):
            clusters.append(pts[np.asarray(component, dtype=np.int64)])
            if len(clusters) >= int(max_clusters):
                break
    return clusters


def four_wheel_steer_screen(path_xyz, max_speed=0.25, max_accel=0.25,
                            wheel_base_x=0.56, track_y=0.27,
                            wheel_radius=0.095, steering_limit=1.5708):
    """Necessary-condition screen using nominal hip-pivot geometry, never a controller model."""
    pts = np.asarray(path_xyz, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 3:
        raise ValueError("path_requires_at_least_three_xyz_points")
    if not np.isfinite(pts).all():
        raise ValueError("path_contains_nonfinite_points")
    params = (max_speed, max_accel, wheel_base_x, track_y, wheel_radius, steering_limit)
    if not all(math.isfinite(float(v)) and float(v) > 0 for v in params):
        raise ValueError("invalid_wheel_screen_parameters")

    step = np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1)
    keep = np.concatenate(([True], step > 1e-5))
    pts = pts[keep]
    if pts.shape[0] < 3:
        raise ValueError("path_has_insufficient_distinct_points")
    ds = np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1)
    arc = np.concatenate(([0.0], np.cumsum(ds)))
    segment_yaw = np.unwrap(np.arctan2(np.diff(pts[:, 1]), np.diff(pts[:, 0])))
    yaw = np.empty(pts.shape[0], dtype=np.float64)
    yaw[0], yaw[-1] = segment_yaw[0], segment_yaw[-1]
    if pts.shape[0] > 2:
        yaw[1:-1] = 0.5 * (segment_yaw[:-1] + segment_yaw[1:])
    curvature = np.gradient(yaw, arc, edge_order=1)

    speed = np.zeros(pts.shape[0], dtype=np.float64)
    for i in range(1, len(speed)):
        speed[i] = min(float(max_speed), math.sqrt(
            speed[i-1] * speed[i-1] + 2.0 * float(max_accel) * ds[i-1]))
    speed[-1] = 0.0
    for i in range(len(speed)-2, -1, -1):
        speed[i] = min(speed[i], math.sqrt(
            speed[i+1] * speed[i+1] + 2.0 * float(max_accel) * ds[i]))

    accel = np.abs(np.diff(speed * speed) / np.maximum(2.0 * ds, 1e-9))
    yaw_rate = speed * curvature
    wheel_positions = {
        "fl": (wheel_base_x / 2.0, track_y / 2.0),
        "fr": (wheel_base_x / 2.0, -track_y / 2.0),
        "rl": (-wheel_base_x / 2.0, track_y / 2.0),
        "rr": (-wheel_base_x / 2.0, -track_y / 2.0),
    }
    steering = {}
    wheel_linear = {}
    wheel_angular = {}
    for name, (wx, wy) in wheel_positions.items():
        vx = speed - yaw_rate * wy
        vy = yaw_rate * wx
        magnitude = np.hypot(vx, vy)
        angle = np.arctan2(vy, vx)
        # A wheel may reverse drive direction to keep its steering target inside
        # the URDF's +/-90 degree joint interval.
        reverse = angle > math.pi / 2.0
        angle = np.where(reverse, angle - math.pi, angle)
        reverse = angle < -math.pi / 2.0
        angle = np.where(reverse, angle + math.pi, angle)
        steering[name] = angle
        wheel_linear[name] = magnitude
        wheel_angular[name] = magnitude / float(wheel_radius)

    max_steer = max(float(np.max(np.abs(v))) for v in steering.values())
    max_wheel_linear = max(float(np.max(v)) for v in wheel_linear.values())
    max_wheel_angular = max(float(np.max(v)) for v in wheel_angular.values())
    max_path_accel = float(np.max(accel)) if accel.size else 0.0
    first_idx = next((i for i, v in enumerate(speed) if v >= 0.03), min(1, len(speed)-1))
    first_angles = {name: float(values[first_idx]) for name, values in steering.items()}
    return {
        "necessary_conditions_pass": bool(
            max_steer <= float(steering_limit) + 1e-6
            and float(np.max(speed)) <= float(max_speed) + 1e-6
            and max_path_accel <= float(max_accel) + 1e-3
        ),
        "geometry_basis": "nominal_hip_pivot_proxy_not_measured_wheel_contact_geometry",
        "max_path_speed_mps": float(np.max(speed)),
        "max_path_acceleration_mps2": max_path_accel,
        "max_abs_steering_angle_rad": max_steer,
        "max_wheel_linear_speed_mps": max_wheel_linear,
        "max_wheel_angular_speed_radps": max_wheel_angular,
        "wheel_steering_angles_rad": first_angles,
        "wheel_positions_m": {k: [float(v[0]), float(v[1])] for k, v in wheel_positions.items()},
        "controller_model_verified": False,
        "wheel_contact_geometry_verified": False,
        "collision_envelope_verified": False,
        "execution_authorized": False,
    }


def goal_is_valid(goal, *, live_ready, static_ready):
    """Pure fail-closed RViz goal contract."""
    if not live_ready:
        return False, "live_localization_or_cloud_not_ready"
    if not static_ready:
        return False, "native_scan_static_pcd_not_ready"
    if str(goal.header.frame_id).lstrip("/") != "map":
        return False, "goal_frame_must_be_map"
    p = goal.pose.position
    if not all(math.isfinite(float(v)) for v in (p.x, p.y, p.z)):
        return False, "goal_nonfinite"
    if float(p.z) < -0.1:
        return False, "goal_z_below_scan_ground_guard"
    return True, "accepted"


def scan_output_is_fresh(last_processed_stamp_ns, latest_input_stamp_ns,
                         output_age_s, input_age_s, max_cloud_age_s,
                         max_scan_lag_s):
    """Accept bounded scan-processing lag while rejecting stale data."""
    if last_processed_stamp_ns is None or latest_input_stamp_ns is None:
        return False, None
    try:
        processed = int(last_processed_stamp_ns)
        latest = int(latest_input_stamp_ns)
        output_age = float(output_age_s)
        input_age = float(input_age_s)
        max_age = float(max_cloud_age_s)
        max_lag = float(max_scan_lag_s)
    except (TypeError, ValueError, OverflowError):
        return False, None
    values = (output_age, input_age, max_age, max_lag)
    if not all(math.isfinite(v) for v in values) or max_age <= 0.0 or max_lag <= 0.0:
        return False, None
    lag_s = (latest - processed) / 1e9
    fresh = (
        processed > 0 and latest >= processed
        and -0.2 <= output_age <= max_age
        and -0.2 <= input_age <= max_age
        and 0.0 <= lag_s <= max_lag
    )
    return bool(fresh), float(lag_s)


def trajectory_window_is_active(start_stamp_ns, now_ns, duration_s,
                                future_tolerance_s=0.25, end_grace_s=0.2):
    """Check whether a stamped B-spline is within its active time domain."""
    try:
        start = int(start_stamp_ns)
        now = int(now_ns)
        duration = float(duration_s)
        future_tolerance = float(future_tolerance_s)
        end_grace = float(end_grace_s)
    except (TypeError, ValueError, OverflowError):
        return False, None
    if (
        start <= 0 or now <= 0
        or not math.isfinite(duration) or duration <= 0.0
        or not math.isfinite(future_tolerance) or future_tolerance < 0.0
        or not math.isfinite(end_grace) or end_grace < 0.0
    ):
        return False, None
    age_s = (now - start) / 1e9
    if not math.isfinite(age_s):
        return False, None
    active = -future_tolerance <= age_s <= duration + end_grace
    return bool(active), float(age_s)


def sensor_cloud_stamp_gate(stamp_ns, now_ns, max_age_s,
                            previous_stamp_ns=None, future_tolerance_s=0.2):
    """Validate cloud age and monotonic source time without changing its stamp."""
    try:
        stamp = int(stamp_ns)
        now = int(now_ns)
        max_age = float(max_age_s)
        future_tolerance = float(future_tolerance_s)
        previous = None if previous_stamp_ns is None else int(previous_stamp_ns)
    except (TypeError, ValueError, OverflowError):
        return False, "cloud_stamp_invalid", None
    if (
        stamp <= 0 or now <= 0
        or not math.isfinite(max_age) or max_age <= 0.0
        or not math.isfinite(future_tolerance) or future_tolerance < 0.0
    ):
        return False, "cloud_stamp_invalid", None
    age_s = (now - stamp) / 1e9
    if not math.isfinite(age_s):
        return False, "cloud_stamp_invalid", None
    if age_s < -future_tolerance:
        return False, "cloud_stamp_in_future", float(age_s)
    if age_s > max_age:
        return False, "cloud_stamp_stale", float(age_s)
    if previous is not None and stamp <= previous:
        return False, "cloud_stamp_not_monotonic", float(age_s)
    return True, "accepted", float(age_s)


def stamped_data_is_fresh(stamp_ns, now_ns, max_age_s, future_tolerance_s=0.2):
    """Validate a sensor/source timestamp against the node's ROS clock."""
    try:
        stamp = int(stamp_ns)
        now = int(now_ns)
        max_age = float(max_age_s)
        future_tolerance = float(future_tolerance_s)
    except (TypeError, ValueError, OverflowError):
        return False, None
    if (
        stamp <= 0 or now <= 0
        or not math.isfinite(max_age) or max_age <= 0.0
        or not math.isfinite(future_tolerance) or future_tolerance < 0.0
    ):
        return False, None
    age_s = (now - stamp) / 1e9
    if not math.isfinite(age_s):
        return False, None
    return bool(-future_tolerance <= age_s <= max_age), float(age_s)


def evaluate_safety_gate(*, health_fresh, navigation_ready,
                         feasibility_fresh, feasibility_status,
                         candidate_review_ready, operator_hold,
                         physical_checks, dynamic_obstacle_fresh=False,
                         dynamic_obstacle_status="UNKNOWN"):
    """Report a review-only safety state; never authorize robot motion.

    Dynamic diagnostics are deliberately fail-closed: missing, stale, invalid,
    or candidate-bearing observations require review. This is a software
    acceptance gate only and never commands a stop or authorizes motion.
    """
    blockers = []
    dynamic_status = str(dynamic_obstacle_status)
    dynamic_status_valid = dynamic_status in (
        "NO_UNMATCHED_CLUSTERS", "CANDIDATES_AVAILABLE")
    if bool(operator_hold):
        blockers.append("operator_hold_active")
    if not bool(health_fresh):
        blockers.append("system_health_missing_or_stale")
    if not bool(navigation_ready):
        blockers.append("navigation_not_ready")
    if not bool(feasibility_fresh):
        blockers.append("candidate_feasibility_missing_or_stale")
    elif str(feasibility_status) != "NECESSARY_CONDITIONS_PASS":
        blockers.append("candidate_kinematic_screen_not_passed")
    if not bool(candidate_review_ready):
        blockers.append("candidate_review_feedback_incomplete")
    if not bool(dynamic_obstacle_fresh):
        blockers.append("dynamic_obstacle_status_missing_or_stale")
    elif not dynamic_status_valid:
        blockers.append("dynamic_obstacle_status_invalid_or_waiting")
    elif dynamic_status == "CANDIDATES_AVAILABLE":
        blockers.append("dynamic_obstacle_candidates_require_review")
    required = (
        "emergency_stop_chain_verified",
        "communication_loss_stop_verified",
        "localization_loss_stop_verified",
        "dynamic_obstacle_stop_verified",
        "manual_takeover_verified",
        "wheel_controller_mapping_verified",
        "wheel_contact_geometry_verified",
        "collision_envelope_verified",
        "braking_distance_verified",
    )
    checks = physical_checks if isinstance(physical_checks, dict) else {}
    missing = [name for name in required if checks.get(name) is not True]
    blockers.extend("physical_acceptance_missing:" + name for name in missing)
    if bool(operator_hold):
        status = "OPERATOR_HOLD"
    elif not bool(health_fresh) or not bool(navigation_ready):
        status = "NAVIGATION_BLOCKED"
    elif not bool(feasibility_fresh) or str(feasibility_status) != "NECESSARY_CONDITIONS_PASS":
        status = "CANDIDATE_NOT_READY"
    elif not bool(candidate_review_ready):
        status = "CANDIDATE_REVIEW_INCOMPLETE"
    elif not bool(dynamic_obstacle_fresh) or not dynamic_status_valid:
        status = "DYNAMIC_OBSTACLE_STATUS_NOT_READY"
    elif dynamic_status == "CANDIDATES_AVAILABLE":
        status = "DYNAMIC_OBSTACLE_REVIEW_REQUIRED"
    elif missing:
        status = "PHYSICAL_ACCEPTANCE_INCOMPLETE"
    else:
        status = "REVIEW_ONLY_NOT_AUTHORIZED"
    return {
        "status": status,
        "blockers": blockers,
        "physical_acceptance_missing": missing,
        "dynamic_obstacle_status": dynamic_status,
        "dynamic_obstacle_status_fresh": bool(dynamic_obstacle_fresh),
        "dynamic_obstacle_review_required": bool(dynamic_status == "CANDIDATES_AVAILABLE"),
        "execution_authorized": False,
        "actuator_output": "DISABLED",
        "safety_authorization": "NEVER",
    }
