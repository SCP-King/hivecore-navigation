"""Pure Python safety gate and PointCloud2 XYZ transform primitives.

No ROS imports, motor commands, TF lookups or disk IO.
Only registered x/y/z FLOAT32 fields may be transformed. The rest is preserved.
"""
import math
import struct
import time


def lookup_path(obj, dotted):
    if not dotted or any(not part for part in dotted.split(".")):
        raise ValueError("status message field path must be explicit")
    value = obj
    for key in dotted.split("."):
        if not hasattr(value, key):
            raise AttributeError("missing LocalizationStatus member: " + dotted)
        value = getattr(value, key)
    return value


class ReadinessGate:
    """Fail closed for missing, stale, malformed or non-tracking status."""

    def __init__(self, timeout_s=1.0, outer_path=None, inner_path=None,
                 tf_flag_path="registration_output_tf_published",
                 outer_value="Localization", inner_value="tracking",
                 fixture=False):
        if not 0.1 <= timeout_s <= 30:
            raise ValueError("timeout_s out of range")
        self.timeout_s = float(timeout_s)
        self.outer_path = outer_path or ""
        self.inner_path = inner_path or ""
        self.tf_flag_path = tf_flag_path
        self.outer_value = outer_value
        self.inner_value = inner_value
        self.fixture = fixture
        self.last_time = None
        self.last_ok = False
        self.reason = "no_status"

    def observe(self, msg, received_monotonic=None):
        now = time.monotonic() if received_monotonic is None else received_monotonic
        self.last_time = now
        try:
            if self.fixture:
                # Test-only std_msgs/Bool; never accepted for real SLAM status.
                self.last_ok = type(msg.data) is bool and msg.data is True
                self.reason = "offline_fixture_ready" if self.last_ok else "fixture_not_ready"
            else:
                # Explicit paths prevent guessing undocumented .msg layouts.
                outer = lookup_path(msg, self.outer_path)
                inner = lookup_path(msg, self.inner_path)
                tf = lookup_path(msg, self.tf_flag_path)
                self.last_ok = (outer == self.outer_value and inner == self.inner_value
                                and type(tf) is bool and tf is True)
                self.reason = "localization_tracking" if self.last_ok else "localization_not_tracking"
        except (AttributeError, ValueError, TypeError) as exc:
            self.last_ok = False
            self.reason = "status_contract_mismatch:" + str(exc)
        return self.last_ok

    def check(self, now=None):
        current = time.monotonic() if now is None else now
        if self.last_time is None:
            return False, "no_status"
        age = current - self.last_time
        if not (0 <= age <= self.timeout_s):
            return False, "status_stale_or_clock_reversed"
        if not self.last_ok:
            return False, self.reason
        return True, self.reason


def quaternion_rotate(q, point):
    x, y, z, w = q
    n = x*x + y*y + z*z + w*w
    if not math.isfinite(n) or n < 1e-12:
        raise ValueError("invalid quaternion")
    x, y, z, w = (v / math.sqrt(n) for v in (x, y, z, w))
    px, py, pz = point
    # R(q)*p = p + 2*w*(qv cross p) + 2*(qv cross (qv cross p)).
    tx, ty, tz = (2*(y*pz-z*py), 2*(z*px-x*pz), 2*(x*py-y*px))
    return (px + w*tx + y*tz - z*ty,
            py + w*ty + z*tx - x*tz,
            pz + w*tz + x*ty - y*tx)


def xyz_transform_bytes(data, width, height, point_step, row_step, fields,
                        is_bigendian, translation, quaternion, max_points=500000):
    """Transform XYZ FLOAT32 in a PointCloud2 layout, preserving other fields.

    Handles row padding, non-standard field offsets, endianness and organized
    clouds. Non-finite coordinates are preserved unchanged. Raises on
    unsupported/corrupt layouts rather than publishing mislabeled data.
    """
    if width < 0 or height < 0 or point_step < 12 or row_step < width*point_step:
        raise ValueError("invalid cloud dimensions")
    if width * height > max_points:
        raise ValueError("cloud point limit exceeded")
    if row_step * height > len(data):
        raise ValueError("truncated cloud data")
    offsets = {}
    for f in fields:
        name, offset, datatype, count = f
        if name in ("x", "y", "z"):
            if datatype != 7 or count != 1 or offset < 0 or offset+4 > point_step:
                raise ValueError("XYZ fields must be FLOAT32 within point_step")
            if name in offsets:
                raise ValueError("duplicate XYZ field")
            offsets[name] = offset
    if set(offsets) != {"x", "y", "z"} or len(set(offsets.values())) != 3:
        raise ValueError("missing or overlapping XYZ fields")
    if not all(math.isfinite(v) for v in translation):
        raise ValueError("invalid transform translation")
    order = ">" if is_bigendian else "<"
    f32 = struct.Struct(order + "f")
    output = bytearray(data)
    for row in range(height):
        for col in range(width):
            base = row*row_step + col*point_step
            xyz = [f32.unpack_from(output, base+offsets[k])[0] for k in ("x","y","z")]
            if not all(math.isfinite(v) for v in xyz):
                continue
            rotated = quaternion_rotate(quaternion, xyz)
            transformed = [rotated[i]+translation[i] for i in range(3)]
            if not all(math.isfinite(v) for v in transformed):
                raise ValueError("non-finite transformed XYZ")
            for key, value in zip(("x","y","z"), transformed):
                f32.pack_into(output, base+offsets[key], value)
    return bytes(output)
