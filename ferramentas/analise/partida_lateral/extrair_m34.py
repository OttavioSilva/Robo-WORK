"""Extrai da missao34 (11:03:10-11:04:10) cmd_vel_nav/cmd_vel/cmd_vel_hardware, PWM das 4 rodas (tx_frames),
IMU (yaw, gyro z) e paredes frente/tras pelo /scan_front. Salva JSON."""
import sys, math, json, struct, time, datetime
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, saida = sys.argv[1], sys.argv[2]
d0 = datetime.datetime(2026, 11, 11, 11, 3, 5).timestamp()
d1 = datetime.datetime(2026, 11, 11, 11, 4, 12).timestamp()
TOP = ['/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel', '/cmd_vel_hardware', '/hardware/tx_frames', '/imu/data', '/scan_front', '/rosout', '/odom']
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
T = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=TOP))
# mcap: pula ate d0
try:
    r.seek(int(d0 * 1e9))
except Exception:
    pass
out = {k: [] for k in TOP}
LX, LY = 0.33, 0.005
while r.has_next():
    top, d, tn = r.read_next()
    t = tn * 1e-9
    if t < d0:
        continue
    if t > d1:
        break
    m = deserialize_message(d, T[top])
    if top in ('/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel', '/cmd_vel_hardware'):
        tw = m.twist if hasattr(m, 'twist') else m
        out[top].append((t, tw.linear.x, tw.linear.y, tw.angular.z))
    elif top == '/hardware/tx_frames':
        b = bytes(m.data)
        if len(b) >= 17 and b[3] is not None:
            pwm = struct.unpack('<4h', b[8:16])
            out[top].append((t, b[3]) + pwm + (b[16],))
    elif top == '/imu/data':
        q = m.orientation
        y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        out[top].append((t, y, m.angular_velocity.z, m.linear_acceleration.x, m.linear_acceleration.y))
    elif top == '/odom':
        out[top].append((t, m.pose.pose.position.x, m.pose.pose.position.y))
    elif top == '/scan_front':
        fr, tr = [], []
        a = m.angle_min
        for rr in m.ranges:
            if math.isfinite(rr) and m.range_min < rr < 2.0:
                x, yy = LX + rr * math.cos(a), LY + rr * math.sin(a)
                if abs(yy) < 0.22:
                    if 0.36 < x < 1.3:
                        fr.append(x)
                    elif -1.2 < x < -0.2:
                        tr.append(x)
            a += m.angle_increment
        fr.sort(); tr.sort()
        out[top].append((t, fr[len(fr)//2] if len(fr) >= 3 else None, tr[len(tr)//2] if len(tr) >= 3 else None, len(fr), len(tr)))
    elif top == '/rosout':
        if m.name in ('alinhador_passagem', 'collision_monitor'):
            out[top].append((t, m.name, m.msg))
json.dump(out, open(saida, 'w'))
print({k: len(v) for k, v in out.items()})
