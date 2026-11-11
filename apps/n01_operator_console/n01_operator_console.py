#!/usr/bin/env python3
"""N-01 operator console.

A single-file, stdlib-only web console for the N-01 navigation stack.

  GET  /                operator page
  GET  /api/state       live snapshot (pose, status, cloud, bspline)
  POST /api/goal        {"x":..,"y":..}      -> /move_base_simple/goal
  POST /api/enable      {"on":true|false}     -> /n01/traj_tracker/enable
  POST /api/estop                             -> disable tracking

Nothing is published to /cmd_vel here. The console can only:
  * send a goal into the fail-closed goal gate
  * toggle the trajectory tracker's enable flag
Both are operator actions; the gate and the tracker still apply their own checks.
"""
from __future__ import annotations

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from scan_planner_msgs.msg import Bspline

MAX_CLOUD_POINTS = 8000
MAX_TRAJ_POINTS = 200
VOXEL_XY = 0.1


class Store:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0, "stamp": 0.0}
        self.have_pose = False
        self.tracker = {}
        self.gate = {}
        self.ready = False
        self.cloud = []
        self.cloud_ts = 0.0
        self.cloud_fb = []
        self.cloud_fb_ts = 0.0
        self.cloud_accum = {}
        self.pose_fb = {"x": 0.0, "y": 0.0, "yaw": 0.0, "stamp": 0.0}
        self.pose_fb_ts = 0.0
        self.traj = []
        self.traj_stamp = 0.0
        self.goal = None


STORE = Store()


class ConsoleNode(Node):
    def __init__(self) -> None:
        super().__init__("n01_operator_console")

        cloud_qos = QoSProfile(depth=5)
        cloud_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        cloud_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(Odometry, "/body_pose", self.on_pose, 20)
        self.create_subscription(String, "/n01/traj_tracker/status", self.on_tracker, 10)
        self.create_subscription(String, "/n01/live_navigation/goal_gate_status", self.on_gate, 10)
        self.create_subscription(Bool, "/n01/live_navigation/navigation_ready", self.on_ready, 10)
        self.create_subscription(PointCloud2, "/cloud", self.on_cloud, cloud_qos)
        self.create_subscription(PointCloud2, "/cloud_registered", self.on_cloud_fb, cloud_qos)
        self.create_subscription(Odometry, "/Odometry", self.on_pose_fb, cloud_qos)
        self.create_subscription(Bspline, "/planning/bspline", self.on_bspline, 10)

        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.enable_pub = self.create_publisher(Bool, "/n01/traj_tracker/enable", 10)
        self.get_logger().warn("operator console ready")

    def on_pose(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with STORE.lock:
            STORE.pose = {"x": round(p.x, 3), "y": round(p.y, 3),
                          "yaw": round(yaw, 3), "stamp": round(stamp, 3)}
            STORE.have_pose = True

    def on_tracker(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with STORE.lock:
            STORE.tracker = data

    def on_gate(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with STORE.lock:
            STORE.gate = data

    def on_ready(self, msg: Bool) -> None:
        with STORE.lock:
            STORE.ready = bool(msg.data)

    def on_cloud(self, msg: PointCloud2) -> None:
        flat = self._decode(msg)
        if flat is None:
            return
        with STORE.lock:
            STORE.cloud = flat
            STORE.cloud_ts = time.time()

    def on_cloud_fb(self, msg: PointCloud2) -> None:
        flat = self._decode(msg)
        if flat is None:
            return
        inv = 1.0 / VOXEL_XY
        with STORE.lock:
            for i in range(0, len(flat) - 1, 2):
                x = flat[i]
                y = flat[i + 1]
                STORE.cloud_accum[(int(round(x * inv)), int(round(y * inv)))] = (x, y)
            STORE.cloud_fb = flat
            STORE.cloud_fb_ts = time.time()

    def reset_accum(self) -> dict:
        with STORE.lock:
            n = len(STORE.cloud_accum)
            STORE.cloud_accum.clear()
        return {"ok": True, "cleared": n}

    def on_pose_fb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        now = time.time()
        with STORE.lock:
            STORE.pose_fb = {"x": round(p.x, 3), "y": round(p.y, 3),
                             "yaw": round(yaw, 3), "stamp": round(now, 3)}
            STORE.pose_fb_ts = now

    def _decode(self, msg: PointCloud2):
        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True)
        except Exception:
            return None
        total = pts.shape[0]
        if total == 0:
            return None
        step = max(1, total // MAX_CLOUD_POINTS)
        sub = pts[::step][:MAX_CLOUD_POINTS]
        flat = []
        for row in sub:
            flat.append(round(float(row[0]), 2))
            flat.append(round(float(row[1]), 2))
        return flat

    def on_bspline(self, msg: Bspline) -> None:
        pts = []
        for p in msg.pos_pts:
            pts.append([round(float(p.x), 3), round(float(p.y), 3)])
        with STORE.lock:
            STORE.traj = pts[:MAX_TRAJ_POINTS]
            STORE.traj_stamp = time.time()

    def send_goal(self, x: float, y: float) -> dict:
        with STORE.lock:
            px, py = STORE.pose["x"], STORE.pose["y"]
        yaw = math.atan2(y - py, x - px)
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        self.goal_pub.publish(msg)
        dist = math.hypot(x - px, y - py)
        with STORE.lock:
            STORE.goal = {"x": x, "y": y, "dist": round(dist, 2)}
        return {"ok": True, "goal": [x, y], "distance": round(dist, 2)}

    def set_enable(self, on: bool) -> dict:
        msg = Bool()
        msg.data = bool(on)
        for _ in range(3):
            self.enable_pub.publish(msg)
            time.sleep(0.05)
        return {"ok": True, "enable": bool(on)}


HTML = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>N-01 操作台</title>
<style>
:root{--bg:#f7f7f5;--panel:#fff;--line:#e3e2de;--text:#26251f;--muted:#6b6a63;
--ok:#0f6e56;--warn:#854f0b;--bad:#a32d2d;--accent:#185fa5}
*{box-sizing:border-box}
body{margin:0;font:13px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI","PingFang SC",sans-serif;
background:var(--bg);color:var(--text)}
header{display:flex;align-items:center;gap:14px;padding:10px 16px;background:var(--panel);
border-bottom:1px solid var(--line);flex-wrap:wrap}
h1{font-size:15px;font-weight:500;margin:0}
.badge{font-size:12px;padding:2px 8px;border-radius:999px;border:1px solid var(--line);
color:var(--muted);background:#fafaf8}
.badge.on{color:var(--ok);border-color:#9fe1cb;background:#e1f5ee}
.badge.off{color:var(--bad);border-color:#f7c1c1;background:#fcebeb}
.badge.warn{color:var(--warn);border-color:#fac775;background:#faeeda}
main{display:grid;grid-template-columns:1fr 300px;gap:12px;padding:12px;height:calc(100vh - 53px)}
#wrap{position:relative;background:var(--panel);border:1px solid var(--line);border-radius:12px;overflow:hidden}
canvas{display:block;width:100%;height:100%;cursor:crosshair}
#hint{position:absolute;left:12px;bottom:10px;font-size:12px;color:var(--muted);
background:rgba(255,255,255,.85);padding:3px 8px;border-radius:6px}
aside{display:flex;flex-direction:column;gap:12px;overflow:auto}
.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:12px}
.card h2{font-size:13px;font-weight:500;margin:0 0 8px;color:var(--muted)}
.kv{display:grid;grid-template-columns:76px 1fr;gap:3px 8px;font-size:12px}
.kv b{font-weight:400;color:var(--muted)}
.kv span{font-variant-numeric:tabular-nums}
button{font:inherit;padding:8px 12px;border-radius:8px;border:1px solid var(--line);
background:#fff;cursor:pointer;width:100%}
button:hover{border-color:#b4b2a9}
button.primary{background:#e1f5ee;border-color:#9fe1cb;color:#085041}
button.danger{background:#fcebeb;border-color:#f7c1c1;color:#501313}
button:disabled{opacity:.45;cursor:not-allowed}
.row{display:flex;gap:8px;margin-top:8px}
pre{font-size:11px;white-space:pre-wrap;word-break:break-all;margin:0;color:var(--muted);
max-height:150px;overflow:auto}
</style>
</head>
<body>
<header>
  <h1>N-01 操作台</h1>
  <span id="b-ready" class="badge">READY ?</span>
  <span id="b-track" class="badge">跟踪器 ?</span>
  <span id="b-state" class="badge">-</span>
  <span id="b-conn" class="badge">连接中</span>
</header>
<main>
  <div id="wrap"><canvas id="cv"></canvas><div id="hint">在地图上单击 = 发送该点作为目标（需 READY）</div></div>
  <aside>
    <div class="card">
      <h2>机器人</h2>
      <div class="kv">
        <b>位置</b><span id="k-pos">-</span>
        <b>朝向</b><span id="k-yaw">-</span>
        <b>目标</b><span id="k-goal">-</span>
        <b>轨迹</b><span id="k-traj">-</span>
      </div>
    </div>
    <div class="card">
      <h2>速度指令（控制器）</h2>
      <div class="kv">
        <b>vx</b><span id="k-vx">-</span>
        <b>vy</b><span id="k-vy">-</span>
        <b>wz</b><span id="k-wz">-</span>
        <b>执行时间</b><span id="k-exec">-</span>
      </div>
    </div>
    <div class="card">
      <h2>执行控制</h2>
      <button id="btn-en" class="primary">启用速度输出</button>
      <div class="row"><button id="btn-stop" class="danger">急停（关闭输出）</button></div>
      <div class="row"><button id="btn-fit">适配视野</button><button id="btn-clear">清空累积地图</button></div>
      <div class="row" style="font-size:11px;color:var(--muted)">
        启用后控制器才会向 /cmd_vel 发指令。限速 vx/vy 0.3、wz 0.4。
      </div>
    </div>
    <div class="card">
      <h2>目标门控</h2>
      <pre id="p-gate">-</pre>
    </div>
  </aside>
</main>
<script>
const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
let S=null,scale=40,ox=0,oy=0,fitted=false;
const q=id=>document.getElementById(id);

function fit(){
  const w=cv.clientWidth,h=cv.clientHeight;
  cv.width=w*devicePixelRatio;cv.height=h*devicePixelRatio;
  ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);
}
function toPx(x,y){return [ (x-ox)*scale, cv.clientHeight-(y-oy)*scale ];}
function toWorld(px,py){return [ px/scale+ox, (cv.clientHeight-py)/scale+oy ];}
function autoFit(){
  const w=cv.clientWidth,h=cv.clientHeight;
  const cx = (S && S.have_pose) ? S.pose.x : 0;
  const cy = (S && S.have_pose) ? S.pose.y : 0;
  const span = 12.0;
  scale = Math.min(w,h)/span;
  ox = cx - w/2/scale;
  oy = cy - h/2/scale;
  fitted = true;
}
function draw(){
  const w=cv.clientWidth,h=cv.clientHeight;
  ctx.clearRect(0,0,w,h);
  if(!S)return;
  if(!fitted)autoFit();
  // cloud
  ctx.fillStyle='#b4b2a9';
  for(let i=0;i<S.cloud.length;i+=2){
    const [px,py]=toPx(S.cloud[i],S.cloud[i+1]);
    ctx.fillRect(px,py,1.2,1.2);
  }
  // bspline control points
  if(S.traj.length>1){
    ctx.strokeStyle='#185fa5';ctx.lineWidth=1.5;ctx.beginPath();
    S.traj.forEach((p,i)=>{const [px,py]=toPx(p[0],p[1]);i?ctx.lineTo(px,py):ctx.moveTo(px,py);});
    ctx.stroke();
    ctx.fillStyle='#185fa5';
    S.traj.forEach(p=>{const [px,py]=toPx(p[0],p[1]);ctx.fillRect(px-1.5,py-1.5,3,3);});
  }
  // goal
  if(S.goal){
    const [gx,gy]=toPx(S.goal.x,S.goal.y);
    ctx.strokeStyle='#0f6e56';ctx.lineWidth=2;
    ctx.beginPath();ctx.moveTo(gx-7,gy);ctx.lineTo(gx+7,gy);
    ctx.moveTo(gx,gy-7);ctx.lineTo(gx,gy+7);ctx.stroke();
  }
  // robot
  if(S.have_pose){
    const p=S.pose,[px,py]=toPx(p.x,p.y);
    ctx.strokeStyle='#a32d2d';ctx.fillStyle='#a32d2d';ctx.lineWidth=2;
    ctx.beginPath();
    const a=p.yaw, len=15;
    ctx.moveTo(px,py);
    ctx.lineTo(px+Math.cos(a)*len,py-Math.sin(a)*len);
    ctx.stroke();
    ctx.beginPath();ctx.arc(px,py,4,0,7);ctx.fill();
  }
}
async function poll(){
  try{
    const r=await fetch('/api/state');S=await r.json();
    q('b-conn').textContent='已连接';q('b-conn').className='badge on';
    q('b-ready').textContent=S.ready?'READY':'NOT READY';
    q('b-ready').className='badge '+(S.ready?'on':'off');
    const t=S.tracker||{};
    const en=!!t.enabled;
    q('b-track').textContent=en?'输出已启用':'输出关闭';
    q('b-track').className='badge '+(en?'warn':'');
    q('b-state').textContent=t.state||'-';
    q('b-state').className='badge '+((t.state==='TRACKING')?'on':(t.state==='ROTATING_IN_PLACE'?'warn':''));
    if(S.have_pose){q('k-pos').textContent=S.pose.x.toFixed(2)+', '+S.pose.y.toFixed(2);
      q('k-yaw').textContent=(S.pose.yaw*180/Math.PI).toFixed(1)+'°';}
    q('k-goal').textContent=S.goal?(S.goal.x.toFixed(2)+', '+S.goal.y.toFixed(2)+' ('+S.goal.dist+'m)'):'未设置';
    q('k-traj').textContent=S.traj.length?S.traj.length+' 个控制点':'无';
    const c=(t.cmd||{});
    q('k-vx').textContent=(c.vx??0).toFixed(3);
    q('k-vy').textContent=(c.vy??0).toFixed(3);
    q('k-wz').textContent=(c.wz??0).toFixed(3);
    q('k-exec').textContent=(t.exec_time??0).toFixed(2)+' / '+(t.traj_duration??0).toFixed(2)+' s';
    q('btn-en').disabled=en;
    q('p-gate').textContent=JSON.stringify(S.gate,null,1).slice(0,600);
    q('hint').textContent='点云:'+(S.cloud_source||'-')+' 共 '+(S.cloud_total||0)
      +' 格 ｜ 位姿:'+(S.pose_source||'-')+' ｜ 单击地图 = 发送该点作为目标';
    draw();
  }catch(e){
    q('b-conn').textContent='未连接';q('b-conn').className='badge off';
  }
}
cv.addEventListener('click',async ev=>{
  if(!S||!S.ready){alert('门控未 READY，目标会被拒绝');return;}
  const r=cv.getBoundingClientRect();
  const [x,y]=toWorld(ev.clientX-r.left,ev.clientY-r.top);
  const res=await fetch('/api/goal',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({x:+x.toFixed(2),y:+y.toFixed(2)})});
  const j=await res.json();
  if(j.distance!==undefined&&j.distance<1.5)alert('目标距离 '+j.distance+' m 偏近，可能被判定为障碍并改写，建议 ≥1.5 m');
});
q('btn-en').addEventListener('click',async()=>{
  if(!confirm('确认场地清空、急停在手？启用后会真的向底盘发速度指令。'))return;
  await fetch('/api/enable',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({on:true})});
});
q('btn-stop').addEventListener('click',async()=>{
  await fetch('/api/estop',{method:'POST'});
});
q('btn-fit').addEventListener('click',()=>{fitted=false;draw();});
q('btn-clear').addEventListener('click',async()=>{
  await fetch('/api/reset_accum',{method:'POST'});fitted=false;draw();
});
window.addEventListener('resize',()=>{fit();fitted=false;draw();});
fit();setInterval(poll,300);poll();
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    node: ConsoleNode = None  # type: ignore

    def log_message(self, fmt, *args):  # silence access log
        return

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code: int = 200) -> None:
        self._send(code, json.dumps(obj, ensure_ascii=False).encode(), "application/json")

    def do_GET(self):  # noqa: N802
        if self.path in ("/", "/index.html"):
            self._send(200, HTML.encode(), "text/html; charset=utf-8")
            return
        if self.path == "/api/state":
            with STORE.lock:
                use_fb_cloud = (STORE.cloud_ts == 0.0
                                or time.time() - STORE.cloud_ts > 3.0)
                if use_fb_cloud:
                    vals = list(STORE.cloud_accum.values())
                    accum_n = len(vals)
                    if accum_n > MAX_CLOUD_POINTS:
                        step = accum_n // MAX_CLOUD_POINTS + 1
                        vals = vals[::step]
                    cloud = []
                    for x, y in vals[:MAX_CLOUD_POINTS]:
                        cloud.append(x)
                        cloud.append(y)
                else:
                    cloud = list(STORE.cloud)
                    accum_n = 0
                use_fb_pose = (not STORE.have_pose
                               or (STORE.pose_fb_ts > 0.0
                                   and time.time() - STORE.pose_fb_ts < 2.0
                                   and STORE.pose["stamp"] <= 1.0))
                pose = dict(STORE.pose_fb) if use_fb_pose else dict(STORE.pose)
                snap = {
                    "pose": pose,
                    "have_pose": bool(STORE.have_pose or STORE.pose_fb_ts > 0.0),
                    "pose_source": "fastlio" if use_fb_pose else "n01",
                    "tracker": dict(STORE.tracker),
                    "gate": dict(STORE.gate),
                    "ready": STORE.ready,
                    "cloud": cloud,
                    "cloud_source": "fastlio-accum" if use_fb_cloud else "n01",
                    "cloud_total": accum_n,
                    "traj": [list(p) for p in STORE.traj],
                    "goal": dict(STORE.goal) if STORE.goal else None,
                }
            self._json(snap)
            return
        self._json({"error": "not found"}, 404)

    def do_POST(self):  # noqa: N802
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(raw or b"{}")
        except Exception:
            self._json({"error": "bad json"}, 400)
            return

        if self.path == "/api/goal":
            try:
                x = float(payload["x"])
                y = float(payload["y"])
            except Exception:
                self._json({"error": "need x,y"}, 400)
                return
            self._json(Handler.node.send_goal(x, y))
            return
        if self.path == "/api/enable":
            self._json(Handler.node.set_enable(bool(payload.get("on"))))
            return
        if self.path == "/api/estop":
            self._json(Handler.node.set_enable(False))
            return
        if self.path == "/api/reset_accum":
            self._json(Handler.node.reset_accum())
            return
        self._json({"error": "not found"}, 404)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8901)
    ap.add_argument("--host", default="0.0.0.0")
    args = ap.parse_args()

    rclpy.init()
    node = ConsoleNode()
    Handler.node = node

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"N-01 operator console: http://{args.host}:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
