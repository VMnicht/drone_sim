#!/usr/bin/env python3

import json
import math
import queue
import threading
import time
from pathlib import Path
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool


HTML = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ROS2 Drone Ground Station</title>
<style>
body{font:15px system-ui;margin:0;background:#0d1423;color:#e8eefc}header{padding:18px 24px;background:#16213a}
main{display:grid;grid-template-columns:1fr 1fr;gap:16px;padding:18px}.card{background:#17243d;padding:16px;border-radius:10px}
table{width:100%;border-collapse:collapse}th,td{padding:8px;border-bottom:1px solid #31405e;text-align:right}th:first-child,td:first-child{text-align:left}
input,select,button{padding:8px;margin:4px;background:#0e1930;color:#fff;border:1px solid #506184;border-radius:5px}button{cursor:pointer;background:#2864c7}
canvas{width:100%;height:220px;background:#0b1220}#message{min-height:1.4em;color:#7ee0a1}.safe{color:#7ee0a1}.stale{color:#ffbb66}
@media(max-width:850px){main{grid-template-columns:1fr}}
</style></head><body><header><h2>ROS2 无人机地面站</h2><div id="message">连接中…</div><div id="system"></div></header><main>
<section class="card"><h3>实时状态</h3><table><thead><tr><th>ID</th><th>x</th><th>y</th><th>z</th><th>速度</th><th>状态</th></tr></thead><tbody id="rows"></tbody></table></section>
<section class="card"><h3>任务、扰动与故障控制</h3><label>无人机 <select id="drone"></select></label><br>
<label>x <input id="x" type="number" step="0.1" value="1"></label><label>y <input id="y" type="number" step="0.1" value="0"></label><br>
<label>z <input id="z" type="number" step="0.1" value="1.5"></label><label>yaw <input id="yaw" type="number" step="0.1" value="0"></label><br>
<button onclick="goal()">发送目标</button><button onclick="resetDrone()">重置</button>
<button onclick="disturbance(true)">启用 YAML 扰动</button><button onclick="disturbance(false)">关闭扰动</button>
<button onclick="fault(true)">启用 YAML 故障</button><button onclick="fault(false)">清除故障</button></section>
<section class="card" style="grid-column:1/-1"><h3>高度历史</h3><canvas id="plot" width="1000" height="220"></canvas></section>
<section class="card" style="grid-column:1/-1"><h3>最近实验结果</h3><table><thead><tr><th>场景</th><th>结果</th><th>最终误差/m</th><th>最小间距/m</th></tr></thead><tbody id="results"></tbody></table></section>
</main><script>
const history={}, colors=['#61a5ff','#76e6a3','#ffb55e','#d68cff'];
async function post(path,data){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});const j=await r.json();message.textContent=j.message||JSON.stringify(j);if(!r.ok)throw Error(message.textContent)}
function selected(){return drone.value} function number(id){return Number(document.getElementById(id).value)}
function goal(){post('/api/goal',{drone_id:selected(),x:number('x'),y:number('y'),z:number('z'),yaw:number('yaw')})}
function resetDrone(){post('/api/reset',{drone_id:selected()})}
function fault(enabled){post('/api/fault',{enabled})}
function disturbance(enabled){post('/api/disturbance',{enabled})}
function draw(ids){const c=plot,ctx=c.getContext('2d');ctx.clearRect(0,0,c.width,c.height);ctx.strokeStyle='#40506d';for(let i=0;i<5;i++){const y=i*c.height/4;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(c.width,y);ctx.stroke()}
ids.forEach((id,k)=>{const h=history[id]||[];ctx.strokeStyle=colors[k%colors.length];ctx.beginPath();h.forEach((v,i)=>{const x=i*c.width/199,y=c.height-Math.max(0,Math.min(3,v))*c.height/3;i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke()})}
async function refresh(){try{const r=await fetch('/api/status'),s=await r.json(),ids=s.drone_ids;if(drone.options.length!==ids.length){drone.innerHTML=ids.map(id=>`<option>${id}</option>`).join('')}
rows.innerHTML=ids.map(id=>{const d=s.drones[id];if(!d)return `<tr><td>${id}</td><td colspan=4>等待数据</td><td class=stale>离线</td></tr>`;history[id]=(history[id]||[]).concat([d.position[2]]).slice(-200);return `<tr><td>${id}</td><td>${d.position[0].toFixed(2)}</td><td>${d.position[1].toFixed(2)}</td><td>${d.position[2].toFixed(2)}</td><td>${d.speed.toFixed(2)}</td><td class=${d.stale?'stale':'safe'}>${d.stale?'过期':'正常'}</td></tr>`}).join('');draw(ids);system.textContent=`规划：${s.planner_status}　故障：${s.fault_status}`;message.textContent=`更新 ${new Date().toLocaleTimeString()}`}
catch(e){message.textContent='连接失败: '+e}setTimeout(refresh,500)}refresh();
async function refreshResults(){try{const r=await fetch('/api/results'),items=await r.json();results.innerHTML=items.map(v=>`<tr><td>${v.scenario}</td><td class=safe>已记录</td><td>${v.final_position_error_m==null?'—':Number(v.final_position_error_m).toFixed(3)}</td><td>${v.minimum_obstacle_clearance_m==null?'—':Number(v.minimum_obstacle_clearance_m).toFixed(3)}</td></tr>`).join('')||'<tr><td colspan=4>尚无实验结果</td></tr>'}catch(e){results.innerHTML='<tr><td colspan=4>结果读取失败</td></tr>'}setTimeout(refreshResults,3000)}refreshResults();
</script></body></html>"""


class GroundStationHandler(BaseHTTPRequestHandler):
    server_version = "DroneGroundStation/1.0"

    def log_message(self, format_string, *args):
        self.server.node.get_logger().debug(format_string % args)

    def send_json(self, value, status=HTTPStatus.OK):
        payload = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            payload = HTML.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        if path == "/api/status":
            self.send_json(self.server.node.status_snapshot())
            return
        if path == "/api/results":
            self.send_json(self.server.node.results_snapshot())
            return
        self.send_json({"message": "not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > self.server.node.maximum_request_bytes:
            self.send_json({"message": "invalid request size"}, HTTPStatus.BAD_REQUEST)
            return
        try:
            data = json.loads(self.rfile.read(length))
            path = urlparse(self.path).path
            if path == "/api/goal":
                self.server.node.validate_goal(data)
                self.server.node.commands.put(("goal", data))
                self.send_json({"message": "goal queued"}, HTTPStatus.ACCEPTED)
                return
            if path == "/api/reset":
                self.server.node.validate_drone_id(data.get("drone_id"))
                self.server.node.commands.put(("reset", data))
                self.send_json({"message": "reset queued"}, HTTPStatus.ACCEPTED)
                return
            if path == "/api/fault":
                if not isinstance(data.get("enabled"), bool):
                    raise ValueError("enabled must be boolean")
                self.server.node.commands.put(("fault", data))
                self.send_json({"message": "fault command queued"}, HTTPStatus.ACCEPTED)
                return
            if path == "/api/disturbance":
                if not isinstance(data.get("enabled"), bool):
                    raise ValueError("enabled must be boolean")
                self.server.node.commands.put(("disturbance", data))
                self.send_json(
                    {"message": "disturbance command queued"}, HTTPStatus.ACCEPTED
                )
                return
            self.send_json({"message": "not found"}, HTTPStatus.NOT_FOUND)
        except (ValueError, TypeError, json.JSONDecodeError) as exception:
            self.send_json({"message": str(exception)}, HTTPStatus.BAD_REQUEST)


class WebGroundStationNode(Node):
    def __init__(self):
        super().__init__("web_ground_station_node")
        self.declare_parameter("ground_station_bind_address", "127.0.0.1")
        self.declare_parameter("ground_station_port", 8080)
        self.declare_parameter("ground_station_command_frequency", 20.0)
        self.declare_parameter("ground_station_stale_timeout", 1.0)
        self.declare_parameter("ground_station_maximum_request_bytes", 4096)
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("ground_station_drone_ids", ["drone"])
        self.declare_parameter("ground_station_odometry_topics", ["/drone/odom"])
        self.declare_parameter("ground_station_goal_topics", ["/drone/goal"])
        self.declare_parameter("ground_station_reset_services", ["/drone/reset"])
        self.declare_parameter("fault_enable_service", "/fault/enable")
        self.declare_parameter(
            "disturbance_enable_service", "/drone/disturbance/enable"
        )
        self.declare_parameter("fault_status_topic", "/fault/status")
        self.declare_parameter("planner_status_topic", "/drone/planner_status")
        self.declare_parameter("ground_station_qos_depth", 20)
        self.declare_parameter("ground_station_goal_minimum", [-20.0, -20.0, 0.0])
        self.declare_parameter("ground_station_goal_maximum", [20.0, 20.0, 10.0])
        self.declare_parameter(
            "ground_station_results_root", "artifacts/experiments"
        )
        self.declare_parameter("ground_station_maximum_results", 20)

        self.bind_address = self.string_parameter("ground_station_bind_address")
        self.port = int(self.get_parameter("ground_station_port").value)
        self.command_frequency = self.positive_parameter(
            "ground_station_command_frequency"
        )
        self.stale_timeout = self.positive_parameter("ground_station_stale_timeout")
        self.maximum_request_bytes = self.positive_integer_parameter(
            "ground_station_maximum_request_bytes"
        )
        self.world_frame = self.string_parameter("world_frame")
        self.drone_ids = self.string_list("ground_station_drone_ids")
        odometry_topics = self.string_list("ground_station_odometry_topics")
        goal_topics = self.string_list("ground_station_goal_topics")
        reset_services = self.string_list("ground_station_reset_services")
        fault_enable_service = self.string_parameter("fault_enable_service")
        disturbance_enable_service = self.string_parameter(
            "disturbance_enable_service"
        )
        fault_status_topic = self.string_parameter("fault_status_topic")
        planner_status_topic = self.string_parameter("planner_status_topic")
        self.qos_depth = self.positive_integer_parameter("ground_station_qos_depth")
        self.goal_minimum = self.vector_parameter("ground_station_goal_minimum")
        self.goal_maximum = self.vector_parameter("ground_station_goal_maximum")
        self.results_root = Path(
            self.string_parameter("ground_station_results_root")
        ).expanduser().resolve()
        self.maximum_results = self.positive_integer_parameter(
            "ground_station_maximum_results"
        )
        if not (1 <= self.port <= 65535) or not self.bind_address or not self.world_frame:
            raise ValueError("ground station bind address, port or frame is invalid")
        if any(lo >= hi for lo, hi in zip(self.goal_minimum, self.goal_maximum)):
            raise ValueError("ground station goal bounds are invalid")
        if not (
            len(self.drone_ids)
            == len(odometry_topics)
            == len(goal_topics)
            == len(reset_services)
        ):
            raise ValueError("ground station drone/topic/service arrays must match")
        if len(set(self.drone_ids)) != len(self.drone_ids):
            raise ValueError("ground station drone IDs must be unique")

        self.lock = threading.Lock()
        self.telemetry = {}
        self.commands = queue.Queue()
        self.goal_publishers = {}
        self.reset_clients = {}
        self.fault_client = self.create_client(SetBool, fault_enable_service)
        self.disturbance_client = self.create_client(
            SetBool, disturbance_enable_service
        )
        self.fault_status = "等待状态"
        self.planner_status = "等待状态"
        self.fault_status_subscription = self.create_subscription(
            String, fault_status_topic,
            lambda message: setattr(self, "fault_status", message.data), self.qos_depth
        )
        self.planner_status_subscription = self.create_subscription(
            String, planner_status_topic,
            lambda message: setattr(self, "planner_status", message.data), self.qos_depth
        )
        self.odometry_subscriptions = []
        for drone_id, odometry_topic, goal_topic, reset_service in zip(
            self.drone_ids, odometry_topics, goal_topics, reset_services
        ):
            self.goal_publishers[drone_id] = self.create_publisher(
                PoseStamped, goal_topic, self.qos_depth
            )
            self.reset_clients[drone_id] = self.create_client(Empty, reset_service)
            self.odometry_subscriptions.append(
                self.create_subscription(
                    Odometry,
                    odometry_topic,
                    lambda message, identifier=drone_id: self.odometry_callback(
                        identifier, message
                    ),
                    self.qos_depth,
                )
            )
        self.command_timer = self.create_timer(
            1.0 / self.command_frequency, self.process_commands
        )
        self.http_server = ThreadingHTTPServer(
            (self.bind_address, self.port), GroundStationHandler
        )
        self.http_server.node = self
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            name="drone-ground-station-http",
            daemon=True,
        )
        self.http_thread.start()
        self.get_logger().info(
            f"Web ground station listening on http://{self.bind_address}:{self.port}"
        )

    def string_parameter(self, name):
        return str(self.get_parameter(name).value)

    def string_list(self, name):
        values = [str(value) for value in self.get_parameter(name).value]
        if not values or any(not value for value in values):
            raise ValueError(f"{name} must contain non-empty strings")
        return values

    def vector_parameter(self, name):
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise ValueError(f"{name} must contain three finite values")
        return values

    def positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def positive_integer_parameter(self, name):
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError(f"{name} must be positive")
        return value

    def validate_drone_id(self, drone_id):
        if drone_id not in self.goal_publishers:
            raise ValueError("unknown drone_id")

    def validate_goal(self, data):
        self.validate_drone_id(data.get("drone_id"))
        values = [float(data[name]) for name in ("x", "y", "z", "yaw")]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("goal values must be finite")
        if any(
            value < low or value > high
            for value, low, high in zip(values[:3], self.goal_minimum, self.goal_maximum)
        ):
            raise ValueError("goal lies outside configured bounds")

    def odometry_callback(self, drone_id, message):
        linear = message.twist.twist.linear
        value = {
            "position": [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ],
            "speed": math.sqrt(linear.x**2 + linear.y**2 + linear.z**2),
            "received": time.monotonic(),
        }
        with self.lock:
            self.telemetry[drone_id] = value

    def status_snapshot(self):
        now = time.monotonic()
        with self.lock:
            drones = {key: dict(value) for key, value in self.telemetry.items()}
        for value in drones.values():
            value["stale"] = now - value.pop("received") > self.stale_timeout
        return {
            "drone_ids": self.drone_ids,
            "drones": drones,
            "fault_status": self.fault_status,
            "planner_status": self.planner_status,
        }

    def results_snapshot(self):
        results = []
        if not self.results_root.is_dir():
            return results
        summary_paths = sorted(
            self.results_root.glob("*/summary.json"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        for path in summary_paths[: self.maximum_results]:
            try:
                summary = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError, TypeError):
                continue
            results.append(
                {
                    "scenario": str(summary.get("scenario", path.parent.name)),
                    "final_position_error_m": summary.get("final_position_error_m"),
                    "minimum_obstacle_clearance_m": summary.get(
                        "minimum_obstacle_clearance_m"
                    ),
                }
            )
        return results

    def process_commands(self):
        while True:
            try:
                command, data = self.commands.get_nowait()
            except queue.Empty:
                return
            if command == "goal":
                drone_id = data["drone_id"]
                message = PoseStamped()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = self.world_frame
                message.pose.position.x = float(data["x"])
                message.pose.position.y = float(data["y"])
                message.pose.position.z = float(data["z"])
                yaw = float(data["yaw"])
                message.pose.orientation = Quaternion(
                    w=math.cos(0.5 * yaw), z=math.sin(0.5 * yaw)
                )
                self.goal_publishers[drone_id].publish(message)
            elif command == "reset":
                drone_id = data["drone_id"]
                client = self.reset_clients[drone_id]
                if client.service_is_ready():
                    client.call_async(Empty.Request())
                else:
                    self.get_logger().warning(
                        f"Reset service for {drone_id} is not ready"
                    )
            elif command == "fault":
                if self.fault_client.service_is_ready():
                    self.fault_client.call_async(SetBool.Request(data=bool(data["enabled"])))
                else:
                    self.get_logger().warning("Fault enable service is not ready")
            elif command == "disturbance":
                if self.disturbance_client.service_is_ready():
                    self.disturbance_client.call_async(
                        SetBool.Request(data=bool(data["enabled"]))
                    )
                else:
                    self.get_logger().warning(
                        "Disturbance enable service is not ready"
                    )

    def destroy_node(self):
        self.http_server.shutdown()
        self.http_server.server_close()
        self.http_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebGroundStationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
