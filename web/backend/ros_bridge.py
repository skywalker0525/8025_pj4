import asyncio
import os
import json
import threading
from typing import Set

from cv_bridge import CvBridge
import cv2
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, String


class DashboardBridge(Node):
    def __init__(self) -> None:
        super().__init__('dashboard_bridge')
        self.bridge = CvBridge()
        self.latest_telemetry = {}
        self.latest_jpeg = None
        self.command_pub = self.create_publisher(String, '/mission/command', 10)
        self.manual_cmd_pub = self.create_publisher(Twist, '/mission/manual_cmd_vel', 10)
        self.gimbal_pub = self.create_publisher(Float64MultiArray, '/mission/manual_gimbal', 10)
        self.create_subscription(String, '/mission/telemetry', self.on_telemetry, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_camera_image, 10)

    def on_telemetry(self, msg: String) -> None:
        try:
            self.latest_telemetry = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_telemetry = {'raw': msg.data}

    def on_camera_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                self.latest_jpeg = encoded.tobytes()
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert camera frame for web stream: {exc}')

    def publish_command(self, command: str) -> None:
        self.command_pub.publish(String(data=command))

    def publish_manual_velocity(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.manual_cmd_pub.publish(msg)

    def publish_gimbal(self, pan: float, tilt: float) -> None:
        self.gimbal_pub.publish(Float64MultiArray(data=[float(pan), float(tilt)]))


app = FastAPI(title='Construction Robot Digital Twin Bridge')
app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_credentials=True,
    allow_methods=['*'],
    allow_headers=['*'],
)

STATIC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'static'))
app.mount('/static', StaticFiles(directory=STATIC_DIR), name='static')

bridge = None
clients: Set[WebSocket] = set()


def spin_ros() -> None:
    global bridge
    rclpy.init()
    bridge = DashboardBridge()
    rclpy.spin(bridge)


@app.on_event('startup')
async def startup() -> None:
    thread = threading.Thread(target=spin_ros, daemon=True)
    thread.start()


@app.on_event('shutdown')
async def shutdown() -> None:
    if bridge is not None:
        bridge.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


@app.get('/api/telemetry')
async def get_telemetry():
    return bridge.latest_telemetry if bridge is not None else {}


@app.get('/')
async def dashboard():
    return FileResponse(os.path.join(STATIC_DIR, 'index.html'))


@app.post('/api/command/{command}')
async def post_command(command: str):
    if bridge is not None:
        bridge.publish_command(command.upper())
    return {'ok': True, 'command': command.upper()}


@app.post('/api/manual/cmd_vel')
async def post_manual_cmd_vel(payload: dict):
    if bridge is not None:
        bridge.publish_manual_velocity(payload.get('linear', 0.0), payload.get('angular', 0.0))
    return {'ok': True}


@app.post('/api/manual/gimbal')
async def post_manual_gimbal(payload: dict):
    if bridge is not None:
        bridge.publish_gimbal(payload.get('pan', 0.0), payload.get('tilt', 0.0))
    return {'ok': True}


async def camera_frames():
    while True:
        frame = bridge.latest_jpeg if bridge is not None else None
        if frame is not None:
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
            )
        await asyncio.sleep(0.05)


@app.get('/camera/stream')
async def camera_stream():
    return StreamingResponse(
        camera_frames(),
        media_type='multipart/x-mixed-replace; boundary=frame',
    )


@app.websocket('/ws')
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.add(websocket)
    try:
        while True:
            telemetry = bridge.latest_telemetry if bridge is not None else {}
            await websocket.send_json(telemetry)
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        clients.discard(websocket)
