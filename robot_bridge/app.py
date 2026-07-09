from datetime import datetime

from flask import Flask, jsonify

from robot_bridge.config import HOST, PORT, MOTION_TOPIC
from robot_bridge.ros_motion import stop as stop_motion


app = Flask(__name__)


def now_iso():
    return datetime.utcnow().isoformat() + "Z"


@app.route("/status", methods=["GET"])
def status():
    return jsonify({
        "ok": True,
        "service": "mini_pupper_robot_bridge",
        "timestamp": now_iso(),
        "robot": "mini_pupper_2",
        "status": "READY",
        "motion_topic": MOTION_TOPIC,
        "ros2_runtime": "~/ros2_ws",
        "controller": "/quadruped_controller_node",
    })


@app.route("/stop", methods=["POST"])
def stop():
    stop_motion()
    return jsonify({
        "ok": True,
        "action": "stop",
        "timestamp": now_iso(),
        "message": "Zero velocity published to /cmd_vel.",
    })


if __name__ == "__main__":
    app.run(host=HOST, port=PORT)
