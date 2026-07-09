from datetime import datetime

from flask import Flask, jsonify, request

from robot_bridge.config import HOST, PORT, MOTION_TOPIC
from robot_bridge.ros_motion import publish_motion, stop as stop_motion


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


@app.route("/motion", methods=["POST"])
def motion():
    data = request.get_json(silent=True) or {}

    linear_x = float(data.get("linear_x", 0.0))
    angular_z = float(data.get("angular_z", 0.0))
    duration = float(data.get("duration", 0.25))

    publish_motion(
        linear_x=linear_x,
        angular_z=angular_z,
        duration=duration,
    )

    return jsonify({
        "ok": True,
        "action": "motion",
        "timestamp": now_iso(),
        "linear_x": linear_x,
        "angular_z": angular_z,
        "duration": duration,
        "message": "Motion command published to /cmd_vel.",
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
