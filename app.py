from flask import Flask, jsonify
from datetime import datetime


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
        "motion_topic": "/cmd_vel",
        "ros2_runtime": "~/ros2_ws",
        "controller": "/quadruped_controller_node",
    })


@app.route("/stop", methods=["POST"])
def stop():
    return jsonify({
        "ok": True,
        "action": "stop",
        "timestamp": now_iso(),
        "message": "Stop endpoint received. ROS2 stop publishing will be added next.",
    })


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8090)
