from robot_bridge.app import app
from robot_bridge.config import HOST, PORT


if __name__ == "__main__":
    app.run(host=HOST, port=PORT)
