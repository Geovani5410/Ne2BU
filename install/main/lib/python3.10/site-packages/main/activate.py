import subprocess
import time

def main():
    comandos = [
        "source /opt/ros/humble/setup.bash && ros2 run main wake_word",
        "source /opt/ros/humble/setup.bash && ros2 run main voz_node",
        "source /opt/ros/humble/setup.bash && ros2 run main llm_node",
        "source /opt/ros/humble/setup.bash && ros2 run main tts_node",
        "source /opt/ros/humble/setup.bash && ros2 run main history_node",
        #"source /opt/ros/humble/setup.bash && ros2 run main dashboard",
        "source  /opt/ros/humble/setup.bash && ros2 run main bridge_node"
    ]

    for cmd in comandos:
        subprocess.run(["gnome-terminal", "--", "bash", "-c", f"{cmd}; exec bash"])