# Running PISTA\_Robot with TurtleBot 3 (ROS 2 Humble)

## Prerequisites

* **Hardware**: TurtleBot 3 (Burger, Waffle, or Waffle Pi) with up‑to‑date firmware.
* **Robot SBC**: Ubuntu 22.04 + ROS 2 Humble.
* **Dev workstation**: Ubuntu 22.04 + ROS 2 Humble (can be the SBC itself if powerful enough).
* **Network**: Robot and workstation on the same Wi‑Fi / LAN and can `ping` each other.
* **Tools**: `git`, `python3`, `colcon`, and `vcs` (optional for multi‑repo).
* **Environment variables** (set on *both* machines):

  ```bash
  export ROS_DOMAIN_ID=30        # Shared DDS domain (choose any free integer 0‑101)
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ```

## Setup (Workstation)

1. **Clone the repository**

   ```bash
   git clone https://github.com/lovelms1/PISTA_Robot.git
   cd PISTA_Robot
   ```

2. **Remove stale build artefacts** (if any committed accidentally)

   ```bash
   rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
   ```

3. **Build the workspace**

   ```bash
   cd ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

4. **Export your robot model** (Burger/Waffle/Waffle Pi)

   ```bash
   export TURTLEBOT3_MODEL=burger   # choose the one you actually have
   ```

## Robot Bring‑up (on the SBC)

SSH into the robot SBC (or open a local terminal) and run:

```bash
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

Wait until all sensor topics (e.g., `/scan`, `/odom`) appear.

## Connectivity Check

Back on the workstation, verify you can see the robot’s topics:

```bash
ros2 topic list | grep cmd_vel
```

If the list is empty, re‑check Wi‑Fi, firewall, or `ROS_DOMAIN_ID` values.

## Running the Demo Node

In **any** new terminal you must *source* the workspace first:

```bash
cd ~/PISTA_Robot/ros2_ws
source install/setup.bash
```

Then launch the included square‑driver demo:

```bash
ros2 run pista_robot_api demo
```

The TurtleBot should drive a 0.25 m/s square (approx. 0.4 m side) and stop.

## Writing Your Own Script

```python
#!/usr/bin/env python3
import rclpy
from pista_robot_api.turtlebot_api import TurtleBotAPI

def main():
    rclpy.init()
    bot = TurtleBotAPI()
    bot.forward(0.15, 2.0)     # 15 cm/s for 2 s (≈30 cm)
    bot.turn_left(90)           # 90° in place
    bot.stop()
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

Save as `my_script.py` (make executable: `chmod +x my_script.py`) and run with:

```bash
python3 my_script.py
```

## Troubleshooting

| Symptom                 | Check / Fix                                                                           |
| ----------------------- | ------------------------------------------------------------------------------------- |
| Robot does **not move** | `ros2 topic echo /cmd_vel` – are messages streaming? Ensure `Twist` publisher exists. |
| **Discovery issues**    | Match `ROS_DOMAIN_ID`; disable firewall or use wired LAN.                             |
| **Package not found**   | Always `source install/setup.bash` in every terminal.                                 |
| **LiDAR data empty**    | Give ROS a few seconds to start publishing; confirm `/scan` topic exists.             |

## Further Reading

* TurtleBot 3 e‑Manual: [https://emanual.robotis.com](https://emanual.robotis.com)
* ROS 2 Tutorials (Python): [https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/index.html)
* ROS 2 launch tutorial (for writing your own launch files)

---

**End of file – happy robotics!**
