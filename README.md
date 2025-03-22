# Sailboat Simulator

![image](https://github.com/user-attachments/assets/2ce6fac6-3691-49eb-a9a2-d0b390212459)

---
<br>
Expected ressults, <br>

![sailboat](https://github.com/user-attachments/assets/bfe3bdff-bdb4-4f41-a160-33d66d336404)


---
## ROS 2

```bash
/fore_sail_joint/cmd_pos
/main_sail_joint/cmd_pos
/rudder_joint/cmd_pos
/world/waves/model/rs750/link/base_link/sensor/anemometer/anemometer
/world/waves/model/rs750/link/base_link/sensor/imu_sensor/imu
/world/waves/model/rs750/link/base_link/sensor/navsat_sensor/navsat
```


```bash
ros2 topic pub /main_sail_joint/cmd_pos std_msgs/msg/Float64 "{data: -5.0}"
ros2 topic pub /fore_sail_joint/cmd_pos std_msgs/msg/Float64 "{data: 2.0}"
ros2 topic pub /rudder_joint/cmd_pos std_msgs/msg/Float64 "{data: 3.0}"
```

## GazeboSim

```bash
gz topic -t /main_sail_joint/cmd_pos -m gz.msgs.Double --pub "data: 2.0"
gz topic -t /rudder_joint/cmd_pos -m gz.msgs.Double --pub "data: -2.0"
gz topic -t /fore_sail_joint/cmd_pos -m gz.msgs.Double --pub "data: 2.0"
```
