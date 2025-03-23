# Sailboat Simulator

The Following repository provides the simulator of Sailboat RS750 in the GazeboSim simulator.<br>
The simulator has been built based on the packages: [asv_sim](https://github.com/srmainwaring/asv_sim), [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim) and [rs750](https://github.com/srmainwaring/rs750).<br>
The repository provides a consistent Docker environment with GazeoSim, Arduopilot (SITL). <br>
SITL allows you to simulate the vehicle hardware and firmware ([ArduSub](https://www.ardusub.com/) ) on your host directly.<br>

![image](https://github.com/user-attachments/assets/2ce6fac6-3691-49eb-a9a2-d0b390212459)

---
<br>
Expected results, <br>

![sailboat](https://github.com/user-attachments/assets/bfe3bdff-bdb4-4f41-a160-33d66d336404)

---
## Prerequisites

- Download and Install [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) (optional).
- Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to support Docker to access GPU (required).
- Repository has been tested on: Ubuntu 22.04, Ubuntu 24.04, ArchLinux (Kernel 6.8).


## Build

```bash
git clone https://github.com/markusbuchholz/gazebosim_bluerov2_ardupilot_sitl.git

cd gazebosim_bluerov2_ardupilot_sitl/bluerov2_ardupilot_SITL/docker

sudo ./build.sh

```

## Build in Docker

Adjust in ```run.sh```.

```bash
local_gz_ws="/home/markus/bluerov2_ardupilot_SITL/gz_ws"
local_SITL_Models="/home/markus/bluerov2_ardupilot_SITL/SITL_Models"
```

```bash
sudo ./run.sh

colcon build

source install/setup.bash

cd ../gz_ws

colcon build --symlink-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DCMAKE_CXX_STANDARD=17

source install/setup.bash

source gazebo_exports.sh
 
```
---

## ROS 2

Available topics,
```bash
/fore_sail_joint/cmd_pos
/main_sail_joint/cmd_pos
/rudder_joint/cmd_pos
/world/waves/model/rs750/link/base_link/sensor/anemometer/anemometer
/world/waves/model/rs750/link/base_link/sensor/imu_sensor/imu
/world/waves/model/rs750/link/base_link/sensor/navsat_sensor/navsat
```

Navigate RS750 using following commands (from GazeboSim launch ```Wave Control plugin```),

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



