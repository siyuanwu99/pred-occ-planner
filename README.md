# pred-occ-planner

title: **Multi-Agent Trajectory Planning in Dynamic Environments with Occupancy Prediction**

submitted to IROS 2023

This work proposes a decentralized multi-agent trajectory planning framework based on an occupancy prediction map to address the challenge of autonomous navigation of multiple drones in dynamic environments. The occupancy prediction map forecasts the future occupancy status which provides a simplified approach for planning in dynamic environment without segmenting moving obstacles from the environment.

## Installation

**Tested environment**: Ubuntu 20.04 + ROS Noetic

**Prerequisites:** Ubuntu 16.04, 18.04, or 20.04 with `ros-<your_distribution>-desktop-full` installation

1. Install [OSQP](https://github.com/osqp/osqp). You can follow these [installation guidelines](https://osqp.org/docs/get_started/sources.html#build-the-binaries).

   ```shell
   git clone --recursive https://github.com/osqp/osqp
   cd osqp
   mkdir build && cd build
   cmake -G "Unix Makefiles" ..
   cmake --build .
   sudo cmake --build . --target install
   ```

2. Create a ROS workspace

   ```shell
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   ```

3. Clone this repository

   ```shell
   git clone https://github.com/edmundwsy/pred-occ-planner.git
   cd pred-occ-planner
   ```

4. Update submodules and build

   ```shell
   git submodule init & git submodule update
   cd ../..

   catkin build
   ```

## Run Simulation

You can start the simulation by following scripts:

```shell
# Go to your workspace
source devel/setup.bash
roslaunch planner sim_fkpcp_4_case_1.launch
```

Then it will start a RVIZ window with 4 drones in a dynamic environment as follows:
![sim](./images/sim.gif)

Select "2D Nav" then click the RVIZ window to send a trigger. Drones will start planning automatically.

You can try other launch file for different tasks as well.

## Licence

The source code is released under GPLv3 license.

## Contact

If you have any questions, please contact:

- Siyuan Wu {[s.wu-14@student.tudelft.nl]()}

## Acknowledgements

We thanks greatly for the authors of the following opensource projects:

- [Fast-planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) (quadrotor simulation)
- [map-generator](https://github.com/yuwei-wu/map_generator) (complex structured map generation)
- [EGO-planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2) (comparison baseline)
- [MADER](https://github.com/mit-acl/mader) (comparison baseline)
- [DSP-map](https://github.com/g-ch/DSP-map) (inspiration of occupancy prediction map)
