# Decentralized Multi-Agent Trajectory Planning in Dynamic Environments with Spatiotemporal Occupancy Grid Maps

This paper proposes a decentralized trajectory planning framework for the collision avoidance problem of mul- tiple micro aerial vehicles (MAVs) in environments with static and dynamic obstacles. The framework utilizes spatiotemporal occupancy grid maps (SOGM), which forecast the occupancy status of neighboring space in the near future, as the environ- ment representation. Based on this representation, we extend the kinodynamic A\* and the corridor-constrained trajectory optimization algorithms to efficiently tackle static and dynamic obstacles with arbitrary shapes. Collision avoidance between communicating robots is integrated by sharing planned tra- jectories and projecting them onto the SOGM. The simulation results show that our method achieves competitive performance against state-of-the-art methods in dynamic environments with different numbers and shapes of obstacles. Finally, the proposed method is validated in real experiments.

https://user-images.githubusercontent.com/44539400/223415381-6c7bb7a4-94b9-4e77-b92f-c02b3ea6eb01.mp4

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
   git clone https://github.com/siyuanwu99/pred-occ-planner.git
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
roslaunch plan_manager sim_fkpcp_4_case_4.launch
```

You can try a much more complex dynamic map as follows:

```
roslaunch plan_manager sim_new_4_case_3.launch
```

Then it will start a RVIZ window with 4 drones in a dynamic environment as follows:
![sim](./images/sim.gif)

Select "2D Nav" then click the RVIZ window to send a trigger. Drones will start planning automatically.

You can try other launch file for different tasks as well.

## Licence

The source code is released under GPLv3 license.

## Contact

If you find this work helpful, I would greatly appreciate it if you could kindly cite this paper: [Decentralized Multi-Agent Trajectory Planning in Dynamic Environments with Spatiotemporal Occupancy Grid Maps](https://autonomousrobots.nl/assets/files/publications/24-wu-icra.pdf)

```
@article{wu2024decentralized,
  title={Decentralized Multi-Agent Trajectory Planning in Dynamic Environments with Spatiotemporal Occupancy Grid Maps},
  author={Wu, Siyuan and Chen, Gang and Shi, Moji and Alonso-Mora, Javier},
  journal={arXiv preprint arXiv:2404.15602},
  year={2024}
}
```

If you have any questions, please contact:

- Siyuan Wu {[siyuanwu99@gmail.com]()}

## Acknowledgements

We thanks greatly for the authors of the following opensource projects:

- [Fast-planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) (quadrotor simulation)
- [map-generator](https://github.com/yuwei-wu/map_generator) (complex structured map generation)
- [EGO-planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2) (comparison baseline)
- [MADER](https://github.com/mit-acl/mader) (comparison baseline)
- [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) (efficient corridor generation and trajectory optimization)
- [DSP-map](https://github.com/g-ch/DSP-map) (inspiration of occupancy prediction map)
