# Nurse-bot
ROS based indoor guidance and delivery robot for hospitals.

[![Build Status](https://app.travis-ci.com/anubhavparas/nurse-bot.svg?branch=main)](https://app.travis-ci.com/github/anubhavparas/nurse-bot)
[![Coverage Status](https://coveralls.io/repos/github/anubhavparas/nurse-bot/badge.png?branch=main)](https://coveralls.io/github/anubhavparas/nurse-bot?branch=main)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
---

Authors: 
- [Anubhav Paras](https://github.com/anubhavparas) (Driver)
- [Sakshi Kakde](https://github.com/sakshikakde) (Navigator)
- [Siddharth Telang](https://github.com/siddharthtelang) (Design Keeper)

## Overview
In recent times the need for robots in hospitals is increasing rapidly.  The  applications  range  from  high-end  surgery  to assisting staff with non-patient-facing tasks. Most of the times at  the  hospitals,  we  wish  to  keep  minimum  human  contact. However, as the number of patients and visitors in the hospitals is  soaring,  we  need  more  people  to  assist  the  medical  staff in  their  activities  and  share  the  workload.  Thus,  a  balance is  required  between  these  two  where  we  can  have  safety(minimum people) as well as shared workload. In  such  a  scenario,  this  balance  can  be  achieved  if  robots are deployed to assist the medical faculty in their day-to-day work. The tasks of such assistant robots can be:      

- Disinfecting the hospital area
- Fetching and delivering items from central supply
- Distributing masks, PPE kits, sanitizers
- Delivering medications to patients

With this thought, we propose to ACME robotics our assistant robot, NurseBot, that would have the following capabilities:
- Service-triggered guidance : the robot would be be able to guide anyone from one part of the hospital to another,for example, patients from their room to radiology room,guiding new visitors from entry to their required location and back to exit, and guiding people in case of emergency.
- Service-triggered delivery :  Carrying  medicines,  sup-plies, test samples, PPE kits, etc. from a pickup location to target location within the hospital.


![alt text](./docs/images/TIAGoBot.png?raw=true&style=centerme "NurseBot TIAGo")


## Licence
MIT License

Copyright (c) 2021 Anubhav Paras, Sakshi Kakde, Siddharth Telang
```
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Quadchart
[Link](https://drive.google.com/file/d/1vug1ymcjOOooU_Qy6xKL-kgeXVq3C4rI/view?usp=sharing) to the quadchart.

## Proposal Report
[Link](https://drive.google.com/file/d/1iRTBGCKyLE6nexrg7UcQuHMEan0OO_aT/view?usp=sharing) to proposal report.

## Deliverables
- ROS-based  modules  (mapping,  navigation,  planner  and control)  in  C++  for  a  robot  to  guide  people  and  deliver items in an indoor hospital environment.
- Simulations of the modules in Gazebo and RViz.
- Overview  of  proposed  work  including  timeline,  risks, mitigations and UML diagrams.
- GitHub repository with README.
- Continuous integration and code coverage with TravisCIand Coveralls.
- Memory leaks check and profiling using Valgrind.
- Developer-level documentation.

## Potential Risks and Mitigation
- In case the fast dynamic obstacles are  not  detected, we will  put a constraint that the dynamic  obstacles should be stationary.
- In case the local path planner fails for dynamic obstacles,the robot will simple wait for the obstacle to disappear.

## Design
The following are the current designs for the Nursebot ROS package: 

![alt text](./UML/revised/UML_Class_Diagram_NurseBot.png?raw=true "NurseBot Class Diagram")

Activity flow diagrams can be found [here](./UML/revised/).

The following are the custom ros msg and ros action being used by publisher-subscriber and ros action client-server respectively:
- [Task.msg](msg/Task.msg)
- [NBTask.action](action/NBTask.action)

## Development process
Agile Development Process will be used in the development process with Test-Driven Development.

## Product Backlog
Refer the [document](https://docs.google.com/spreadsheets/d/1DaxNY1oEGywqltWkgmxOholQeVLlYNCZLeL-HbtvFWg/edit?usp=sharing) for product backlog.

## AIP Sprint Sheet
Refer the [sheet](https://docs.google.com/document/d/1zCVvXDaD1QKs-hEOy-ZBZ3aXe3g5vJHUmsjQiuhsIhI/edit?usp=sharing) for AIP sprint.

## Tools and Technologies used
- Ubuntu 18.04(LTS)
- C++ 14+
- CMake
- OpenCV
- Travis CI
- Coveralls

## Dependencies with licenses
- OpenCV 4.5.0 (covered under the open-source Apache 2 License)
- ROS Melodic
- RViz, Gazebo 9.0
- TIAGo++ ROS Package
- Eigen 3.4 the Mozilla Public License 2.0
- Boost 1.65 Boost software license
- GMock BSD 3-Clause "New" or "Revised" License
- GTest BSD 3-Clause "New" or "Revised" License


## Standard install via command-line

### Install Dependencies
- Make sure you have ROS Melodic installed in your computer. If not, refer to [site](http://wiki.ros.org/melodic/Installation/Ubuntu).

#### Code Coverage
```
sudo apt-get install -y -qq lcov
```
#### Installing TIAGo package:
- Create a workspace:
    ```
    mkdir ~/nursebot_ws
    cd ~/nursebot_ws
    ```
- Copy the [tiago_dual_public.rosinstall](requirements/tiago_dual_public.rosinstall) file in ` ~/nursebot_ws`. Then run the following instruction in order to clone all the required repositories within the workspace:
    ```
    rosinstall src /opt/ros/melodic tiago_dual_public.rosinstall
    ```

- Set up rosdep
    ```
    rosdep update
    ```
- Then you may run the following instruction to make sure that all dependencies referenced in the workspace are installed:
    ```
    rosdep install --from-paths src --ignore-src -y --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3 joint_impedance_trajectory_controller cartesian_impedance_controller omni_base_description omni_drive_controller"
    ```
- Build the workspace:
    ```
    sudo apt-get install ros-melodic-fiducials
    source /opt/ros/melodic/setup.bash
    catkin_make -DCATKIN_ENABLE_TESTING=0
    ```

#### Cloning nurse-bot package:
```
cd ~/nursebot_ws/src
git clone https://github.com/anubhavparas/nurse-bot.git
cd ~/nursebot_ws/
catkin_make
source devel/setup.bash
```


### Running the nurse-bot application
- First copy and/or replace the [configurations/simple_office_with_people](configurations/simple_office_with_people) world folder in `~/.pal/tiago_maps/configurations/` folder. This folder contains the pre-build maps and their configs after gmapping. __Note__: Copy the exact folder with the same name with its content.

- To run the demo, run the launch file:
    ```
    cd ~/nursebot_ws/
    source devel/setup.bash
    ```
    ```
    roslaunch nurse_bot nursebot_demo.launch
    ```
### Rosbag:
- To enable rosbag recording. By default it is disabled. 
    - This will store the rosbag recording in a file `home/<username>/.ros/recorded_topics.bag`
        ```
        roslaunch nurse_bot nursebot_demo.launch is_record_bag:=true
        ```

### Running the tests
- Build the test:
    ```
    cd ~/nursebot_ws/
    catkin_make
    catkin_make run_tests (to run all the test executables)
    (or)
    catkin_make run_tests nursebot_gtest (to run specific executable)
    ```
- tests can be run using `rostest` too, once `$catkin_make run_tests` have been run once. 
    ```
    source devel/setup.bash
    rostest nurse_bot nursebot_gtest.test
    ```

## Run cppcheck and cpplint
Run cppcheck: Results are stored in `./results/cppcheck_process.txt`, `./results/cppcheck_result.txt` 
```
sh run_cppcheck.sh
```

Run cpplint: Results are stored in `./results/cpplint_result.txt`
```
sh run_cpplint.sh
```

## Generate Doxygen Documents
To install doxygen run the following command:
```
sudo apt-get install doxygen
```

To generate the doxygen documents:
```
doxygen doxygen.config
```

The documents are generated in `./docs/doxygen` folder.

## Demonstration Links:
- [Guidance Task](https://drive.google.com/file/d/1KXg1YCIkf8f1LYsRyMdXWANSphi5n7kf/view?usp=sharing)
- [Delivery Task](https://drive.google.com/file/d/1Zz7KHegs3K-Q1lZsVuD25LJ9Ajzc5aUt/view?usp=sharing)


## Presentation
- [Video](https://drive.google.com/file/d/16NC7g0qo2npWoVmM8W78iiNOrEdmkWB1/view?usp=sharing)
- [PPT](https://docs.google.com/presentation/d/1cla1x4mnNUFcSlie9drLzIUO8PGBb18yNXufHVvToSA/edit?usp=sharing)


