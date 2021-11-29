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
- GitHub repository with README.•Continuous integration and code coverage with TravisCIand Coveralls.
- Memory leaks check and profiling using Valgrind.
- Developer-level documentation.

## Potential Risks and Mitigation
- In  case  the  fast  dynamic  obstacles  are  not  detected,  wewill  put  a  constraint  that  the  dynamic  obstacles  should be stationary.
- In case the local path planner fails for dynamic obstacles,the robot will simple wait for the obstacle to disappear.

## Design

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
- Eigen 3.4 the Mozilla Public License 2.0
- Boost 1.65 Boost software license
- GMock BSD 3-Clause "New" or "Revised" License
- GTest BSD 3-Clause "New" or "Revised" License


## Standard install via command-line


### Install Dependencies

### Running the nurse-bot application

This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.

## Building for code coverage

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
