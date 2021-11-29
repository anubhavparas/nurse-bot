# nurse-bot
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

## Proposal Report

## Deliverables

## Potential Risks and Mitigation

## Design

## Development process
Agile Development Process will be used in the development process with Test-Driven Development.

## Product Backlog

## AIP Sprint Sheet

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