## OpenDLV Microservice to interface with VelodyneLidar VLP32c units

This repository provides source code to interface with a VelodyneLidar VLP32c
unit for the OpenDLV software ecosystem.

[![Build Status](https://travis-ci.org/chalmers-revere/opendlv-device-lidar-vlp32c.svg?branch=master)](https://travis-ci.org/chalmers-revere/opendlv-device-lidar-vlp32c) [![License: GPLv3](https://img.shields.io/badge/license-GPL--3-blue.svg
)](https://www.gnu.org/licenses/gpl-3.0.txt)


## Table of Contents
* [Dependencies](#dependencies)
* [Usage](#usage)
* [Build from sources on the example of Ubuntu 16.04 LTS](#build-from-sources-on-the-example-of-ubuntu-1604-lts)
* [License](#license)
* [PointCloudReading data structure](#pointcloudreading-data-structure)


## Dependencies
No dependencies! You just need a C++14-compliant compiler to compile this
project as it ships the following dependencies as part of the source distribution:

* [libcluon](https://github.com/chrberger/libcluon) - [![License: GPLv3](https://img.shields.io/badge/license-GPL--3-blue.svg
)](https://www.gnu.org/licenses/gpl-3.0.txt)
* [Unit Test Framework Catch2](https://github.com/catchorg/Catch2/releases/tag/v2.1.2) - [![License: Boost Software License v1.0](https://img.shields.io/badge/License-Boost%20v1-blue.svg)](http://www.boost.org/LICENSE_1_0.txt)


## Usage
This microservice is created automatically on changes to this repository via Docker's public registry for:
* [x86_64](https://hub.docker.com/r/chalmersrevere/opendlv-device-lidar-vlp32c-amd64/tags/)
* [armhf](https://hub.docker.com/r/chalmersrevere/opendlv-device-lidar-vlp32c-armhf/tags/)
* [aarch64](https://hub.docker.com/r/chalmersrevere/opendlv-device-lidar-vlp32c-aarch64/tags/)

To run this microservice using our pre-built Docker multi-arch images to connect
to a VelodyneLidar VLP32c unit broadcasting data to `0.0.0.0:2368` and to publish
the messages according to OpenDLV Standard Message Set into session 111 in
Google Protobuf format, simply start it as follows:

```
docker run --init --rm --net=host chalmersrevere/opendlv-device-lidar-vlp32c-multi:v0.0.2 --vlp32c_ip=0.0.0.0 --vlp32c_port=2368 --cid=111 --verbose
```

## Build from sources on the example of Ubuntu 16.04 LTS
To build this software, you need cmake, C++14 or newer, and make. Having these
preconditions, just run `cmake` and `make` as follows:

```
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release ..
make && make test && make install
```


## License

* This project is released under the terms of the GNU GPLv3 License


## PointCloudReading data structure

`opendlv-device-lidar-vlp32c` receives the data from VLP-32C (Velodyne LiDAR with 32 layers)
UDP packets as input and tranforms the payload into a more compact [PointCloudReading](https://github.com/chalmers-revere/opendlv.standard-message-set/blob/bd5007e7723654563c388129a96a70b559f7fef6/opendlv.odvd#L165-L172)
representation (CPC), which is a compact form of the original 3D point cloud.
Further details of this implementation can be found in the paper
"Hang Yin and Christian Berger, Mastering data complexity for autonomous driving with adaptive point clouds for urban environments, 2017 IEEE Intelligent Vehicles Symposium, 2017 (https://www.researchgate.net/publication/318093493_Mastering_Data_Complexity_for_Autonomous_Driving_with_Adaptive_Point_Clouds_for_Urban_Environments)".

Using CPC, it is possible to encode a complete scan of VPL-16 (Velodyne LiDAR with 16 layers)
into a single UDP packet, assuming 10Hz rotation rate. In contrast, the number of points fired
by VLP-32C per scan is more than double the size of VLP-16. Assuming 10Hz rotation rate, VLP-32C
is able to fire up to 70,000 points, which cannot be all stored in one single UDP packet. Therefore,
`opendlv-device-lidar-vlp32c` stores the data of a complete scan in three separate `PointCloudReading`
messages each containing a subset of the 32 layers. The first message contains 12 layers; the second
message 11 layers, and the third contains 9 layers.

For every 32 points with the same azimuth, they are re-ordered with increasing vertical angle. The top
layer has a vertical angle of -30.67 degree. The bottom layer has a vertical angle of 10.67 degree. From
the top layer all the way down to the bottom layer, the vertical angle increment alternates between 1.33
and 1.34 degree. For instance, the vertical angle of the top layer, Layer 0, is -30.67 degree, while the
vertical angles of Layer 1 and Layer 2 are -29.33 and -28 degree, respectively.

The layers from different `PointCloudReading` interleave with each other. The first CPC includes Layer 0,
Layer 1, and every 3rd layer after Layer 1, thereby resulting in 12 layers. The second CPC includes Layer 2,
Layer 3, and every 3rd layer after Layer 3, resulting in 11 layers. The third CPC includes Layer 5 and
every 3rd layer after Layer 5, resulting in 9 layers. More specifically,

* The first `PointCloudReading` message includes layers 0, 1, 4, 7, 10, 13, 16, 19, 22, 25, 28, 31.
* The second `PointCloudReading` message includes layers 2, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30.
* The third `PointCloudReading` message includes layers 5, 8, 11, 14, 17, 20, 23, 26, 29.

Since these three CPCs include unique numbers of layers, a receiver can easily distinguish them from the
number of layers. For instance, a `PointCloudReading` with 16 layers implies a CPC from VLP-16, while a
`PointCloudReading` message with 11 layers implies the second CPC from a VLP-32C unit.

All three `PointCloudReading` messages that belong together have the same sample timestamp. In this way,
a receiver will figure out if a new `PointCloudReading` message belongs to the same scan as the previous
one belongs to a new scan. For 10Hz rotation rate, the delay between every two adjacent scans is roughly
100ms. If the difference between the sample timestamps of two `PointCloudReading` messages is close to
100ms, they must come from different scans. 
