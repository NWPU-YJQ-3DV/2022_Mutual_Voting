# Mutual-Voting-for-Ranking-3D-Correspondences
Source code of PAMI paper

## Introduction  

![](figures/pipeline.png#pic_center=50%x)

## Repository layout  
The repository contains a set of subfolders:  
* [`Linux`](https://github.com/NWPU-YJQ-3DV/2022_Mutual_Voting/tree/main/Linux) - source code for Linux platform.  
* [`Windows`](https://github.com/NWPU-YJQ-3DV/2022_Mutual_Voting/tree/main/Windows) - source code for Windows platform.
* [`demo`](https://github.com/NWPU-YJQ-3DV/2022_Mutual_Voting/tree/main/demo) - test point clouds.

## Build
MV depends on [PCL](https://github.com/PointCloudLibrary/pcl/tags) (`>= 1.10.1`). Please install the library first.

To build MV, you need [CMake](https://cmake.org/download/) (`>= 3.23`) and, of course, a compiler that supports `>= C++11`. The code in this repository has been tested on Windows (MSVC `=2022` `x64`), and Linux (GCC `=10.4.0`). Machines nowadays typically provide higher [support](https://en.cppreference.com/w/cpp/compiler_support), so you should be able to build MAC on almost all platforms.

### Windows version  
Please refer to [Compiling on Windows](https://github.com/NWPU-YJQ-3DV/2022_Mutual_Voting/blob/main/Windows/README.md) for details.

### Linux version
Please refer to [Compiling on Linux](https://github.com/NWPU-YJQ-3DV/2022_Mutual_Voting/blob/main/Linux/README.md) for details.

## Usage:
