# Mutual-Voting-for-Ranking-3D-Correspondences
Source code of [PAMI 2023 paper](https://ieeexplore.ieee.org/abstract/document/10105460) 

## Introduction  

![pipeline](figures/pipeline.png#pic_center)

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

## Data organization
Please refer to [Demo](https://github.com/NWPU-YJQ-3DV/2022_Mutual_Voting/blob/main/demo/README.md) for details.

## Usage:
* `--help` list all usages.
### Required args:
* `--input_path` input data path.
* `--output_path` output data path.
* `--dataset_name` dataset name. 
* `--RANSAC_iters` number of ransac iterations.
### Optional args:
* `--no_logs` forbid generation of log files.
* `--corr_index_mode` input correspondence file contains indices instead of coordinates.

## Citation
If you find this code useful for your work or use it in your project, please consider citing:

```shell
@article{yang2023mutual,
  title={Mutual Voting for Ranking 3D Correspondences},
  author={Yang, Jiaqi and Zhang, Xiyu and Fan, Shichao and Ren, Chunlin and Zhang, Yanning},
  journal={IEEE Transactions on Pattern Analysis and Machine Intelligence},
  year={2023},
  publisher={IEEE}
}
```
