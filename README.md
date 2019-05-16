# EasyLabel

EasyLabel: RGBD-dataset creation tool

EasyLabel provides tools to semi-automatically annotate recorded sequences of organized pointcloud files. 
It is an open source project from the [Vision4Robotics (V4R)](https://v4r.acin.tuwien.ac.at) group at the ACIN institute, TU Vienna.
An example result of using EasyLabel is the Object Clutter Indoor Dataset [OCID](https://www.acin.tuwien.ac.at/en/vision-for-robotics/software-tools/object-clutter-indoor-dataset/).

# Dependencies:

EasyLabel is created to work with RGB-D data and uses [OpenCV](http://opencv.org/)  and [PCL](http://pointclouds.org/) to process data. Here is a list of **required** components:

| Name | Version | Licence |
| ------------- |:-------------:| -----:|
| [OpenCV](http://opencv.org/)  | 2.4+  or 3.x | 3-clause BSD |
| [Boost](http://www.boost.org/)  | 1.48+ | Boost Software License |
| [PCL](http://pointclouds.org/)  | 1.8.x | BSD |
| [CMake](https://cmake.org)  | 3.5.1+ | 3-clause BSD |

## Install from Source

```
cd ~/somewhere
git clone 'https://github.com/markus-suchi/easylabel.git'
cd easylabel
mkdir build
cd build
cmake ..
make
```

## Citation
If you find EasyLabel helpful please cite:
```
ICRA2019 publication text in near future
```

If you have any questions please contact:
- Markus Suchi – email: suchi@acin.tuwien.ac.at
- Tim Patten – email: patten@acin.tuwien.ac.at

## Tutorial
```
I am currently working on a tutorial and will provide information here.
```
