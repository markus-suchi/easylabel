# EasyLabel

EasyLabel: RGBD-dataset creation tool

EasyLabel provides tools to semi-automatically annotate recorded sequences of organized point-cloud files. 
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
@inproceedings{DBLP:conf/icra/SuchiPFV19,
  author    = {Markus Suchi and
               Timothy Patten and
               David Fischinger and
               Markus Vincze},
  title     = {EasyLabel: {A} Semi-Automatic Pixel-wise Object Annotation Tool for
               Creating Robotic {RGB-D} Datasets},
  booktitle = {International Conference on Robotics and Automation, {ICRA} 2019,
               Montreal, QC, Canada, May 20-24, 2019},
  pages     = {6678--6684},
  year      = {2019},
  crossref  = {DBLP:conf/icra/2019},
  url       = {https://doi.org/10.1109/ICRA.2019.8793917},
  doi       = {10.1109/ICRA.2019.8793917},
  timestamp = {Tue, 13 Aug 2019 20:25:20 +0200},
  biburl    = {https://dblp.org/rec/bib/conf/icra/SuchiPFV19},
  bibsource = {dblp computer science bibliography, https://dblp.org}
}
```
```
@proceedings{DBLP:conf/icra/2019,
  title     = {International Conference on Robotics and Automation, {ICRA} 2019,
               Montreal, QC, Canada, May 20-24, 2019},
  publisher = {{IEEE}},
  year      = {2019},
  url       = {http://ieeexplore.ieee.org/xpl/mostRecentIssue.jsp?punumber=8780387},
  isbn      = {978-1-5386-6027-0},
  timestamp = {Tue, 13 Aug 2019 20:23:21 +0200},
  biburl    = {https://dblp.org/rec/bib/conf/icra/2019},
  bibsource = {dblp computer science bibliography, https://dblp.org}
}
```

If you have any questions please contact:
- Markus Suchi – email: suchi@acin.tuwien.ac.at
- Tim Patten – email: patten@acin.tuwien.ac.at

## Usage
This is a short description of how to use easylabel.
To try out the annotation tool you can obtain sample data by running the following script.
Change to easylabel root folder.
```
./scripts/get_tutorial_data.sh

Creates the following data structure:
data_tutorial  
│
└───extract
│   │    
│   │
│   └─── 000
│        │   cloud_000000.pcd
│        │      │   ...
│        │   cloud_000001.pcd
│        │   ...
│  ...
│   │
│   └─── 003
│        │   ...
│   
└───accu [folder for accumulated cloud data]
│   
└───label [folder for labeled cloud data]

```
In the extract folder you will get 4 pre-recorded sequences with 10 point-cloud files each. 
Those files are the input for further processing.

1. Generate the accumulated cloud files. Use the following command:
```
bin/el_accu -s data_tutorial/extract/000/ -t data_tutorial/accu/000.pcd
..
bin/el_accu -s data_tutorial/extract/003/ -t data_tutorial/accu/003.pcd
```
To use the differencing you have to use at least 2 sequences.

2. Generate the label clouds files: 
The first file in the given folder (sorted by name: 000.pcd) will be considered background.
The rest of the files contain 1 additional object (001.pcd - 003.pcd).
Use the following command:
```
bin/el_diff -s data_tutorial/accu/ -t data_tutorial/label -v
```
You will see a visualization of the labeling process.
On the left side is the input cloud, and the generated label template on the right.
The middle window displays the current refinement step for each object.
Type 'n' in the visualization window to get to the next step.
The folder data_tutorial/label contains the final result. 
You can view the result files with Point Cloud Library (PCL) pcl_viewer.
```
pcl_viewer data_tutorial/label/* -multiview 1
```
Hit 6 to see the labeling.

