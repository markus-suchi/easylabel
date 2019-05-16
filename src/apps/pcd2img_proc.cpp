/****************************************************************************
** Copyright (C) 2017 TU Wien, ACIN, Vision 4 Robotics (V4R) group
** Contact: v4r.acin.tuwien.ac.at
**
** This file is part of V4R - EasyLabel
**
** V4R - EasyLabel is distributed under dual licenses - GPLv3 or closed source.
**
** GNU General Public License Usage
** V4R - EasyLabel is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published
** by the Free Software Foundation version 3.
**
** V4R - EasyLabel is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** Please review the following information to ensure the GNU General Public
** License requirements will be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** Commercial License Usage
** If GPL is not suitable for your project, you must purchase a commercial
** license to use V4R - EasyLabel.
** Licensees holding valid commercial V4R - EasyLabel licenses may
** use this file in accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the
** terms contained in a written agreement between you and TU Wien, ACIN, V4R.
** For licensing terms and conditions please contact office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

/**
 * @file pcd2img_proc.cpp
 * @author Markus Suchi (suchi@acin.tuwien.ac.at)
 * @date 2019
 * @brief : Commandline program for dumping depth-, rgb-,
 *          and label-images from pointcloud
 *
*/

#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <stdint.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>

typedef pcl::PointXYZRGBL PointT;

int main(int argc, char** argv) {
  uint16_t nan_label = 0;
  namespace po = boost::program_options;
  std::string source_file("");
  std::string target_rgb("");
  std::string target_depth("");
  std::string target_label("");

  po::options_description desc(
      "el_pcd2img: Dumps depth-, rgb-, and label-images from pointcloud"
      "\n================================================================"
      "\n**Allowed options");
  desc.add_options()("help,h", "produce help message")(
      "source_file,s", po::value<std::string>(&source_file)->required(), "Source file for  (.pcd).")(
      "r,r", po::value<std::string>(&target_rgb)->default_value(""), "Output file for rgb.")(
      "d,d", po::value<std::string>(&target_depth)->default_value(""), "Output file for depth.")(
      "l,l", po::value<std::string>(&target_label)->default_value(""), "Output file for label.");
  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
  std::vector<std::string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);
  po::store(parsed, vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 0;
  }
  try {
    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return 0;
  }

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  pcl::io::loadPCDFile(source_file, *cloud);

  bool do_label = false;
  bool do_rgb = false;
  bool do_depth = false;

  if (target_rgb.empty() && target_depth.empty() && target_label.empty()) {
    std::cout << "ERROR: At least one output has to be defined." << std::endl;
    std::cout << desc << std::endl;
    return 1;
  } else {
    if (!target_rgb.empty()) do_rgb = true;
    if (!target_label.empty()) do_label = true;
    if (!target_depth.empty()) do_depth = true;
  }

  cv::Mat img_rgb;
  img_rgb.create(cloud->height, cloud->width, CV_8UC3);
  cv::Mat img_depth;
  img_depth.create(cloud->height, cloud->width, CV_16UC1);
  cv::Mat img_label;
  img_label.create(cloud->height, cloud->width, CV_16UC1);

  for (size_t cols = 0; cols < cloud->width; ++cols) {
    for (size_t rows = 0; rows < cloud->height; ++rows) {
      if (pcl::isFinite(cloud->at(cols, rows))) {
        if (do_label) {
          img_label.at<uint16_t>(rows, cols) = cloud->at(cols, rows).label + 1;
        }
        if (do_depth) {
          img_depth.at<uint16_t>(rows, cols) = cloud->at(cols, rows).z * 1000.0f;
        }
        if (do_rgb) {
          img_rgb.at<cv::Vec3b>(rows, cols)[2] = cloud->at(cols, rows).r;
          img_rgb.at<cv::Vec3b>(rows, cols)[1] = cloud->at(cols, rows).g;
          img_rgb.at<cv::Vec3b>(rows, cols)[0] = cloud->at(cols, rows).b;
        }
      } else {
        if (do_label) {
          img_label.at<uint16_t>(rows, cols) = nan_label;
        }
        if (do_depth) {
          img_depth.at<uint16_t>(rows, cols) = 0;
        }
        if (do_rgb) {
          img_rgb.at<cv::Vec3b>(rows, cols)[2] = cloud->at(cols, rows).r;
          img_rgb.at<cv::Vec3b>(rows, cols)[1] = cloud->at(cols, rows).g;
          img_rgb.at<cv::Vec3b>(rows, cols)[0] = cloud->at(cols, rows).b;
        }
      }
    }
  }

  if (do_label) {
    cv::imwrite(target_label, img_label);
  }
  if (do_depth) {
    cv::imwrite(target_depth, img_depth);
  }
  if (do_rgb) {
    cv::imwrite(target_rgb, img_rgb);
  }

  return 0;
}
