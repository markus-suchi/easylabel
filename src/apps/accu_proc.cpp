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
 * @file accu_proc.cpp
 * @author Markus Suchi (suchi@acin.tuwien.ac.at)
 * @date 2019
 * @brief : Commandline program for accumulating pointcloud data from files.
 *
 */

#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <boost/program_options.hpp>

#include <easylabel/depth_accumulator.h>
#include <ext/v4r/io/include/filesystem.h>

typedef pcl::PointXYZRGB PointT;

int main(int argc, char** argv) {
  namespace po = boost::program_options;

  std::string test_dir("");
  std::string output_dir("");

  po::options_description desc(
      "el_accu: Accumulates depth from sequences of pointclouds"
      "\n========================================================"
      "\n**Allowed options");
  desc.add_options()("help,h", "produce help message")("source_dir,s", po::value<std::string>(&test_dir)->required(),
                                                       "Directory with scenes stored as point clouds (.pcd).")(
      "target_dir,t", po::value<std::string>(&output_dir)->required(), "Outputfile for accumulated point cloud.");

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

  std::vector<std::string> views = v4r::io::getFilesInDirectory(test_dir, ".*.pcd", false);

  easylabel::DepthAccumulator accumulator;

  pcl::PointCloud<PointT>::Ptr accu_cloud(new pcl::PointCloud<PointT>);
  for (size_t v_id = 0; v_id < views.size(); v_id++) {
    const std::string fn = test_dir + views[v_id];
    const std::string on = output_dir + views[v_id];

    std::cout << "Accumulate file " << v_id + 1 << "/" << views.size() << " " << views[v_id] << std::endl;

    pcl::io::loadPCDFile(fn, *accu_cloud);
    accumulator.Add<PointT>(accu_cloud);
  }

  pcl::PointCloud<PointT>::Ptr outcloud(new pcl::PointCloud<PointT>);
  // take RGB infos from last image
  accumulator.Apply(accu_cloud, outcloud);
  pcl::io::savePCDFile(output_dir, *outcloud, true);
}
