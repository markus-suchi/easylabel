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
 * @file remove_proc.cpp
 * @author Markus Suchi (suchi@acin.tuwien.ac.at)
 * @date 2019
 * @brief : Commandline program to remove small patches from pointclouds,
 *          using Euclidian distance clustering.
 *
 */

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/program_options.hpp>

typedef pcl::PointXYZRGBL PointT;

static int MIN_CLUSTER_SIZE = 5;
static float CLUSTER_TOLERANCE = 0.03f;

static void extractEuclidianDistanceCluster(const pcl::PointCloud<PointT>::Ptr cloud,
                                            pcl::PointCloud<PointT>::Ptr cloud_clustered) {
  // copy to output
  *cloud_clustered = *cloud;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud_clustered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(CLUSTER_TOLERANCE);
  ec.setMinClusterSize(1);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_clustered);
  ec.extract(cluster_indices);

  int label = 100;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    if (it->indices.size() <= MIN_CLUSTER_SIZE) {
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
        if (!std::isnan(cloud_clustered->points[*pit].z)) {
          cloud_clustered->points[*pit].label = label;
        }
      }
    }
  }
}

static void maskPointCloud(const pcl::PointCloud<PointT>::Ptr input, const std::vector<int> &indices,
                           pcl::PointCloud<PointT>::Ptr result) {
  const float init_value = std::numeric_limits<float>::quiet_NaN();
  PointT init_point;
  init_point.x = init_point.y = init_point.z = init_value;

  result->is_dense = input->is_dense;
  result->height = input->height;
  result->width = input->width;
  result->points.resize(input->width * input->height, init_point);

  for (std::vector<int>::const_iterator it = indices.begin(); it != indices.end(); ++it) {
    result->points[*it] = input->points[*it];
  }
}

static void filterCloudByLabel(const uint32_t label, const pcl::PointCloud<PointT>::Ptr input) {
  for (size_t iindex = 0; iindex < input->size(); iindex++) {
    if (input->points[iindex].label == label) {
      input->points[iindex].label = 0;
      input->points[iindex].x = std::numeric_limits<float>::quiet_NaN();
      input->points[iindex].y = std::numeric_limits<float>::quiet_NaN();
      input->points[iindex].z = std::numeric_limits<float>::quiet_NaN();
    }
  }
}

int main(int argc, char *argv[]) {
  namespace po = boost::program_options;

  std::string source_file("");
  std::string output_file("");
  bool view = false;

  po::options_description desc(
      "el_remove: Remove small patches from pcd files"
      "\n=============================================="
      "\n**Allowed options");
  desc.add_options()("help,h", "produce help message")(
      "source_file,s", po::value<std::string>(&source_file)->required(), "source pcd file")(
      "target_file,t", po::value<std::string>(&output_file)->required(), "File with filtered point cloud.")(
      "view_results,v", po::bool_switch(&view), "View results.")(
      "cluster_tolerance,c",
      po::value<float>(&CLUSTER_TOLERANCE)->default_value(CLUSTER_TOLERANCE)->implicit_value(CLUSTER_TOLERANCE),
      "Cluster tolerance.")(
      "cluster_min_size,u",
      po::value<int>(&MIN_CLUSTER_SIZE)->default_value(MIN_CLUSTER_SIZE)->implicit_value(MIN_CLUSTER_SIZE),
      "Minimal cluster size.");

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
  } catch (std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return 0;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr result_clustered(new pcl::PointCloud<PointT>);

  std::cout << source_file << std::endl;
  pcl::io::loadPCDFile(source_file, *cloud);
  extractEuclidianDistanceCluster(cloud, result_clustered);

  // show result
  if (view) {
    pcl::visualization::PCLVisualizer viewer = pcl::visualization::PCLVisualizer("PCL Viewer");
    // differ result vis
    int v1(0);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    // cluster result vis
    int v2(0);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> labelvis(result_clustered, "label");

    while (!viewer.wasStopped()) {
      if (!viewer.updatePointCloud<PointT>(cloud, rgb, "result")) {
        viewer.addPointCloud<PointT>(cloud, rgb, "result", v1);
      }
      if (!viewer.updatePointCloud<PointT>(result_clustered, labelvis, "template")) {
        viewer.addPointCloud<PointT>(result_clustered, labelvis, "template", v2);
      }
      usleep(100);
      viewer.spinOnce();
    }
  }
  // copy the clustered cloud and filter by label
  pcl::PointCloud<PointT>::Ptr result_filtered(new pcl::PointCloud<PointT>);
  *result_filtered = *result_clustered;
  filterCloudByLabel(100, result_filtered);

  // copy for rgb output
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  copyPointCloud(*result_filtered, *result_rgb);

  // save labeled pointcloud frame
  if (output_file != "") {
    std::cout << "saving " << output_file << std::endl;
    pcl::io::savePCDFile(output_file, *result_rgb, true);
  }
  return 0;
}
