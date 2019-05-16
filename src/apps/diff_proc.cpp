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
 * @file diff_proc.cpp
 * @author Markus Suchi (suchi@acin.tuwien.ac.at)
 * @date 2019
 * @brief : Commandline program for labeling pointcloud data from files.
 *
 * Input is a directory containing sequential recorded pcl files.
 * First file in source directory is used as background image and assigned label
 * 0. Differences in depth are the basis to segment objects. The whole sequence
 * of files are processed to generate a template map. In a final step each point
 * of each file is compared to the nearest object in the template and gets the
 * corresponding label assigned. You can view and save results using
 * corresponding parameters.
 *
 */

#include <cmath>
#include <iostream>
#include <string>

#include <ext/v4r/io/include/filesystem.h>

#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/program_options.hpp>

#include <easylabel/depth_differ.h>
#include <easylabel/label_helper.h>

typedef pcl::PointXYZRGBL PointT;

// defaults
static float TEMP_ADAPTIVE_P1 = 0.0055f;
static float TEMP_ADAPTIVE_P2 = 0.0005f;
static float TEMP_ADAPTIVE_P3 = 0.003f;
static float CLUSTER_TOLERANCE = 0.006f;
static float MIN_THRESH_DISTANCE = 0.001f;
static float MIN_LABEL_DISTANCE = 0.0017f;
static int MIN_CLUSTER_SIZE = 180;
static float MAX_LABEL_DISTANCE = 1000.0f;
static float MAX_NN_DISTANCE = 0.003f;
// visualizer
static bool next = false;
static pcl::visualization::PCLVisualizer::Ptr viewer;

// VIZUALIZER
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
      *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
  if (event.getKeySym() == "n" && event.keyDown()) {
    next = true;
  }
}

int main(int argc, char *argv[]) {
  namespace po = boost::program_options;
  namespace el = easylabel;

  std::string source_dir("");
  std::string output_dir("");
  float thresh = MIN_THRESH_DISTANCE;
  bool adapt = true;
  bool noadapt = false;
  bool uselabels = false;
  bool reverse = false;
  bool nn = true;
  bool view = false;

  po::options_description desc(
      "el_differ: Depth difference labeling"
      "\n===================================="
      "\n**Allowed options");
  desc.add_options()("help,h", "produce help message")("source_dir,s", po::value<std::string>(&source_dir)->required(),
                                                       "Directory with scenes point clouds (.pcd)")(
      "target_dir,t", po::value<std::string>(&output_dir)->default_value("")->implicit_value("./"),
      "Directory with extracted point clouds (.pcd)")(
      "min_dist,m", po::value<float>(&thresh)->default_value(MIN_THRESH_DISTANCE)->implicit_value(MIN_THRESH_DISTANCE),
      "Minimal distance threshold for change (m)")("no_adaptive_smoothing,a", po::bool_switch(&noadapt),
                                                   "Disable depth adaptive distance change threshold")(
      "use_labels,b", po::bool_switch(&uselabels), "Reuse labeling from background pcd")(
      "view_results,v", po::bool_switch(&view), "Visualize resulting labeling")(
      "cluster_tolerance,c",
      po::value<float>(&CLUSTER_TOLERANCE)->default_value(CLUSTER_TOLERANCE)->implicit_value(CLUSTER_TOLERANCE),
      "Cluster tolerance")(
      "cluster_min_size,u",
      po::value<int>(&MIN_CLUSTER_SIZE)->default_value(MIN_CLUSTER_SIZE)->implicit_value(MIN_CLUSTER_SIZE),
      "Minimal cluster size")(
      "p1", po::value<float>(&TEMP_ADAPTIVE_P1)->default_value(TEMP_ADAPTIVE_P1)->implicit_value(TEMP_ADAPTIVE_P1),
      "Distance threshold P1: P1*x²+P2*x+P3")(
      "p2", po::value<float>(&TEMP_ADAPTIVE_P2)->default_value(TEMP_ADAPTIVE_P2)->implicit_value(TEMP_ADAPTIVE_P2),
      "Distance threshold P2: P1*x²+P2*x+P3")(
      "p3", po::value<float>(&TEMP_ADAPTIVE_P3)->default_value(TEMP_ADAPTIVE_P3)->implicit_value(TEMP_ADAPTIVE_P3),
      "Distance threshold P3: P1*x²+P2*x+P3")(
      "label_distance,l",
      po::value<float>(&MIN_LABEL_DISTANCE)->default_value(MIN_LABEL_DISTANCE)->implicit_value(MIN_LABEL_DISTANCE),
      "Minimal distance to template label")("reverse,r",
                                            po::value<bool>(&reverse)->default_value(false)->implicit_value(false),
                                            "Reverse image sorting order")(
      "nearest neighbor,n", po::value<bool>(&nn)->default_value(true)->implicit_value(true),
      "Set nearest neigbor instead of distance clustering")(
      "nearest_neighbor_max_distance,d",
      po::value<float>(&MAX_NN_DISTANCE)->default_value(MAX_NN_DISTANCE)->implicit_value(MAX_NN_DISTANCE),
      "Max allowed distance to nearest neighbor");

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

  // override if adaptive smoothing is turned off
  if (noadapt) adapt = false;

  // prepare depth differencing
  el::DepthDiffer differ;
  if (adapt) {
    differ.SetEnableAdaptive(TEMP_ADAPTIVE_P1, TEMP_ADAPTIVE_P2, TEMP_ADAPTIVE_P3);
  } else {
    differ.SetDisableAdaptive(thresh);
  }

  // if we have a directory apply depth difference sequence, else do not create a template and require to get a
  // template via parameter
  bool create_template = true;
  std::vector<std::string> views;
  int start_id = 1;
  if (std::string::npos == source_dir.find(".pcd")) {
    views = v4r::io::getFilesInDirectory(source_dir, ".*.pcd", false);
  } else {
    views.push_back(boost::filesystem::path(source_dir).filename().string());
    source_dir = boost::filesystem::path(source_dir).parent_path().string() + "/";
    create_template = false;
    start_id = 0;
  }

  std::cout << "Processing " << views.size() << " files in " << source_dir << " to " << output_dir << "." << std::endl;
  // sort the entries
  if (reverse) {
    std::sort(views.begin(), views.end(), std::greater<std::string>());
  } else {
    std::sort(views.begin(), views.end());
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr result_clustered(new pcl::PointCloud<PointT>);

  // first cloud is background
  std::string fn = source_dir + views[0];
  pcl::io::loadPCDFile(fn, *cloud);
  uint32_t startlabel = 0;
  if (uselabels && !create_template) {
    el::labelhelper::addBackgroundwithLabels(cloud, result_clustered);
  } else if (uselabels) {
    startlabel = el::labelhelper::addBackgroundwithLabels(cloud, result_clustered);
  } else {
    el::labelhelper::addBackground(cloud, result_clustered, 0);
  }

  cloud->clear();

  if (create_template) {
    // CREATE TEMPLATE MAP
    std::cout << "Create template map (nn: " << std::boolalpha << nn << ")" << std::endl;
    // always two consecutive images
    for (size_t v_id = 0; v_id < views.size() - 1; v_id++) {
      fn = source_dir + views[v_id];
      const std::string fn2 = source_dir + views[v_id + 1];
      std::cout << "Comparing file " << v_id + 1 << "/" << views.size() << " " << views[v_id] << " with "
                << views[v_id + 1] << std::endl;
      pcl::io::loadPCDFile(fn, *cloud);
      pcl::io::loadPCDFile(fn2, *cloud2);

      std::vector<int> differ_indices;
      differ_indices.clear();
      differ_indices.reserve(10000);
      std::cout << "Calculating difference...";
      differ.Apply<PointT>(cloud, cloud2, differ_indices);
      std::cout << "found " << differ_indices.size() << " points." << std::endl;

      pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>);
      pcl::PointCloud<PointT>::Ptr result_masked(new pcl::PointCloud<PointT>);
      el::labelhelper::maskPointCloud(cloud2, differ_indices, result_masked);
      if (differ_indices.size() > 0) {
        if (nn) {
          pcl::search::KdTree<PointT>::Ptr kdtree3(new pcl::search::KdTree<PointT>);
          kdtree3->setInputCloud(result_clustered);
          int cc = 0;
          for (size_t i = 0; i < result_masked->points.size(); i++) {
            PointT point = result_masked->points[i];
            if (std::isnan(point.z)) continue;

            PointT neighbor;
            int id = 0;
            float dist =
                el::labelhelper::findNearestNeighbor(kdtree3, point, result_clustered, neighbor, id, MAX_NN_DISTANCE);
            if (dist <= MAX_NN_DISTANCE) {
              point.label = neighbor.label;
              // This excludes this point from clustering since it is already assigned a label from neighbor (no new
              // point)
              result_masked->points[i].z = std::numeric_limits<float>::quiet_NaN();
              result_clustered->push_back(point);
              cc++;
            } else {
              point.label = startlabel + v_id + 1;
            }
          }
          if (differ_indices.size() == cc) {
            std::cout << "Warning: All labels are assigned to existing labels. No new labels added." << std::endl;
          } else {
            el::labelhelper::extractEuclidianDistanceCluster(result_masked, result_clustered, startlabel + v_id + 1);
          }
        } else {
          el::labelhelper::extractEuclidianDistanceCluster(result_masked, result_clustered, startlabel + v_id + 1);
        }
      }
    }
  }  // END CREATE TEMPLATE MAP

  // visualizer
  int v1(0);
  int v2(0);
  int v3(0);
  if (view) {
    viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("PCL Viewer"));
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void *)&viewer);
    // differ result vis
    viewer->createViewPort(0.0, 0.0, 0.32, 1.0, v1);
    // cluster result vis
    viewer->createViewPort(0.33, 0.0, 0.65, 1.0, v2);
    // cluster template vis
    viewer->createViewPort(0.66, 0.0, 0.98, 1.0, v3);
    //viewer->addText("Hit 'n' to continue.",35,15,"Message",v1);
    viewer->addText("Hit 'n' to continue.",15,15,15,1.0,1.0,1.0,"Message",v1);
  }

  // LABEL SINGLE FRAMES
  std::cout << "Label single frames." << std::endl;
  pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);

  for (size_t v_id = start_id; v_id < views.size(); v_id++) {
    std::cout << "-----------Refining file " << v_id << ": " << views[v_id] << "----------------" << std::endl;
    pcl::PointCloud<PointT>::Ptr result_labeled(new pcl::PointCloud<PointT>);
    fn = source_dir + views[v_id];
    pcl::io::loadPCDFile(fn, *cloud);

    // make labeled version
    pcl::copyPointCloud(*cloud, *result_labeled);
    for (size_t i = 0; i < result_labeled->points.size(); i++) {
      // mark as unassigned (last index + 1)
      result_labeled->points[i].label = startlabel + views.size() + 1;
    }

    // lookup entry from source image to template image, find label and assign
    // we do need to make a copy so that we do not region grow into other labels
    pcl::PointCloud<PointT>::Ptr result_labeled_template(new pcl::PointCloud<PointT>);
    *result_labeled_template = *result_labeled;
    kdtree->setInputCloud(result_labeled_template);

    int i = 1;
    for (size_t index = 0; index < result_clustered->size(); index++) {
      float x_cloud = result_clustered->at(index).x;
      float y_cloud = result_clustered->at(index).y;
      float z_cloud = result_clustered->at(index).z;
      uint32_t label = result_clustered->at(index).label;
      PointT neighbor;
      PointT search;
      int id = 0;
      if ((!std::isnan(z_cloud)) && (startlabel + v_id >= label)) {
        search.x = x_cloud;
        search.y = y_cloud;
        search.z = z_cloud;
        search.label = label;

        float dist = el::labelhelper::findNearestNeighbor(kdtree, search, result_labeled_template, neighbor, id);
        if (dist <= MIN_LABEL_DISTANCE) {
          result_labeled->at(id).label = search.label;
        }
      }
      i++;
    }

    pcl::PointCloud<PointT>::Ptr result_clustered_current(new pcl::PointCloud<PointT>);
    el::labelhelper::filterCloudByLabel(startlabel + v_id, result_clustered, result_clustered_current);
    pcl::search::KdTree<PointT>::Ptr kdtree2(new pcl::search::KdTree<PointT>);
    kdtree2->setInputCloud(result_clustered_current);

    // preprocess image, all unassigned labels get nearest neighbor label
    // create new labeled object without altering the current one -> avoid
    // region leaking
    pcl::PointCloud<PointT>::Ptr result_labeled_template_refine(new pcl::PointCloud<PointT>);
    *result_labeled_template_refine = *result_labeled;

    for (size_t index = 0; index < result_labeled_template_refine->size(); index++) {
      float x_cloud = result_labeled_template_refine->at(index).x;
      float y_cloud = result_labeled_template_refine->at(index).y;
      float z_cloud = result_labeled_template_refine->at(index).z;
      uint32_t label = result_labeled_template_refine->at(index).label;
      PointT neighbor;
      PointT search;
      int id = 0;
      if ((!std::isnan(z_cloud)) && label == startlabel + views.size() + 1) {
        search.x = x_cloud;
        search.y = y_cloud;
        search.z = z_cloud;
        search.label = label;

        float dist = el::labelhelper::findKNearestNeighbor(kdtree2, search, result_clustered_current, neighbor, id);
        if (dist < MAX_LABEL_DISTANCE) {
          result_labeled->at(index).label = result_clustered_current->at(id).label;
        } else {
          std::cout << "Skipping point, nearest labeled point distance  " << dist << ">=" << MAX_LABEL_DISTANCE
                    << std::endl;
        }
      }
    }
    // END LABEL SINGLE FRAMES

    // visualizer
    if (view) {
      pcl::visualization::PointCloudColorHandlerGenericField<PointT> labelvis2(result_clustered, "label");
      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
      pcl::visualization::PointCloudColorHandlerGenericField<PointT> labelvis(result_labeled, "label");      
      if (!viewer->wasStopped()) {        
        if (!viewer->updatePointCloud<PointT>(cloud, rgb, "result")) {
          viewer->addPointCloud<PointT>(cloud, rgb, "result", v1);
        }
        if (!viewer->updatePointCloud<PointT>(result_labeled, labelvis, "clustered")) {
          viewer->addPointCloud<PointT>(result_labeled, labelvis, "clustered", v2);
        }
        if (!viewer->updatePointCloud<PointT>(result_clustered, labelvis2, "template")) {
          viewer->addPointCloud<PointT>(result_clustered, labelvis2, "template", v3);
        }

        while (!next) {
          usleep(1000);
          viewer->spinOnce();
        }
        next = false;
      }
    }

    if (output_dir != "") {
      std::string out_file = "";
      if (std::string::npos == output_dir.find(".pcd")) {
        out_file = output_dir + "/result_" + views[v_id];
      } else {
        out_file = output_dir;
      }
      std::cout << "Saving " << out_file << std::endl;
      pcl::io::savePCDFile(out_file, *result_labeled, true);
    }
  }

  return 0;
}
