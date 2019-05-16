/****************************************************************************
** Copyright (C) 2019 TU Wien, ACIN, Vision 4 Robotics (V4R) group
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
 * @file label_helper.h
 * @author Markus Suchi (suchi@acin.tuwien.ac.at)
 * @date 2019
 * @brief : Helper functions for assigning labels using Euclidian distance clustering and nearest neigbors.
 *
 */

#pragma once

#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

namespace easylabel {
namespace labelhelper {

static float CLUSTER_TOLERANCE = 0.006f;
static float MIN_LABEL_DISTANCE = 0.0017f;
static int MIN_CLUSTER_SIZE = 180;
static float MAX_LABEL_DISTANCE = 1000.0f;
static float MAX_NN_DISTANCE = 0.003f;

float findKNearestNeighbor(const pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr kdtree,
                           const pcl::PointXYZRGBL &search_point, const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
                           pcl::PointXYZRGBL &n_neighbor, int &index);

float findNearestNeighbor(const pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr kdtree,
                          const pcl::PointXYZRGBL &search_point, const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
                          pcl::PointXYZRGBL &n_neighbor, int &index, float dist = MIN_LABEL_DISTANCE);

void extractEuclidianDistanceCluster(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered,
                                     pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_clustered, const uint32_t label);

void addBackground(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered,
                   pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_clustered, const uint32_t label);

uint32_t addBackgroundwithLabels(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered,
                                 pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_clustered);

void maskPointCloud(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr input, const std::vector<int> &indices,
                    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr result);

void filterCloudByLabel(const uint32_t label, const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr input,
                        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr output);
}  // namespace labelhelper
}  // namespace easylabel
