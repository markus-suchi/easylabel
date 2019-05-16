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
 * @file depth_accumulator.h
 * @author Markus Suchi (suchi@acin.tuwien.ac.at)
 * @date 2019
 * @brief : Accumulates depth from incoming pointclouds
 *
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

namespace easylabel {
class DepthAccumulator {
 public:
  DepthAccumulator();
  DepthAccumulator(float thresh_);
  DepthAccumulator(float thresh_, uint32_t width, uint32_t height);

  void Get(pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud);
  void Apply(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input_cloud,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud);

  template <typename PointT>
  void Add(const typename pcl::PointCloud<PointT>::ConstPtr input_cloud);
  void Reset();

 private:
  float thresh_;
  uint32_t width_;
  uint32_t height_;
  cv::Mat accu_cc_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accu_cloud_;
};
}  // namespace easylabel
