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
 * @file depth_differ.h
 * @author Markus Suchi (suchi@acin.tuwien.ac.at)
 * @date 2019
 * @brief : Finds depth differences between two pointcloud files.
 *          Threshold parameter can be depth adapted using quadratic scaling.
 *
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

namespace easylabel {
class DepthDiffer {
 public:
  DepthDiffer();
  void SetDisableAdaptive(float thresh);
  void SetEnableAdaptive(float p1, float p2, float p3);
  template <typename PointT>
  void Apply(const typename pcl::PointCloud<PointT>::ConstPtr cloud,
             const typename pcl::PointCloud<PointT>::ConstPtr cloud2, std::vector<int> &differ_indices) const;

 private:
  float thresh_;
  float p1_;
  float p2_;
  float p3_;
  bool adaptive_;

  float AdaptiveDepth(float depth) const;
};
}  // namespace easylabel
