#include "depth_accumulator.h"

#include <limits>
#include <cmath>

#include <pcl/common/io.h>

namespace easylabel {
DepthAccumulator::DepthAccumulator() : thresh_(0.03f), width_(640), height_(480) {
  Reset();
};

DepthAccumulator::DepthAccumulator(float thresh_)
    : thresh_(thresh_), width_(640), height_(480) {
  Reset();
};

DepthAccumulator::DepthAccumulator(float thresh_, uint32_t width, uint32_t height)
    : thresh_(thresh_), width_(width), height_(height) {
  Reset();
};

void DepthAccumulator::Reset() {
  accu_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  accu_cloud_->height = height_;
  accu_cloud_->width = width_;
  accu_cloud_->resize(width_ * height_);
  accu_cc_=cv::Mat1f(width_,height_);

  for (int i = 0; i < width_; ++i) {
    for (int j = 0; j < height_; ++j) {
      accu_cc_.at<int>(i,j) = 0;
      accu_cloud_->at(i, j).x = std::numeric_limits<float>::quiet_NaN();
      accu_cloud_->at(i, j).y = std::numeric_limits<float>::quiet_NaN();
      accu_cloud_->at(i, j).z = std::numeric_limits<float>::quiet_NaN();
    }
  }
}

template <typename PointT>
void DepthAccumulator::Add(
    const typename pcl::PointCloud<PointT>::ConstPtr input_cloud) {
  assert(input_cloud->width == width_ && input_cloud->height == height_);
  for (int i = 0; i < width_; ++i) {
    for (int j = 0; j < height_; ++j) {
      float z = input_cloud->at(i, j).z;
      float x = input_cloud->at(i, j).x;
      float y = input_cloud->at(i, j).y;
      if (!std::isnan(z)) {
        if (accu_cc_.at<int>(i,j) == 0) {
          accu_cloud_->at(i, j).z = z;
          accu_cloud_->at(i, j).x = x;
          accu_cloud_->at(i, j).y = y;
          accu_cc_.at<int>(i,j)= accu_cc_.at<int>(i,j)+1;
        } else {
          if (std::fabs(accu_cloud_->at(i, j).z - z) < thresh_) {
            accu_cc_.at<int>(i,j)= accu_cc_.at<int>(i,j)+1;;
            accu_cloud_->at(i, j).z =
                (z +
                (accu_cc_.at<int>(i,j) - 1) * accu_cloud_->at(i, j).z) / accu_cc_.at<int>(i,j);
          } else {
            accu_cc_.at<int>(i,j)= accu_cc_.at<int>(i,j)-1;
          }
        }
      }
    }
  }
}

void DepthAccumulator::Get(pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud) {
  pcl::copyPointCloud(*accu_cloud_, *result_cloud);
}

void DepthAccumulator::Apply(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud) {
  assert(input_cloud->width == width_ && input_cloud->height == height_);
  result_cloud->width = width_;
  result_cloud->height = height_;
  result_cloud->resize(width_ * height_);
  result_cloud->is_dense = input_cloud->is_dense;

  pcl::PointXYZRGB addPoint;
  for (int rows = 0; rows < height_; ++rows) {
    for (int cols = 0; cols < width_; ++cols) {
      if (accu_cloud_->at(cols, rows).z == 0 ||
          std::isnan(accu_cloud_->at(cols, rows).z)) {
        addPoint.z = std::numeric_limits<float>::quiet_NaN();
        addPoint.r = input_cloud->at(cols, rows).r;
        addPoint.g = input_cloud->at(cols, rows).g;
        addPoint.b = input_cloud->at(cols, rows).b;
      } else {
        addPoint.x = accu_cloud_->at(cols, rows).x;
        addPoint.y = accu_cloud_->at(cols, rows).y;
        addPoint.z = accu_cloud_->at(cols, rows).z;
        addPoint.r = input_cloud->at(cols, rows).r;
        addPoint.g = input_cloud->at(cols, rows).g;
        addPoint.b = input_cloud->at(cols, rows).b;
      }
      result_cloud->at(cols, rows) = addPoint;
    }
  }
}

template void DepthAccumulator::Add<pcl::PointXYZ>(
    const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud);
template void DepthAccumulator::Add<pcl::PointXYZRGB>(
    const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input_cloud);
}  // namespace easylabel
