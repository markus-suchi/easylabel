#include "depth_differ.h"

namespace easylabel {
DepthDiffer::DepthDiffer() : adaptive_(true), thresh_(0.0f), p1_(0.0055), p2_(0.0005), p3_(0.003){};

void DepthDiffer::SetDisableAdaptive(float thresh) {
  thresh_ = thresh;
  adaptive_ = false;
}

void DepthDiffer::SetEnableAdaptive(float p1, float p2, float p3) {
  thresh_ = 0.0f;
  p1_ = p1;
  p2_ = p2;
  p3_ = p3;
  adaptive_ = true;
}

float DepthDiffer::AdaptiveDepth(float depth) const {
  if (!adaptive_) {
    return 0.0f;
  } else {
    if (!std::isnan(depth)) {
      return (p1_ * depth * depth + p2_ * depth + p3_);
    } else {
      return 0.0f;
    }
  }
}

template <typename PointT>
void DepthDiffer::Apply(const typename pcl::PointCloud<PointT>::ConstPtr cloud,
                        const typename pcl::PointCloud<PointT>::ConstPtr cloud2,
                        std::vector<int> &differ_indices) const {
  for (size_t cols = 0; cols < cloud2->width; ++cols) {
    for (size_t rows = 0; rows < cloud2->height; ++rows) {
      float z_cloud = 0;
      float z_cloud2 = 0;
      float thresh = 0;

      if (!std::isnan(cloud->at(cols, rows).z)) {
        z_cloud = cloud->at(cols, rows).z;
      }

      if (!std::isnan(cloud2->at(cols, rows).z)) {
        z_cloud2 = cloud2->at(cols, rows).z;
        float diff = z_cloud2 - z_cloud;
        thresh = thresh + AdaptiveDepth(cloud2->at(cols, rows).z);
        if (std::fabs(diff) > thresh) {
          // std::cout << "depth/diff/thresh/athresh: " << cloud2->at(cols, rows).z << "/" << diff <<
          // "/"<<thresh<<"/"<<temp_thresh << std::endl;
          differ_indices.push_back(rows * cloud2->width + cols);
        }
      }
    }
  }
}

template void DepthDiffer::Apply<pcl::PointXYZ>(const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                                pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,
                                                std::vector<int> &differ_indices) const;
template void DepthDiffer::Apply<pcl::PointXYZRGB>(
    const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2, std::vector<int> &differ_indices) const;
template void DepthDiffer::Apply<pcl::PointXYZL>(const typename pcl::PointCloud<pcl::PointXYZL>::ConstPtr cloud,
                                                 pcl::PointCloud<pcl::PointXYZL>::ConstPtr cloud2,
                                                 std::vector<int> &differ_indices) const;
template void DepthDiffer::Apply<pcl::PointXYZRGBL>(
    const typename pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr input_cloud,
    pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud2, std::vector<int> &differ_indices) const;
}  // namespace easylabel
