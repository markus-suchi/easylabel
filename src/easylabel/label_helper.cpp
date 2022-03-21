#include "label_helper.h"

namespace easylabel{
namespace labelhelper{
float findKNearestNeighbor(const pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr kdtree, const pcl::PointXYZRGBL &search_point,
                                  const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud, pcl::PointXYZRGBL &n_neighbor, int &index) {
  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  size_t i = 0;

  if (kdtree->nearestKSearch(search_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
    for (i = 0; i < pointIdxNKNSearch.size(); ++i) n_neighbor = cloud->points[pointIdxNKNSearch[0]];
    index = pointIdxNKNSearch[0];
    return pointNKNSquaredDistance[0];
  }
  return MAX_LABEL_DISTANCE;
}

float findNearestNeighbor(const pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr kdtree, const pcl::PointXYZRGBL &search_point,
                                 const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud, pcl::PointXYZRGBL &n_neighbor, int &index,
                                 float dist) {
  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  size_t i = 0;

  if (kdtree->radiusSearch(search_point, dist, pointIdxNKNSearch, pointNKNSquaredDistance, 1) > 0) {
    for (i = 0; i < pointIdxNKNSearch.size(); ++i) n_neighbor = cloud->points[pointIdxNKNSearch[0]];
    index = pointIdxNKNSearch[0];
    return pointNKNSquaredDistance[0];
  }
  return MAX_LABEL_DISTANCE;
}

void extractEuclidianDistanceCluster(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered,
                                            pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_clustered, const uint32_t label) {
  std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::removeNaNFromPointCloud(*cloud_filtered, *outputCloud, *indices);

  pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBL>);
  tree->setInputCloud(cloud_filtered,indices);
  //tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBL> ec;
  ec.setClusterTolerance(CLUSTER_TOLERANCE);
  ec.setMinClusterSize(MIN_CLUSTER_SIZE);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.setIndices(indices);
  ec.extract(cluster_indices);

  std::cout << "#cluster found: " << cluster_indices.size() << std::endl;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
      if (!std::isnan(cloud_filtered->points[*pit].z)) {
        pcl::PointXYZRGBL clusterPoint(cloud_filtered->points[*pit]);
        clusterPoint.label = label;
        cloud_clustered->push_back(clusterPoint);
      }
    }
  }
}

void addBackground(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered,
                          pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_clustered, const uint32_t label) {
  for (size_t i = 0; i < cloud_filtered->size(); ++i) {
    if (!std::isnan(cloud_filtered->points[i].z)) {
      pcl::PointXYZRGBL clusterPoint(cloud_filtered->points[i]);
      clusterPoint.label = label;
      cloud_clustered->push_back(clusterPoint);
    }
  }
}

uint32_t addBackgroundwithLabels(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered,
                                        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_clustered) {
  uint32_t label = 0;

  for (size_t i = 0; i < cloud_filtered->size(); ++i) {
    if (!std::isnan(cloud_filtered->points[i].z)) {
      pcl::PointXYZRGBL clusterPoint(cloud_filtered->points[i]);
      if (label <= clusterPoint.label) {
        label = clusterPoint.label;
      }
      cloud_clustered->push_back(clusterPoint);
    }
  }

  return label;
}

void maskPointCloud(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr input, const std::vector<int> &indices,
                           pcl::PointCloud<pcl::PointXYZRGBL>::Ptr result) {
  const float init_value = std::numeric_limits<float>::quiet_NaN();
  pcl::PointXYZRGBL init_point;
  init_point.x = init_point.y = init_point.z = init_value;

  result->is_dense = input->is_dense;
  result->height = input->height;
  result->width = input->width;
  result->points.resize(input->width * input->height, init_point);

  for (std::vector<int>::const_iterator it = indices.begin(); it != indices.end(); ++it) {
    result->points[*it] = input->points[*it];
  }
}

void filterCloudByLabel(const uint32_t label, const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr input,
                               pcl::PointCloud<pcl::PointXYZRGBL>::Ptr output) {
  for (size_t iindex = 0; iindex < input->size(); iindex++) {
    if (input->points[iindex].label <= label) {
      output->points.push_back(input->points[iindex]);
    }
  }
}
} //namespace labelhelper
} //namespace easylabel
