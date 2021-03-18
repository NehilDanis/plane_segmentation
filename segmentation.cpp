#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char *argv[]) {

  PointCloudT::Ptr cloud(new PointCloudT);
  PointCloudT::Ptr cloud_inliers(new PointCloudT);
  PointCloudT::Ptr cloud_outliers(new PointCloudT);

  // USE YOUR OWN POINTCLOUD FILE HERE!
  // Load point cloud
  if (pcl::io::loadPCDFile("arm.pcd", *cloud) < 0) {
    PCL_ERROR("Could not load PCD file !\n");
    return (-1);
  }

  /*
   * How RANSAC Plane detection works
   * 1. Select three random points from your point cloud that will form a plane
   * 2. Find the parameters of the plane.
   * 3. Then using point to plane equation calculate the distance between the
   * points in the point cloud to this plane
   * 4. If the distance is within the specified threshold then these points will
   * be considered as inliners.
   * 5. Repeat the process max_iterations time.
   * */
  pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  // The plane equation will have 4 coefficients
  plane->values.resize(4);

  // Create a segmentation object
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setDistanceThreshold(23);
  seg.setInputCloud(cloud);
  seg.segment(*inliers_plane, *plane);

  if (inliers_plane->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    return (-1);
  }

  std::cout << inliers_plane->indices.size() << std::endl;

  // Extract inliers
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);     // Extract the inliers
  extract.filter(*cloud_inliers); // cloud_inliers contains the plane

  // Extract outliers
  extract.setNegative(true); // Extract the outliers
  extract.filter(
      *cloud_outliers); // cloud_outliers contains everything but the plane

  // create a visualizer
  pcl::visualization::PCLVisualizer viewer("PCL visualizer");

  pcl::visualization::PointCloudColorHandlerCustom<PointT>
      cloud_inliers_handler(cloud, 255, 20, 20); // Plane in RED
  viewer.addPointCloud(cloud_inliers, cloud_inliers_handler, "cloud inliers");

  pcl::visualization::PointCloudColorHandlerCustom<PointT>
      cloud_outliers_handler(cloud, 200, 200, 200); // Everything else in GRAY
  viewer.addPointCloud(cloud_outliers, cloud_outliers_handler,
                       "cloud outliers");

  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }

  return 0;
}
