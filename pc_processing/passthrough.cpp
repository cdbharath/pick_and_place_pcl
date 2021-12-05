#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <thread>


using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

int main ()
{
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile ("test1.pcd", *cloud);

  // Fill in the cloud data
//   cloud->width  = 5;
//   cloud->height = 1;
//   cloud->points.resize (cloud->width * cloud->height);

//   for (auto& point: *cloud)
//   {
//     point.x = 1024 * rand () / (RAND_MAX + 1.0f);
//     point.y = 1024 * rand () / (RAND_MAX + 1.0f);
//     point.z = 1024 * rand () / (RAND_MAX + 1.0f);
//   }

//   std::cerr << "Cloud before filtering: " << std::endl;
//   for (const auto& point: *cloud)
//     std::cerr << "    " << point.x << " "
//                         << point.y << " "
//                         << point.z << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.94, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("box.pcd", *cloud_filtered, false);

  std::cerr << "Cloud after filtering: " << std::endl;
//   for (const auto& point: *cloud_filtered)
//     std::cerr << "    " << point.x << " "
//                         << point.y << " "
//                         << point.z << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = simpleVis(cloud_filtered);
  
  while (!viewer->wasStopped ())
  {
  viewer->spinOnce (100);
  std::this_thread::sleep_for(100ms);
  }

  return (0);
}