#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test1.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test.pcd \n");
    return (-1);
  }
  // std::cout << "Loaded "
  //           << cloud->width * cloud->height
  //           << " data points from test_pcd.pcd with the following fields: "
  //           << std::endl;

  //   for (const auto& point: *cloud)
  //     std::cout << "    " << point.x
  //               << " "    << point.y
  //               << " "    << point.z << std::endl;

  return (0);
}