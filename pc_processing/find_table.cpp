#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}


int main(int argc, char** argv)
{
    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBA>);
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // initialize extract indices
    // pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

    // populate our PointCloud with points
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("test.pcd", *cloud);

    std::vector<int> inliers;

    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (model_p);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    // get outliers 
    // extract.setInputCloud (cloud);
    // extract.setIndices (inliers);
    // extract.setNegative (true);
    // extract.filter (*cloud_f);
    // // copies all inliers of the model computed to another PointCloud
    pcl::copyPointCloud (*cloud, inliers, *final);

    // creates the visualization object and adds either our original cloud or all of the inliers
    // depending on the command line arguments specified.
    pcl::visualization::PCLVisualizer::Ptr viewer;
    // viewer = simpleVis(final);
    viewer = simpleVis(cloud);
    // viewer = simpleVis(cloud_f);

    while (!viewer->wasStopped ())
    {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
    }
    return 0;
 }