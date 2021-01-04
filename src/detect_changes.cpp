#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <vector>
#include <ctime>

int
main (int argc, char** argv)
{
  srand ((unsigned int) time (NULL));

  // Octree resolution - side length of octree voxels
  float resolution = 0.1f;

  // Instantiate octree-based point cloud change detection class
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ> );

  // Generate pointcloud data for cloudA
  cloudA->width = 10000;
  cloudA->height = 1;
  cloudA->points.resize (cloudA->width * cloudA->height);

  for (std::size_t i = 0; i < cloudA->size (); ++i)
  {
    (*cloudA)[i].x =  rand () / (RAND_MAX + 1.0f);
    (*cloudA)[i].y =  rand () / (RAND_MAX + 1.0f);
    (*cloudA)[i].z =  rand () / (RAND_MAX + 1.0f);
  }

  // Add points from cloudA to octree
  octree.setInputCloud (cloudA);
  octree.addPointsFromInputCloud ();

  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  octree.switchBuffers ();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ> );
   
  // Generate pointcloud data for cloudB 
  cloudB->width = 10000;
  cloudB->height = 1;
  cloudB->points.resize (cloudB->width * cloudB->height);

  for (std::size_t i = 0; i < cloudB->size (); ++i)
  {
    (*cloudB)[i].x = 2 * rand () / (RAND_MAX + 1.0f);
    (*cloudB)[i].y = 2 * rand () / (RAND_MAX + 1.0f);
    (*cloudB)[i].z =  rand () / (RAND_MAX + 1.0f);
  }

  // Add points from cloudB to octree
  octree.setInputCloud (cloudB);
  octree.addPointsFromInputCloud ();

  std::vector<int> newPointIdxVector;

  // Get vector of point indices from octree voxels which did not exist in previous buffer
  octree.getPointIndicesFromNewVoxels (newPointIdxVector);

  // Output points
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudC (new pcl::PointCloud<pcl::PointXYZ> );

  std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
  for (std::size_t i = 0; i < newPointIdxVector.size (); ++i)
  {
    cloudC -> push_back((*cloudB)[newPointIdxVector[i]]);
    std::cout << i << "# Index:" << newPointIdxVector[i]
              << "  Point:" << (*cloudB)[newPointIdxVector[i]].x << " "
              << (*cloudB)[newPointIdxVector[i]].y << " "
              << (*cloudB)[newPointIdxVector[i]].z << std::endl;
  }

  //viewer
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud (cloudA, "cloudA");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "cloudA");

  pcl::visualization::PCLVisualizer::Ptr viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer 2"));
  viewer2->setBackgroundColor (0, 0, 0);
  viewer2->addPointCloud (cloudB, "cloudB");
  viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "cloudB");

  pcl::visualization::PCLVisualizer::Ptr viewer3 (new pcl::visualization::PCLVisualizer ("3D Viewer 3"));
  viewer3->setBackgroundColor (0, 0, 0);
  viewer3->addPointCloud (cloudC, "cloudC");
  viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "cloudC");

  while (!viewer->wasStopped () ||  !viewer->wasStopped () )
  {
    viewer->spinOnce (100);
    viewer2->spinOnce (100);
  }
  return 0;
}

