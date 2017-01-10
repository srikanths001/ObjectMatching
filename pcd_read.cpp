#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;

float model_ss_ (7.5f);
std::string model_filename_;
void
parseCommandLine (int argc, char *argv[])
{
  //Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  model_filename_ = argv[filenames[0]];
  //printf(" Filename is %s ", model_filename_);
}

double
computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr filtered_cloud (new pcl::PointCloud<PointType> ());

  parseCommandLine (argc, argv);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (model_filename_, *cloud) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from " << model_filename_ << " with the following fields: "<<cloud->width <<"Height: " <<cloud->height
            << std::endl;

  //
  //  Set up resolution invariance
  //
  /*if (1)
  {
    float resolution = static_cast<float> (computeCloudResolution (cloud));
    printf("Model Res : %.3f\n", model_ss_);
    if (resolution != 0.0f)
    {
      model_ss_   *= resolution;
    }
    printf("AAAAAAAAAAAAAAAA\n");
    std::cout << "Model resolution:       " << resolution << std::endl;
    std::cout << "Model sampling size:    " << model_ss_ << std::endl;
  }

   //float center = (cloud->width * cloud->height)/2;
   float center = 232972;
   //float center_y = cloud->height/2;
   std::cout << "Center point: " << std::endl;
   std::cout << "    " << cloud->points[center].x
              << " "    << cloud->points[center].y
              << " "    << cloud->points[center].z << std::endl;*/

  //
  //  Downsample Clouds to Extract keypoints
  //

  /*pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (cloud);
  uniform_sampling.setRadiusSearch (model_ss_);
  //uniform_sampling.filter (*model_keypoints);
  pcl::PointCloud<int> keypointIndices1;
  uniform_sampling.compute(keypointIndices1);
  pcl::copyPointCloud(*cloud, keypointIndices1.points, *model_keypoints);
  std::cout << "Model total points: " << cloud->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;*/

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  //pass.setInputCloud (model_keypoints);
  pass.setInputCloud (cloud);
  //pass.setFilterFieldName ("x");
  //pass.setFilterLimits (0.0, 0.49);
  //pass.setFilterFieldName ("y");
  //pass.setFilterLimits (-0.13, 0.13);
  //pass.filter (*filtered_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 4.49);
  pass.filter (*filtered_cloud);

  pcl::io::savePCDFileASCII<pcl::PointXYZ> ("filtered_cloud_scene.pcd", *filtered_cloud);
  //center = (filtered_cloud->width * filtered_cloud->height)/2;
  std::cout << "cloud after downsampling: " << std::endl;
   std::cout << "    " << filtered_cloud->width
              << " "    << filtered_cloud->height << std::endl;
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (filtered_cloud);
  while (!viewer.wasStopped ())
   {
   }

  /*for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;*/

  return (0);
}
