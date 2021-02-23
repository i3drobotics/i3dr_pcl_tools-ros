#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <i3dr_pcl_tools/pause_map.h>
#include <i3dr_pcl_tools/resume_map.h>
#include <i3dr_pcl_tools/reset_map.h>
#include <i3dr_pcl_tools/save_map.h>
#include <i3dr_pcl_tools/single_step_map.h>
#include <i3dr_pcl_tools/set_map_resolution.h>
#include <math.h>
#include <mutex>

std::string input_point_cloud2_node;
std::string output_point_cloud2_node;
std::string output_point_cloud_node;
ros::Publisher pub_full_map_pcl2;
ros::Publisher pub_aligned_pcl2;
ros::Time pcl_timestamp;
pcl::PointCloud<pcl::PointXYZ>::Ptr full_map_pcl_XYZ;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_map_pcl_XYZRGB;
pcl::PointCloud<pcl::PointXYZ>::Ptr orig_map_pcl_XYZ;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr orig_map_pcl_XYZRGB;
double visual_resolution,stored_resolution;
bool pause_mapping;
bool single_step;
std::mutex pause_mutex;
std::mutex resolution_mutex;

bool comparePoint_XYZ(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
  if (p1.x != p2.x)
    return p1.x > p2.x;
  else if (p1.y != p2.y)
    return p1.y > p2.y;
  else
    return p1.z > p2.z;
}

bool comparePoint_XYZRGB(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2)
{
  if (p1.x != p2.x)
    return p1.x > p2.x;
  else if (p1.y != p2.y)
    return p1.y > p2.y;
  else
    return p1.z > p2.z;
}

bool equalPoint_XYZ(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
  double diffX = fabs(p1.x - p2.x);
  double diffY = fabs(p1.y - p2.y);
  double diffZ = fabs(p1.z - p2.z);
  //if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
  //     return true;
  double threshold = 0.00001;
  if (diffX <= threshold && diffY <= threshold && diffZ <= threshold)
    return true;
  return false;
}

bool equalPoint_XYZRGB(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2)
{
  double diffX = fabs(p1.x - p2.x);
  double diffY = fabs(p1.y - p2.y);
  double diffZ = fabs(p1.z - p2.z);
  //if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
  //     return true;
  double threshold = 0.00001;
  if (diffX <= threshold && diffY <= threshold && diffZ <= threshold)
    return true;
  return false;
}

bool nanPoint_XYZ(pcl::PointXYZ p)
{
  //std::cerr << p.x << std::endl;
  if (std::isnan(p.x))
  {
    //std::cerr << p.x << std::endl;
    return true;
  }
  return false;
}

bool nanPoint_XYZRGB(pcl::PointXYZRGB p)
{
  //std::cerr << p.x << std::endl;
  if (std::isnan(p.x))
  {
    //std::cerr << p.x << std::endl;
    return true;
  }
  return false;
}

void print_points(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl){
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = input_pcl->begin(); it != input_pcl->end(); it++)
    {
      std::cerr << it->x << ", " << it->y << ", " << it->z << std::endl;
    }
}

void print_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl){
  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = input_pcl->begin(); it != input_pcl->end(); it++)
    {
      std::cerr << it->x << ", " << it->y << ", " << it->z << std::endl;
    }
}

void remove_nan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
  cloud_in->height = 1;

  pcl::PointCloud<pcl::PointXYZ>::iterator nan_end = std::remove_if(cloud_in->points.begin(), cloud_in->points.end(), nanPoint_XYZ);
  cloud_in->points.erase(nan_end, cloud_in->points.end());

  cloud_in->width = cloud_in->size();
}

void remove_nan(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
  cloud_in->height = 1;

  pcl::PointCloud<pcl::PointXYZRGB>::iterator nan_end = std::remove_if(cloud_in->points.begin(), cloud_in->points.end(), nanPoint_XYZRGB);
  cloud_in->points.erase(nan_end, cloud_in->points.end());

  cloud_in->width = cloud_in->size();
}

void remove_point_cloud_duplicates(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl)
{
  //remove duplicate points in point cloud
  std::cerr << "number of points before: " << input_pcl->points.size() << std::endl;

  input_pcl->height = 1;

  pcl::PointCloud<pcl::PointXYZ>::iterator nan_end = std::remove_if(input_pcl->points.begin(), input_pcl->points.end(), nanPoint_XYZ);
  input_pcl->points.erase(nan_end, input_pcl->points.end());

  std::cerr << "number of points after removing nan values: " << input_pcl->points.size() << std::endl;
  input_pcl->width = input_pcl->size();

  std::sort(input_pcl->points.begin(), input_pcl->points.end(), comparePoint_XYZ);

  pcl::PointCloud<pcl::PointXYZ>::iterator unique_end = std::unique(input_pcl->points.begin(), input_pcl->points.end(), equalPoint_XYZ);
  input_pcl->points.erase(unique_end, input_pcl->points.end());

  std::cerr << "number of points after removing duplicates: " << input_pcl->points.size() << std::endl;
  input_pcl->width = input_pcl->size();
}

void remove_point_cloud_duplicates(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl)
{
  //remove duplicate points in point cloud
  //std::cerr << "number of points before: " << input_pcl->points.size() << std::endl;

  input_pcl->height = 1;

  pcl::PointCloud<pcl::PointXYZRGB>::iterator nan_end = std::remove_if(input_pcl->points.begin(), input_pcl->points.end(), nanPoint_XYZRGB);
  input_pcl->points.erase(nan_end, input_pcl->points.end());

  //std::cerr << "number of points after removing nan values: " << input_pcl->points.size() << std::endl;
  input_pcl->width = input_pcl->size();

  std::sort(input_pcl->points.begin(), input_pcl->points.end(), comparePoint_XYZRGB);

  pcl::PointCloud<pcl::PointXYZRGB>::iterator unique_end = std::unique(input_pcl->points.begin(), input_pcl->points.end(), equalPoint_XYZRGB);
  input_pcl->points.erase(unique_end, input_pcl->points.end());

  //std::cerr << "number of points after removing duplicates: " << input_pcl->points.size() << std::endl;
  input_pcl->width = input_pcl->size();
}

pcl::PCLPointCloud2::Ptr filter_voxel_grid_XYZ(pcl::PCLPointCloud2::Ptr input_pcl2, float resolution)
{;
  pcl::PCLPointCloud2::Ptr filtered_pcl2(new pcl::PCLPointCloud2());

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (input_pcl2);
  sor.setLeafSize (resolution, resolution, resolution);
  sor.filter (*filtered_pcl2);

  filtered_pcl2->header = input_pcl2->header;

  return (filtered_pcl2);
}

pcl::PCLPointCloud2::Ptr filter_voxel_grid_XYZRGB(pcl::PCLPointCloud2::Ptr input_pcl2, float resolution)
{
  pcl::PCLPointCloud2::Ptr filtered_pcl2(new pcl::PCLPointCloud2());

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (input_pcl2);
  sor.setLeafSize (resolution, resolution, resolution);
  sor.filter (*filtered_pcl2);

  filtered_pcl2->header = input_pcl2->header;

  return (filtered_pcl2);
}

pcl::PCLPointCloud2::Ptr filter_to_grid_XYZ(pcl::PCLPointCloud2::Ptr input_pcl2, float resolution)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCLPointCloud2::Ptr filtered_pcl2(new pcl::PCLPointCloud2());
  pcl::fromPCLPointCloud2(*input_pcl2, *input_pcl);
  filtered_pcl->width = input_pcl->width;
  filtered_pcl->height = input_pcl->height;
  filtered_pcl->points.resize(filtered_pcl->width * filtered_pcl->height);

  //Display cloud size before filtering
  //std::cerr << "PointCloud before filtering: " << input_pcl2->width * input_pcl2->height
  //          << " data points (" << pcl::getFieldsList(*input_pcl2) << ")." << std::endl;

  for (size_t i = 0; i < input_pcl->points.size(); ++i)
  {
    double x = round(input_pcl->points[i].x / resolution) * resolution;
    double y = round(input_pcl->points[i].y / resolution) * resolution;
    double z = round(input_pcl->points[i].z / resolution) * resolution;
    filtered_pcl->points[i].x = x;
    filtered_pcl->points[i].y = y;
    filtered_pcl->points[i].z = z;
  }

  remove_point_cloud_duplicates(filtered_pcl);

  pcl::toPCLPointCloud2(*filtered_pcl, *filtered_pcl2);
  filtered_pcl2->header = input_pcl2->header;

  //Display cloud size after filtering
  //std::cerr << "PointCloud filtered" << std::endl;
  //std::cerr << "PointCloud after filtering: " << filtered_pcl2->width * filtered_pcl2->height
  //          << " data points (" << pcl::getFieldsList(*filtered_pcl2) << ")." << std::endl;

  return (filtered_pcl2);
}

pcl::PCLPointCloud2::Ptr filter_to_grid_XYZRGB(pcl::PCLPointCloud2::Ptr input_pcl2, float resolution)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PCLPointCloud2::Ptr filtered_pcl2(new pcl::PCLPointCloud2());
  pcl::fromPCLPointCloud2(*input_pcl2, *input_pcl);
  filtered_pcl->width = input_pcl->width;
  filtered_pcl->height = input_pcl->height;
  filtered_pcl->points.resize(filtered_pcl->width * filtered_pcl->height);

  //Display cloud size before filtering
  //std::cerr << "PointCloud before filtering: " << input_pcl2->width * input_pcl2->height
  //          << " data points (" << pcl::getFieldsList(*input_pcl2) << ")." << std::endl;

  for (size_t i = 0; i < input_pcl->points.size(); ++i)
  {
    double x = round(input_pcl->points[i].x / resolution) * resolution;
    double y = round(input_pcl->points[i].y / resolution) * resolution;
    double z = round(input_pcl->points[i].z / resolution) * resolution;
    double r = input_pcl->points[i].r;
    double g = input_pcl->points[i].g;
    double b = input_pcl->points[i].b;
    filtered_pcl->points[i].x = x;
    filtered_pcl->points[i].y = y;
    filtered_pcl->points[i].z = z;
    filtered_pcl->points[i].r = r;
    filtered_pcl->points[i].g = g;
    filtered_pcl->points[i].b = b;
  }

  //print_points(filtered_pcl);

  remove_point_cloud_duplicates(filtered_pcl);

  pcl::toPCLPointCloud2(*filtered_pcl, *filtered_pcl2);
  filtered_pcl2->header = input_pcl2->header;

  //Display cloud size after filtering
  //std::cerr << "PointCloud filtered" << std::endl;
  //std::cerr << "PointCloud after filtering: " << filtered_pcl2->width * filtered_pcl2->height
  //          << " data points (" << pcl::getFieldsList(*filtered_pcl2) << ")." << std::endl;

  return (filtered_pcl2);
}

bool pause_map(i3dr_pcl_tools::pause_map::Request &req,
               i3dr_pcl_tools::pause_map::Response &res)
{
  ROS_INFO("Map paused");
  pause_mutex.lock();
  pause_mapping = true;
  pause_mutex.unlock();
  return true;
}

bool resume_map(i3dr_pcl_tools::resume_map::Request &req,
               i3dr_pcl_tools::resume_map::Response &res)
{
  ROS_INFO("Map resumed");
  pause_mutex.lock();
  pause_mapping = false;
  pause_mutex.unlock();
  return true;
}

bool single_step_map(i3dr_pcl_tools::single_step_map::Request &req,
               i3dr_pcl_tools::single_step_map::Response &res)
{
  ROS_INFO("Map single step");
  pause_mutex.lock();
  single_step = true;
  pause_mutex.unlock();
  return true;
}

bool reset_map(i3dr_pcl_tools::reset_map::Request &req,
               i3dr_pcl_tools::reset_map::Response &res)
{
  ROS_INFO("Reset Global Map");
  full_map_pcl_XYZ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  full_map_pcl_XYZRGB = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  orig_map_pcl_XYZ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  orig_map_pcl_XYZRGB = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  return true;
}

bool save_map(i3dr_pcl_tools::save_map::Request &req,
              i3dr_pcl_tools::save_map::Response &res)
{
  //pcl::PCLPointCloud2::Ptr filtered_pcl2(new pcl::PCLPointCloud2());
  //pcl::PCLPointCloud2::Ptr input_pcl2(new pcl::PCLPointCloud2());
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);

  //ROS_INFO("Filtering points to resolution...");
  //pcl::toPCLPointCloud2(*orig_map_pcl_XYZRGB, *input_pcl2);
  //filtered_pcl2 = filter_voxel_grid_XYZRGB(input_pcl2, req.resolution);
  //pcl::fromPCLPointCloud2(*filtered_pcl2, *filtered_pcl);
  //save point cloud to file
  ROS_INFO("saving point cloud to file: '%s'", req.filepath.c_str());
  pcl::io::savePLYFileBinary(req.filepath, *orig_map_pcl_XYZRGB);
  res.res = "Point cloud saved to file: " + req.filepath;
  ROS_INFO("%s", res.res.c_str());
  return true;
}

bool set_map_resolution(i3dr_pcl_tools::set_map_resolution::Request &req,
              i3dr_pcl_tools::set_map_resolution::Response &res)
{
  resolution_mutex.lock();
  visual_resolution = req.visual;
  stored_resolution = req.stored;
  resolution_mutex.unlock();
  res.res = "Map resolution adjusted";
  return true;
}

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZ> input_pcl)
{
  sensor_msgs::PointCloud2 pcl2_msg;
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl, pcl2_msg);

  //Publish ROS msg
  pub_full_map_pcl2.publish(pcl2_msg);
  //std::cerr << "Published point cloud." << std::endl;
}

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> input_pcl)
{
  sensor_msgs::PointCloud2 pcl2_msg;
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl, pcl2_msg);

  //Publish ROS msg
  pub_full_map_pcl2.publish(pcl2_msg);
  //std::cerr << "Published rgb point cloud map." << std::endl;
}

void add_pcl_to_map(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl, pcl::PointCloud<pcl::PointXYZ>::Ptr map_pcl, bool remove_duplicates)
{
  pcl::PCLPointCloud2::Ptr full_map_pcl2(new pcl::PCLPointCloud2());
  pcl::PointCloud<pcl::PointXYZ>::Ptr full_map_filtered_pcl(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCLPointCloud2::Ptr full_map_filtered_pcl2(new pcl::PCLPointCloud2());

  map_pcl->operator+=(*input_pcl);

  if (remove_duplicates){
    remove_point_cloud_duplicates(map_pcl);
  }

  map_pcl->header = input_pcl->header;
  map_pcl->width = map_pcl->size();
  map_pcl->height = 1;
}

void add_pcl_to_map(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_pcl, bool remove_duplicates)
{

  pcl::PCLPointCloud2::Ptr full_map_pcl2(new pcl::PCLPointCloud2());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_map_filtered_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PCLPointCloud2::Ptr full_map_filtered_pcl2(new pcl::PCLPointCloud2());

  map_pcl->operator+=(*input_pcl);

  if (remove_duplicates){
    remove_point_cloud_duplicates(map_pcl);
  }
  
  map_pcl->header = input_pcl->header;
  map_pcl->width = map_pcl->size();
  map_pcl->height = 1;
}

void icp_convergence(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl, double resolution)
{
  double fit_score = 0.1 * 10;
  if (full_map_pcl_XYZRGB->size() > 0)
  {
    std::cout << "Point cloud input size: " << input_pcl->points.size() << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PCLPointCloud2::Ptr aligned_pcl2(new pcl::PCLPointCloud2());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_filtered_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PCLPointCloud2::Ptr aligned_filtered_pcl2(new pcl::PCLPointCloud2());

    int attempts = 3;

    for (int i = 0; i < attempts; i++)
    {
      fit_score = fit_score / 10;
      std::cerr << "ICP Attempt: " << i + 1 << "/" << attempts << " score maximum: " << fit_score << std::endl;
      icp.setInputSource(input_pcl);
      icp.setInputTarget(full_map_pcl_XYZRGB);
      //icp.setMaxCorrespondenceDistance (resolution);
      icp.align(*aligned_pcl);
      std::cerr << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();
      std::cerr << icp.getFinalTransformation() << std::endl;
      if (icp.hasConverged() && icp.getFitnessScore() < fit_score)
      {
        std::cerr << " ...adding to map" << std::endl;
        pcl::toPCLPointCloud2(*aligned_pcl, *aligned_pcl2);
        aligned_filtered_pcl2 = filter_to_grid_XYZRGB(aligned_pcl2, resolution);
        pcl::fromPCLPointCloud2(*aligned_filtered_pcl2, *aligned_filtered_pcl);
        full_map_pcl_XYZRGB->operator+=(*aligned_filtered_pcl);
        remove_point_cloud_duplicates(full_map_pcl_XYZRGB);
        break;
      }
      else
      {
        std::cerr << " ...fit score too high" << std::endl;
      }
    }
  }
  else
  {
    full_map_pcl_XYZRGB->points = input_pcl->points;
  }
  full_map_pcl_XYZRGB->header = input_pcl->header;
  full_map_pcl_XYZRGB->width = full_map_pcl_XYZRGB->size();
  full_map_pcl_XYZRGB->height = 1;
  pcl_conversions::toPCL(ros::Time::now(), full_map_pcl_XYZRGB->header.stamp);
  //std::cerr << "Stiched point cloud size: " << full_map_pcl->points.size () << std::endl;
}

void icp_convergence(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl, double resolution)
{
  double fit_score = 0.1 * 10;
  if (full_map_pcl_XYZ->size() > 0)
  {
    std::cout << "Point cloud input size: " << input_pcl->points.size() << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCLPointCloud2::Ptr aligned_pcl2(new pcl::PCLPointCloud2());
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_filtered_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCLPointCloud2::Ptr aligned_filtered_pcl2(new pcl::PCLPointCloud2());

    int attempts = 3;

    for (int i = 0; i < attempts; i++)
    {
      fit_score = fit_score / 10;
      std::cerr << "ICP Attempt: " << i + 1 << "/" << attempts << " score maximum: " << fit_score << std::endl;
      icp.setInputSource(input_pcl);
      icp.setInputTarget(full_map_pcl_XYZ);
      //icp.setMaxCorrespondenceDistance (resolution);
      icp.align(*aligned_pcl);
      std::cerr << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();
      std::cerr << icp.getFinalTransformation() << std::endl;
      if (icp.hasConverged() && icp.getFitnessScore() < fit_score)
      {
        std::cerr << " ...adding to map" << std::endl;
        pcl::toPCLPointCloud2(*aligned_pcl, *aligned_pcl2);
        aligned_filtered_pcl2 = filter_to_grid_XYZRGB(aligned_pcl2, resolution);
        pcl::fromPCLPointCloud2(*aligned_filtered_pcl2, *aligned_filtered_pcl);
        full_map_pcl_XYZ->operator+=(*aligned_filtered_pcl);
        remove_point_cloud_duplicates(full_map_pcl_XYZ);
        break;
      }
      else
      {
        std::cerr << " ...fit score too high" << std::endl;
      }
    }
  }
  else
  {
    full_map_pcl_XYZ->points = input_pcl->points;
  }
  full_map_pcl_XYZ->header = input_pcl->header;
  full_map_pcl_XYZ->width = full_map_pcl_XYZ->size();
  full_map_pcl_XYZ->height = 1;
  pcl_conversions::toPCL(ros::Time::now(), full_map_pcl_XYZ->header.stamp);
  //std::cerr << "Stiched point cloud size: " << full_map_pcl->points.size () << std::endl;
}

void input_cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_in_msg)
{
  pause_mutex.lock();
  if (!pause_mapping || single_step){
    single_step = false;
    pause_mutex.unlock();
    pcl_timestamp = cloud_in_msg->header.stamp;

    pcl::PCLPointCloud2::Ptr filtered_pcl2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr filtered2_pcl2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr input_pcl2(new pcl::PCLPointCloud2()); //Converted point cloud

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);    //Point cloud from ROS msg
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZRGB>); //Point cloud from ROS msg
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered2_pcl(new pcl::PointCloud<pcl::PointXYZRGB>); //Point cloud from ROS msg
    //Read point cloud from ROS msg
    pcl::fromROSMsg(*cloud_in_msg, *input_pcl);

    pcl::toPCLPointCloud2(*input_pcl, *input_pcl2);

    resolution_mutex.lock();
    double vis_res = visual_resolution;
    double stor_res = stored_resolution;
    resolution_mutex.unlock();
    filtered_pcl2 = filter_voxel_grid_XYZRGB(input_pcl2, vis_res);
    filtered2_pcl2 = filter_voxel_grid_XYZRGB(input_pcl2, stor_res);
    pcl::fromPCLPointCloud2(*filtered_pcl2, *filtered_pcl);
    pcl::fromPCLPointCloud2(*filtered2_pcl2, *filtered2_pcl);

    add_pcl_to_map(filtered_pcl,full_map_pcl_XYZRGB,true);
    add_pcl_to_map(filtered2_pcl,orig_map_pcl_XYZRGB,false);

    std::cerr << "Mozaic point cloud size: " << orig_map_pcl_XYZRGB->points.size() << std::endl;
    //std::cerr << "Mozaic reduced point cloud size: " << full_map_pcl_XYZRGB->points.size() << std::endl;
  } else {
    pause_mutex.unlock();
  }
  publish_point_cloud(*full_map_pcl_XYZRGB);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_map");
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  full_map_pcl_XYZ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  full_map_pcl_XYZRGB = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  orig_map_pcl_XYZ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  orig_map_pcl_XYZRGB = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  p_nh.getParam("visual_resolution", visual_resolution);
  p_nh.getParam("stored_resolution", stored_resolution);
  p_nh.getParam("cloud_input", input_point_cloud2_node);
  p_nh.getParam("cloud_output", output_point_cloud2_node);
  p_nh.getParam("pause_on_start", pause_mapping);

  ros::Subscriber sub = nh.subscribe(input_point_cloud2_node, 1, input_cloud_callback);
  pub_full_map_pcl2 = nh.advertise<sensor_msgs::PointCloud2>(output_point_cloud2_node, 1);

  ros::ServiceServer service_reset = nh.advertiseService("reset_map", reset_map);
  ros::ServiceServer service_save = nh.advertiseService("save_map", save_map);
  ros::ServiceServer service_resume = nh.advertiseService("resume_map", resume_map);
  ros::ServiceServer service_pause = nh.advertiseService("pause_map", pause_map);
  ros::ServiceServer service_single_step = nh.advertiseService("single_step_map", single_step_map);
  ros::ServiceServer service_resolution = nh.advertiseService("set_map_resolution", set_map_resolution);

  ros::spin();
}
