#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/mls.h>

ros::Publisher pub_output_pcl2;
ros::Publisher pub_output_pcl;
std::string pcl2_output;
std::string pcl_output;
std::string pcl2_input;
std::string pcl_type;

float resolution;

bool comparePoint_XYZ(pcl::PointXYZ p1, pcl::PointXYZ p2){
if (p1.x != p2.x)
    return p1.x > p2.x;
else if (p1.y != p2.y)
    return  p1.y > p2.y;
else
    return p1.z > p2.z;
}

bool comparePoint_XYZRGB(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2){
if (p1.x != p2.x)
    return p1.x > p2.x;
else if (p1.y != p2.y)
    return  p1.y > p2.y;
else
    return p1.z > p2.z;
}

bool equalPoint_XYZ(pcl::PointXYZ p1, pcl::PointXYZ p2){
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

bool equalPoint_XYZRGB(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2){
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

bool nanPoint_XYZ(pcl::PointXYZ p){
  if (std::isnan(p.x)){
    return true;
  }
  return false;
}

bool nanPoint_XYZRGB(pcl::PointXYZRGB p){
  if (std::isnan(p.x)){
    return true;
  }
  return false;
}

void remove_point_cloud_duplicates(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl){
  //remove duplicate points in point cloud
  //std::cerr << "number of points before: " << input_pcl->points.size() << std::endl;

  input_pcl->height = 1;

  pcl::PointCloud<pcl::PointXYZ>::iterator nan_end = std::remove_if(input_pcl->points.begin(), input_pcl->points.end(), nanPoint_XYZ);
  input_pcl->points.erase(nan_end, input_pcl->points.end());

  //std::cerr << "number of points after removing nan values: " << input_pcl->points.size() << std::endl;
  input_pcl->width = input_pcl->size ();

  std::sort(input_pcl->points.begin(), input_pcl->points.end(), comparePoint_XYZ);

  pcl::PointCloud<pcl::PointXYZ>::iterator unique_end = std::unique(input_pcl->points.begin(), input_pcl->points.end(), equalPoint_XYZ);
  input_pcl->points.erase(unique_end, input_pcl->points.end());

  //std::cerr << "number of points after removing duplicates: " << input_pcl->points.size() << std::endl;
  input_pcl->width = input_pcl->size ();
}

void remove_point_cloud_duplicates(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl){
  //remove duplicate points in point cloud
  //std::cerr << "number of points before: " << input_pcl->points.size() << std::endl;

  input_pcl->height = 1;

  pcl::PointCloud<pcl::PointXYZRGB>::iterator nan_end = std::remove_if(input_pcl->points.begin(), input_pcl->points.end(), nanPoint_XYZRGB);
  input_pcl->points.erase(nan_end, input_pcl->points.end());

  //std::cerr << "number of points after removing nan values: " << input_pcl->points.size() << std::endl;
  input_pcl->width = input_pcl->size ();

  std::sort(input_pcl->points.begin(), input_pcl->points.end(), comparePoint_XYZRGB);

  pcl::PointCloud<pcl::PointXYZRGB>::iterator unique_end = std::unique(input_pcl->points.begin(), input_pcl->points.end(), equalPoint_XYZRGB);
  input_pcl->points.erase(unique_end, input_pcl->points.end());

  //std::cerr << "number of points after removing duplicates: " << input_pcl->points.size() << std::endl;
  input_pcl->width = input_pcl->size ();
}

pcl::PCLPointCloud2::Ptr voxel_filter_point_cloud_XYZ(pcl::PCLPointCloud2::Ptr input_pcl,float resolution){
  pcl::PCLPointCloud2::Ptr filtered_pcl2(new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZ> ());
    //Display cloud size before filtering
    //std::cerr << "PointCloud before filtering: " << input_pcl->width * input_pcl->height
    //          << " data points (" << pcl::getFieldsList (*input_pcl) << ")." << std::endl;
  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (input_pcl);
  sor.setLeafSize (resolution, resolution, resolution);
  sor.filter (*filtered_pcl2);

  pcl::fromPCLPointCloud2(*filtered_pcl2,*filtered_pcl);
  remove_point_cloud_duplicates(filtered_pcl);
  pcl::toPCLPointCloud2(*filtered_pcl,*filtered_pcl2);

  //Display cloud size after filtering
  //std::cerr << "PointCloud filtered" << std::endl;
  //std::cerr << "PointCloud after filtering: " << filtered_pcl2->width * filtered_pcl2->height
  //          << " data points (" << pcl::getFieldsList (*filtered_pcl2) << ")." << std::endl;

  return filtered_pcl2;
}

pcl::PCLPointCloud2::Ptr voxel_filter_point_cloud_XYZRGB(pcl::PCLPointCloud2::Ptr input_pcl,float resolution){
  pcl::PCLPointCloud2::Ptr filtered_pcl2(new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZRGB> ());
    //Display cloud size before filtering
    //std::cerr << "PointCloud before filtering: " << input_pcl->width * input_pcl->height
    //          << " data points (" << pcl::getFieldsList (*input_pcl) << ")." << std::endl;
  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (input_pcl);
  sor.setLeafSize (resolution, resolution, resolution);
  sor.filter (*filtered_pcl2);

  pcl::fromPCLPointCloud2(*filtered_pcl2,*filtered_pcl);
  remove_point_cloud_duplicates(filtered_pcl);
  pcl::toPCLPointCloud2(*filtered_pcl,*filtered_pcl2);

  //Display cloud size after filtering
  //std::cerr << "PointCloud filtered" << std::endl;
  //std::cerr << "PointCloud after filtering: " << filtered_pcl2->width * filtered_pcl2->height
  //          << " data points (" << pcl::getFieldsList (*filtered_pcl2) << ")." << std::endl;

  return filtered_pcl2;
}

pcl::PCLPointCloud2::Ptr filter_to_grid_XYZ(pcl::PCLPointCloud2::Ptr input_pcl2,float resolution){
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCLPointCloud2::Ptr filtered_pcl2(new pcl::PCLPointCloud2 ());
  pcl::fromPCLPointCloud2(*input_pcl2,*input_pcl);
  filtered_pcl->width = input_pcl->width;
  filtered_pcl->height = input_pcl->height;
  filtered_pcl->points.resize(filtered_pcl->width * filtered_pcl->height);

  //Display cloud size before filtering
  //std::cerr << "PointCloud before filtering: " << input_pcl2->width * input_pcl2->height
  //          << " data points (" << pcl::getFieldsList (*input_pcl2) << ")." << std::endl;

  for (size_t i = 0; i < input_pcl->points.size (); ++i)
    {
      double x = round(input_pcl->points[i].x/resolution)*resolution;
      double y = round(input_pcl->points[i].y/resolution)*resolution;
      double z = round(input_pcl->points[i].z/resolution)*resolution;
      filtered_pcl->points[i].x = x;
      filtered_pcl->points[i].y = y;
      filtered_pcl->points[i].z = z;
    }

  remove_point_cloud_duplicates(filtered_pcl);

  pcl::toPCLPointCloud2(*filtered_pcl,*filtered_pcl2);
  filtered_pcl2->header = input_pcl2->header;

  //Display cloud size after filtering
  //std::cerr << "PointCloud filtered" << std::endl;
  //std::cerr << "PointCloud after filtering: " << filtered_pcl2->width * filtered_pcl2->height
  //          << " data points (" << pcl::getFieldsList (*filtered_pcl2) << ")." << std::endl;

  return(filtered_pcl2);
}

pcl::PCLPointCloud2::Ptr filter_to_grid_XYZRGB(pcl::PCLPointCloud2::Ptr input_pcl2,float resolution){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCLPointCloud2::Ptr filtered_pcl2(new pcl::PCLPointCloud2 ());
  pcl::fromPCLPointCloud2(*input_pcl2,*input_pcl);
  filtered_pcl->width = input_pcl->width;
  filtered_pcl->height = input_pcl->height;
  filtered_pcl->points.resize(filtered_pcl->width * filtered_pcl->height);

  //Display cloud size before filtering
  //std::cerr << "PointCloud before filtering: " << input_pcl2->width * input_pcl2->height
  //          << " data points (" << pcl::getFieldsList (*input_pcl2) << ")." << std::endl;

  for (size_t i = 0; i < input_pcl->points.size (); ++i)
    {
      double x = round(input_pcl->points[i].x/resolution)*resolution;
      double y = round(input_pcl->points[i].y/resolution)*resolution;
      double z = round(input_pcl->points[i].z/resolution)*resolution;
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

  remove_point_cloud_duplicates(filtered_pcl);

  pcl::toPCLPointCloud2(*filtered_pcl,*filtered_pcl2);
  filtered_pcl2->header = input_pcl2->header;

  //Display cloud size after filtering
  //std::cerr << "PointCloud filtered" << std::endl;
  //std::cerr << "PointCloud after filtering: " << filtered_pcl2->width * filtered_pcl2->height
  //          << " data points (" << pcl::getFieldsList (*filtered_pcl2) << ")." << std::endl;

  return(filtered_pcl2);
}

pcl::PCLPointCloud2::Ptr scale_pcl_XYZ(pcl::PCLPointCloud2::Ptr input_pcl2, float scaleAmount){
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr scaled_pcl(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCLPointCloud2::Ptr scaled_pcl2(new pcl::PCLPointCloud2 ());
  pcl::fromPCLPointCloud2(*input_pcl2,*input_pcl);
  scaled_pcl->width = input_pcl->width;
  scaled_pcl->height = input_pcl->height;
  scaled_pcl->points.resize(scaled_pcl->width * scaled_pcl->height);

  for (size_t i = 0; i < input_pcl->points.size (); ++i)
    {
      double x = input_pcl->points[i].x*scaleAmount;
      double y = input_pcl->points[i].y*scaleAmount;
      double z = input_pcl->points[i].z*scaleAmount;
      scaled_pcl->points[i].x = x;
      scaled_pcl->points[i].y = y;
      scaled_pcl->points[i].z = z;
    }

  remove_point_cloud_duplicates(scaled_pcl);

  pcl::toPCLPointCloud2(*scaled_pcl,*scaled_pcl2);
  scaled_pcl2->header = input_pcl2->header;

  return(scaled_pcl2);
}

pcl::PCLPointCloud2::Ptr scale_pcl_XYZRGB(pcl::PCLPointCloud2::Ptr input_pcl2, float scaleAmount){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scaled_pcl(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCLPointCloud2::Ptr scaled_pcl2(new pcl::PCLPointCloud2 ());
  pcl::fromPCLPointCloud2(*input_pcl2,*input_pcl);
  scaled_pcl->width = input_pcl->width;
  scaled_pcl->height = input_pcl->height;
  scaled_pcl->points.resize(scaled_pcl->width * scaled_pcl->height);

  for (size_t i = 0; i < input_pcl->points.size (); ++i)
    {
      double x = input_pcl->points[i].x*scaleAmount;
      double y = input_pcl->points[i].y*scaleAmount;
      double z = input_pcl->points[i].z*scaleAmount;
      scaled_pcl->points[i].x = x;
      scaled_pcl->points[i].y = y;
      scaled_pcl->points[i].z = z;
    }

  remove_point_cloud_duplicates(scaled_pcl);

  pcl::toPCLPointCloud2(*scaled_pcl,*scaled_pcl2);
  scaled_pcl2->header = input_pcl2->header;

  return(scaled_pcl2);
}

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZ> input_pcl){
  sensor_msgs::PointCloud2 pcl2_msg;
  sensor_msgs::PointCloud pcl_msg;
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl,pcl2_msg);

  //Publish ROS msg
  pub_output_pcl2.publish(pcl2_msg);

  sensor_msgs::convertPointCloud2ToPointCloud(pcl2_msg,pcl_msg);
  pub_output_pcl.publish(pcl_msg);
    //std::cerr << "Published point cloud." << std::endl;
}

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> input_pcl){
  sensor_msgs::PointCloud2 pcl2_msg;
  sensor_msgs::PointCloud pcl_msg;
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl,pcl2_msg);

  //Publish ROS msg
  pub_output_pcl2.publish(pcl2_msg);

  sensor_msgs::convertPointCloud2ToPointCloud(pcl2_msg,pcl_msg);
  pub_output_pcl.publish(pcl_msg);
    //std::cerr << "Published point cloud." << std::endl;
}

void save_point_cloud (std::string save_filepath, pcl::PCLPointCloud2::Ptr pcl_to_save){
  //std::cerr << "Saving to..." << save_filepath << std::endl;
  pcl::PCDWriter writer;
  writer.write (save_filepath, *pcl_to_save,
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
}

pcl::PCLPointCloud2::Ptr load_point_cloud(std::string pcl_filepath){
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  // Fill in the cloud data
  pcl::PCDReader reader;

  //std::cerr << "Reading point cloud..." << std::endl;
  reader.read (pcl_filepath, *cloud);
  return(cloud);
}

void input_cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_in_msg){
    pcl::PCLPointCloud2::Ptr input_pcl2(new pcl::PCLPointCloud2 ()); //Converted point cloud
    pcl::PCLPointCloud2::Ptr filtered_pcl2(new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr output_pcl2(new pcl::PCLPointCloud2 ());

    ros::Time timestamp = cloud_in_msg->header.stamp;
    //std::cerr << "msg received @ " << timestamp << std::endl;

    bool isRGB = false;
    for (int i = 0; i < cloud_in_msg->fields.size(); i++){
      if (cloud_in_msg->fields[i].name == "rgb"){
        isRGB = true;
        break;
      }
    }

    if (isRGB){
      pcl::PointCloud<pcl::PointXYZRGB> input_pcl; //Point cloud from ROS msg
      pcl::PointCloud<pcl::PointXYZRGB> output_pcl;

      //Read point cloud from ROS msg
      pcl::fromROSMsg(*cloud_in_msg,input_pcl);

      //Convert to PCL point cloud 2
      pcl::toPCLPointCloud2(input_pcl,*input_pcl2);

      //cloud_filtered_pcl = voxel_filter_point_cloud(cloud_input_pcl,resolution);
      filtered_pcl2 = filter_to_grid_XYZRGB(input_pcl2,resolution);

      output_pcl2 = filtered_pcl2;
      //Convert to Point Cloud for ROS msg
      pcl::fromPCLPointCloud2(*output_pcl2,output_pcl);

      publish_point_cloud(output_pcl);
    } else {
      pcl::PointCloud<pcl::PointXYZ> input_pcl; //Point cloud from ROS msg
      pcl::PointCloud<pcl::PointXYZ> output_pcl;

      //Read point cloud from ROS msg
      pcl::fromROSMsg(*cloud_in_msg,input_pcl);

      //Convert to PCL point cloud 2
      pcl::toPCLPointCloud2(input_pcl,*input_pcl2);

      //cloud_filtered_pcl = voxel_filter_point_cloud(cloud_input_pcl,resolution);
      filtered_pcl2 = filter_to_grid_XYZ(input_pcl2,resolution);

      output_pcl2 = filtered_pcl2;
      //Convert to Point Cloud for ROS msg
      pcl::fromPCLPointCloud2(*output_pcl2,output_pcl);

      publish_point_cloud(output_pcl);
    }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_reduce");
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  p_nh.getParam("resolution",resolution);
  p_nh.getParam("pcl2_input",pcl2_input);
  p_nh.getParam("pcl_output",pcl_output);
  p_nh.getParam("pcl2_output",pcl2_output);

  ros::Subscriber sub = nh.subscribe (pcl2_input, 1, input_cloud_callback);

  pub_output_pcl2 = nh.advertise<sensor_msgs::PointCloud2>(pcl2_output, 1);
  pub_output_pcl = nh.advertise<sensor_msgs::PointCloud>(pcl_output, 1);

  ros::spin();
}
