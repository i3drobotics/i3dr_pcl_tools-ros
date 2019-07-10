#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

std::string pcl_filepath,tf_filepath;
ros::Time timestamp;
bool include_tf;
std::string point_cloud_frame_id, world_frame_id;
bool spin_once;

void save_point_cloud (std::string save_filepath, pcl::PointCloud<pcl::PointXYZRGB> pcl_to_save){

  ROS_INFO("Saving point cloud... %s", save_filepath.c_str());
  pcl::io::savePLYFileASCII(save_filepath,pcl_to_save);
  ROS_INFO("Point cloud saved. %s", save_filepath.c_str());
}

void save_point_cloud (std::string save_filepath, pcl::PointCloud<pcl::PointXYZ> pcl_to_save){

  ROS_INFO("Saving point cloud... %s", save_filepath.c_str());
  pcl::io::savePLYFileASCII(save_filepath,pcl_to_save);
  ROS_INFO("Point cloud saved. %s", save_filepath.c_str());
}

void save_tf (std::string save_filepath,std::string frame_id,std::string world_frame_id){
  ROS_INFO("Saving TF... %s", save_filepath.c_str());
  //load transform from world to image frame
  tf::StampedTransform transform;
  tf::TransformListener listener;
  listener.waitForTransform(world_frame_id.c_str(), frame_id.c_str(), timestamp, ros::Duration(10.0));
  listener.lookupTransform(world_frame_id.c_str(),frame_id.c_str(),timestamp, transform);
  tf::Quaternion rot = transform.getRotation();
  tf::Vector3 tran = transform.getOrigin();

  //save transform to file
  std::ofstream tf_file;
  tf_file.open (save_filepath.c_str());
  tf_file << tran.x() << " " << tran.y() << " " << tran.z() << " " << rot.x() << " " << rot.y() << " " << rot.z() << " " << rot.w() << " " << "\n";
  tf_file.close();
  ROS_INFO("TF saved. %s", save_filepath.c_str());
}

void input_cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_in_msg){
  timestamp = cloud_in_msg->header.stamp;

  bool isRGB = false;
  for (int i = 0; i < cloud_in_msg->fields.size(); i++){
    if (cloud_in_msg->fields[i].name == "rgb"){
      isRGB = true;
      break;
    }
  }

  if (isRGB){
    pcl::PointCloud<pcl::PointXYZRGB> input_pcl; //Point cloud from ROS msg

    //Read point cloud from ROS msg
    pcl::fromROSMsg(*cloud_in_msg,input_pcl);
    save_point_cloud(pcl_filepath,input_pcl);
  } else {
    pcl::PointCloud<pcl::PointXYZ> input_pcl; //Point cloud from ROS msg

    //Read point cloud from ROS msg
    pcl::fromROSMsg(*cloud_in_msg,input_pcl);
    save_point_cloud(pcl_filepath,input_pcl);
  }

  if (include_tf){
    save_tf(tf_filepath,point_cloud_frame_id,world_frame_id);
  }
  if (spin_once){
    ros::shutdown();
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_pcl");
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  std::string pcl2_input;

  spin_once = true;
  p_nh.getParam("spin_once",spin_once);
  p_nh.getParam("pcl2_input",pcl2_input);
  p_nh.getParam("pcl_filepath",pcl_filepath);
  include_tf = false;
  p_nh.getParam("include_tf",include_tf);
  if (include_tf){
    p_nh.getParam("tf_filepath",tf_filepath);
    p_nh.getParam("point_cloud_frame_id",point_cloud_frame_id);
    p_nh.getParam("world_frame_id",world_frame_id);
  }

  ros::Subscriber sub = nh.subscribe (pcl2_input, 1, input_cloud_callback);

  ros::spin();
}

