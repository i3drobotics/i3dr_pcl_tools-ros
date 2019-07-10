#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

std::string input_point_cloud2_node;
std::string output_point_cloud2_node;
std::string output_point_cloud_node;
ros::Publisher pub_crop_pcl2;
ros::Publisher pub_crop_pcl;
ros::Publisher pub_aligned_pcl2;
ros::Time pcl_timestamp;
double crop_minX,crop_minY,crop_minZ;
double crop_maxX,crop_maxY,crop_maxZ;

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZ> input_pcl){
  sensor_msgs::PointCloud2 pcl2_msg;
  sensor_msgs::PointCloud pcl_msg;
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl,pcl2_msg);

  //Publish ROS msg
  pub_crop_pcl2.publish(pcl2_msg);

  sensor_msgs::convertPointCloud2ToPointCloud(pcl2_msg,pcl_msg);
  pub_crop_pcl.publish(pcl_msg);
  //std::cerr << "Published point cloud." << std::endl;
}

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> input_pcl){
  sensor_msgs::PointCloud2 pcl2_msg;
  sensor_msgs::PointCloud pcl_msg;
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl,pcl2_msg);

  //Publish ROS msg
  pub_crop_pcl2.publish(pcl2_msg);

  sensor_msgs::convertPointCloud2ToPointCloud(pcl2_msg,pcl_msg);
  pub_crop_pcl.publish(pcl_msg);
  //std::cerr << "Published point cloud." << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr crop_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl,double minX,double minY,double minZ,double maxX,double maxY,double maxZ){
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
  boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
  boxFilter.setInputCloud(input_pcl);
  boxFilter.filter(*filtered_pcl);
  return (filtered_pcl);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr crop_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl,double minX,double minY,double minZ,double maxX,double maxY,double maxZ){
  pcl::CropBox<pcl::PointXYZRGB> boxFilter;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
  boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
  boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
  boxFilter.setInputCloud(input_pcl);
  boxFilter.filter(*filtered_pcl);
  return (filtered_pcl);
}

void input_cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_in_msg){
    pcl_timestamp = cloud_in_msg->header.stamp;
    bool isRGB = false;
    for (int i = 0; i < cloud_in_msg->fields.size(); i++){
      if (cloud_in_msg->fields[i].name == "rgb"){
        isRGB = true;
        break;
      }
    }

    //std::cerr << "msg received @ " << pcl_timestamp << std::endl;
    if (isRGB){
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZRGB>); //Point cloud from ROS msg
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB> output_pcl;
      //Read point cloud from ROS msg
      pcl::fromROSMsg(*cloud_in_msg,*input_pcl);

      filtered_pcl = crop_pcl(input_pcl,crop_minX,crop_minY,crop_minZ,crop_maxX,crop_maxY,crop_maxZ);

      output_pcl = *filtered_pcl;
      publish_point_cloud(output_pcl);
    } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZ>); //Point cloud from ROS msg
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ> output_pcl;
      //Read point cloud from ROS msg
      pcl::fromROSMsg(*cloud_in_msg,*input_pcl);

      filtered_pcl = crop_pcl(input_pcl,crop_minX,crop_minY,crop_minZ,crop_maxX,crop_maxY,crop_maxZ);

      output_pcl = *filtered_pcl;
      publish_point_cloud(output_pcl);
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crop_pcl");
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  p_nh.getParam("pcl2_input",input_point_cloud2_node);
  p_nh.getParam("pcl2_output",output_point_cloud2_node);
  p_nh.getParam("pcl_output",output_point_cloud_node);
  p_nh.getParam("minX",crop_minX);
  p_nh.getParam("maxX",crop_maxX);
  p_nh.getParam("minY",crop_minY);
  p_nh.getParam("maxY",crop_maxY);
  p_nh.getParam("minZ",crop_minZ);
  p_nh.getParam("maxZ",crop_maxZ);

  ros::Subscriber sub = nh.subscribe (input_point_cloud2_node, 1, input_cloud_callback);
  pub_crop_pcl2 = nh.advertise<sensor_msgs::PointCloud2>(output_point_cloud2_node, 1);
  pub_crop_pcl = nh.advertise<sensor_msgs::PointCloud>(output_point_cloud_node, 1);

  ros::spin();
}
