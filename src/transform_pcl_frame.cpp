#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>

std::string input_point_cloud2_node;
std::string output_point_cloud2_node;
std::string output_point_cloud_node;
std::string output_frame;
ros::Publisher pub_world_tf_pcl2;
ros::Publisher pub_world_tf_pcl;
tf::TransformListener *tf_listener;
ros::Time pcl_timestamp;

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZ> input_pcl){
  sensor_msgs::PointCloud2 pcl2_msg;
  sensor_msgs::PointCloud pcl_msg;
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl,pcl2_msg);

  //Publish ROS msg
  pub_world_tf_pcl2.publish(pcl2_msg);

  sensor_msgs::convertPointCloud2ToPointCloud(pcl2_msg,pcl_msg);
  pub_world_tf_pcl.publish(pcl_msg);
    //std::cerr << "Published point cloud." << std::endl;
}

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> input_pcl){
  sensor_msgs::PointCloud2 pcl2_msg;
  sensor_msgs::PointCloud pcl_msg;
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl,pcl2_msg);

  //Publish ROS msg
  pub_world_tf_pcl2.publish(pcl2_msg);

  sensor_msgs::convertPointCloud2ToPointCloud(pcl2_msg,pcl_msg);
  pub_world_tf_pcl.publish(pcl_msg);
    //std::cerr << "Published point cloud." << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transform_pcl_to_world(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl){
  //std::cerr << "waiting for transform..." << std::endl;
  tf_listener->waitForTransform(output_frame,input_pcl->header.frame_id, pcl_timestamp, ros::Duration(0.1));

  pcl::PointCloud<pcl::PointXYZ>::Ptr tf_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  //std::cerr << "transforming point cloud to world frame" << std::endl;
  pcl_ros::transformPointCloud(output_frame,*input_pcl,*tf_pcl,*tf_listener);
  tf_pcl->header.frame_id = output_frame;
  return(tf_pcl);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_pcl_to_world(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl){
  //std::cerr << "waiting for transform..." << std::endl;
  tf_listener->waitForTransform(output_frame,input_pcl->header.frame_id, pcl_timestamp, ros::Duration(0.1));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tf_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
  //std::cerr << "transforming point cloud to world frame" << std::endl;
  pcl_ros::transformPointCloud(output_frame,*input_pcl,*tf_pcl,*tf_listener);
  tf_pcl->header.frame_id = output_frame;
  return(tf_pcl);
}

void input_cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_in_msg){
    pcl_timestamp = cloud_in_msg->header.stamp;
    //std::cerr << "msg received @ " << pcl_timestamp << std::endl;

    bool isRGB = false;
    for (int i = 0; i < cloud_in_msg->fields.size(); i++){
      if (cloud_in_msg->fields[i].name == "rgb"){
        isRGB = true;
        break;
      }
    }

    if (isRGB){
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZRGB>); //Point cloud from ROS msg
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tf_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB> output_pcl;
      //Read point cloud from ROS msg
      pcl::fromROSMsg(*cloud_in_msg,*input_pcl);

      tf_pcl = transform_pcl_to_world(input_pcl);

      output_pcl = *tf_pcl;
      publish_point_cloud(output_pcl);
    } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZ>); //Point cloud from ROS msg
      pcl::PointCloud<pcl::PointXYZ>::Ptr tf_pcl(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ> output_pcl;
      //Read point cloud from ROS msg
      pcl::fromROSMsg(*cloud_in_msg,*input_pcl);

      tf_pcl = transform_pcl_to_world(input_pcl);

      output_pcl = *tf_pcl;
      publish_point_cloud(output_pcl);
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform_pcl_frame");
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  p_nh.getParam("pcl2_input",input_point_cloud2_node);
  p_nh.getParam("pcl2_output",output_point_cloud2_node);
  p_nh.getParam("pcl_output",output_point_cloud_node);
  p_nh.getParam("tf",output_frame);

  ros::Subscriber sub = nh.subscribe (input_point_cloud2_node, 1, input_cloud_callback);
  pub_world_tf_pcl2 = nh.advertise<sensor_msgs::PointCloud2>(output_point_cloud2_node, 1);
  pub_world_tf_pcl = nh.advertise<sensor_msgs::PointCloud>(output_point_cloud_node, 1);

  tf_listener = new tf::TransformListener();

  ros::spin();
}
