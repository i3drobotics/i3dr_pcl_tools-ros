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
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub_output_pcl2;
bool include_tf;

pcl::PCLPointCloud2::Ptr load_point_cloud(std::string pcl_filepath,std::string frame_id){
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  // Fill in the cloud data
  pcl::PLYReader reader;
  ROS_INFO("Reading point cloud...");
  ROS_INFO("%s",pcl_filepath.c_str());
  reader.read (pcl_filepath, *cloud);
  cloud->header.frame_id = frame_id;
  pcl_conversions::toPCL(ros::Time::now(),cloud->header.stamp);
  ROS_INFO("point cloud loaded.");
  int point_cloud_size = cloud->width;
  ROS_INFO("%d",point_cloud_size);
  return(cloud);
}

geometry_msgs::TransformStamped load_tf(std::string filepath,std::string frame_id,std::string ref_frame_id){
  ROS_INFO("Loading TF from file...");
  std::string line;
  std::ifstream tf_file (filepath.c_str());
  if (tf_file.is_open())
  {
    //get first line of file
    std::getline (tf_file,line);
    tf_file.close();
  }

  //split string by space delimiter
  char delimiter = ' ';
  std::string token;
  std::istringstream tokenStream(line);
  float pose[7];
  int i = 0;
  while((std::getline(tokenStream,token,delimiter)) && (i < 7)){
    pose[i] = ::atof(token.c_str());
    i++;
  }
  ROS_INFO("%f %f %f %f %f %f %f",pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],pose[6]);

  //load data into transform
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = ref_frame_id;
  odom_trans.child_frame_id = frame_id;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = pose[0];
  odom_trans.transform.translation.y = pose[1];
  odom_trans.transform.translation.z = pose[2];
  odom_trans.transform.rotation.x = pose[3];
  odom_trans.transform.rotation.y = pose[4];
  odom_trans.transform.rotation.z = pose[5];
  odom_trans.transform.rotation.w = pose[6];

  ROS_INFO("TF loaded from file.");
  return(odom_trans);
}

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZ> input_pcl){
  sensor_msgs::PointCloud2 pcl2_msg;
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl,pcl2_msg);
  pcl2_msg.header.stamp=ros::Time::now();

  //Publish ROS msg
  pub_output_pcl2.publish(pcl2_msg);
}

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> input_pcl){
  sensor_msgs::PointCloud2 pcl2_msg;
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl,pcl2_msg);
  pcl2_msg.header.stamp=ros::Time::now();

  //Publish ROS msg
  pub_output_pcl2.publish(pcl2_msg);
}

void publish_tf(geometry_msgs::TransformStamped stamped_tf){
  static tf::TransformBroadcaster br;
  // update uStepper transform
  br.sendTransform(stamped_tf);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "load_pcl");
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  double rate = 1;

  std::string pcl2_output,pcl_filepath,tf_filepath;
  std::string point_cloud_frame_id,world_frame_id;

  p_nh.getParam("pcl_filepath",pcl_filepath);
  p_nh.getParam("pcl2_output",pcl2_output);
  p_nh.getParam("point_cloud_frame_id",point_cloud_frame_id);
  p_nh.getParam("rate",rate);
  include_tf = false;
  p_nh.getParam("include_tf",include_tf);

  if (include_tf){
    p_nh.getParam("tf_filepath",tf_filepath);
    p_nh.getParam("world_frame_id",world_frame_id);
  }

  ros::Rate loop_rate(rate);

  pub_output_pcl2 = nh.advertise<sensor_msgs::PointCloud2>(pcl2_output, 1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_pcl_XYZ (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr loaded_pcl_XYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2::Ptr loaded_pcl2 = load_point_cloud(pcl_filepath,point_cloud_frame_id);

  int point_cloud_size = loaded_pcl2->width;
  ROS_INFO("loaded point cloud size: %d",point_cloud_size);

  bool isRGB = false;
  for (int i = 0; i < loaded_pcl2->fields.size(); i++){
    if (loaded_pcl2->fields[i].name == "rgb"){
      isRGB = true;
      break;
    }
  }

  if (isRGB){
    pcl::fromPCLPointCloud2(*loaded_pcl2,*loaded_pcl_XYZRGB);
  } else {
    pcl::fromPCLPointCloud2(*loaded_pcl2,*loaded_pcl_XYZ);
  }

  geometry_msgs::TransformStamped odom_trans;
  if (include_tf){
    odom_trans = load_tf(tf_filepath,point_cloud_frame_id,world_frame_id);
  }

  while(ros::ok()){
    if (isRGB){
      //std::cerr << "XYZRGB point cloud size: " << loaded_pcl_XYZRGB->size() << std::endl;
      publish_point_cloud(*loaded_pcl_XYZRGB);
    } else {
      //std::cerr << "XYZRGB point cloud size: " << loaded_pcl_XYZ->size() << std::endl;
      publish_point_cloud(*loaded_pcl_XYZ);
    }
    if (include_tf){
      publish_tf(odom_trans);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

