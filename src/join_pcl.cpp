#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>

std::string input_point_cloud2_node_1;
std::string input_point_cloud2_node_2;
std::string output_point_cloud2_node;
std::string output_point_cloud_node;
ros::Publisher pub_full_map_pcl2;
ros::Publisher pub_full_map_pcl;
ros::Publisher pub_aligned_pcl2;
ros::Time pcl_timestamp;
double resolution;

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

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZ> input_pcl){
  sensor_msgs::PointCloud2 pcl2_msg;
  sensor_msgs::PointCloud pcl_msg;
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl,pcl2_msg);

  //Publish ROS msg
  pub_full_map_pcl2.publish(pcl2_msg);

  sensor_msgs::convertPointCloud2ToPointCloud(pcl2_msg,pcl_msg);
  pub_full_map_pcl.publish(pcl_msg);
  //std::cerr << "Published point cloud." << std::endl;
}

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> input_pcl){
  sensor_msgs::PointCloud2 pcl2_msg;
  sensor_msgs::PointCloud pcl_msg;
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl,pcl2_msg);

  //Publish ROS msg
  pub_full_map_pcl2.publish(pcl2_msg);

  sensor_msgs::convertPointCloud2ToPointCloud(pcl2_msg,pcl_msg);
  pub_full_map_pcl.publish(pcl_msg);
  //std::cerr << "Published point cloud." << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr add_point_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl_1,pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl_2){
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pcl(new pcl::PointCloud<pcl::PointXYZ>());
  //add point cloud one & point cloud two to new point cloud
  output_pcl->operator +=(*input_pcl_1);
  output_pcl->operator +=(*input_pcl_2);
  remove_point_cloud_duplicates(output_pcl);
  output_pcl->header = input_pcl_1->header;
  output_pcl->width = output_pcl->size();
  output_pcl->height = 1;
  return(output_pcl);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr add_point_clouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl_1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl_2){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
  //add point cloud one & point cloud two to new point cloud
  output_pcl->operator +=(*input_pcl_1);
  output_pcl->operator +=(*input_pcl_2);
  remove_point_cloud_duplicates(output_pcl);
  output_pcl->header = input_pcl_1->header;
  output_pcl->width = output_pcl->size();
  output_pcl->height = 1;
  return(output_pcl);
}

/*
void add_pcl_to_map(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl){

  pcl::PCLPointCloud2::Ptr full_map_pcl2(new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr full_map_filtered_pcl(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCLPointCloud2::Ptr full_map_filtered_pcl2(new pcl::PCLPointCloud2 ());

  full_map_pcl->operator +=(*input_pcl);

  remove_point_cloud_duplicates(full_map_pcl);
  //pcl::toPCLPointCloud2(*full_map_pcl,*full_map_pcl2);
  //full_map_filtered_pcl2 = filter_to_grid(full_map_pcl2,resolution);
 //pcl::fromPCLPointCloud2(*full_map_filtered_pcl2,*full_map_filtered_pcl);

  full_map_pcl->header = input_pcl->header;
  full_map_pcl->width = full_map_pcl->size();
  full_map_pcl->height = 1;
  //full_map_pcl = full_map_filtered_pcl;
  std::cerr << "Glued point cloud size: " << full_map_pcl->points.size () << std::endl;
}

void icp_convergence(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
  double fit_score = 0.1;
  if (full_map_pcl->size() > 0){
    //std::cout << "Point cloud input size: " << cloud_in->points.size () << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCLPointCloud2::Ptr aligned_pcl2(new pcl::PCLPointCloud2 ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_filtered_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCLPointCloud2::Ptr aligned_filtered_pcl2(new pcl::PCLPointCloud2 ());

    icp.setInputSource(cloud_in);
    icp.setInputTarget(full_map_pcl);
    icp.setMaxCorrespondenceDistance (resolution);
    icp.align(*aligned_pcl);
    std::cerr << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();
    //std::cerr << icp.getFinalTransformation() << std::endl;

    if (icp.hasConverged() && icp.getFitnessScore() < fit_score){
      std::cerr << " ...adding to map" << std::endl;
      pcl::toPCLPointCloud2(*aligned_pcl,*aligned_pcl2);
      aligned_filtered_pcl2 = filter_to_grid(aligned_pcl2,resolution);
      pcl::fromPCLPointCloud2(*aligned_filtered_pcl2,*aligned_filtered_pcl);

      //Convert point cloud to ros msg
      sensor_msgs::PointCloud2 pcl2_msg;
      pcl::toROSMsg(*aligned_filtered_pcl,pcl2_msg);

      //Publish ROS msg
      pub_aligned_pcl2.publish(pcl2_msg);

      full_map_pcl->operator +=(*aligned_filtered_pcl);
      remove_point_cloud_duplicates(full_map_pcl);
      remove_nan(full_map_pcl);
    } else {
      std::cerr << " ...fit score too high" << std::endl;
    }
  } else {
    full_map_pcl->points = cloud_in->points;
  }
  full_map_pcl->header = cloud_in->header;
  full_map_pcl->width = full_map_pcl->size();
  full_map_pcl->height = 1;
  pcl_conversions::toPCL(ros::Time::now(), full_map_pcl->header.stamp);
  //std::cerr << "Stiched point cloud size: " << full_map_pcl->points.size () << std::endl;
}


void input_cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_in_msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZ>); //Point cloud from ROS msg
    pcl::PointCloud<pcl::PointXYZ> output_pcl;

    pcl_timestamp = cloud_in_msg->header.stamp;
    std::cerr << "msg received @ " << pcl_timestamp << std::endl;

    //Read point cloud from ROS msg
    pcl::fromROSMsg(*cloud_in_msg,*input_pcl);

    add_pcl_to_map(input_pcl);
    //icp_convergence(input_pcl);

    output_pcl = *full_map_pcl;
    publish_point_cloud(output_pcl);
}
*/

void joint_input_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_in_msg_1,const sensor_msgs::PointCloud2ConstPtr& cloud_in_msg_2){
  pcl_timestamp = cloud_in_msg_1->header.stamp;
  //std::cerr << "msg received @ " << pcl_timestamp << std::endl;

  bool isRGB = false;
  for (int i = 0; i < cloud_in_msg_1->fields.size(); i++){
    if (cloud_in_msg_1->fields[i].name == "rgb"){
      isRGB = true;
      break;
    }
  }

  if (isRGB){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl_1(new pcl::PointCloud<pcl::PointXYZRGB>); //Point cloud from ROS msg
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl_2(new pcl::PointCloud<pcl::PointXYZRGB>); //Point cloud from ROS msg
    //Read point clouds from ROS msgs
    pcl::fromROSMsg(*cloud_in_msg_1,*input_pcl_1);
    pcl::fromROSMsg(*cloud_in_msg_2,*input_pcl_2);
    //Combine point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_pcl = add_point_clouds(input_pcl_1,input_pcl_2);
    //Publish combined point cloud
    publish_point_cloud(*output_pcl);
  } else {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl_1(new pcl::PointCloud<pcl::PointXYZ>); //Point cloud from ROS msg
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl_2(new pcl::PointCloud<pcl::PointXYZ>); //Point cloud from ROS msg
    //Read point clouds from ROS msgs
    pcl::fromROSMsg(*cloud_in_msg_1,*input_pcl_1);
    pcl::fromROSMsg(*cloud_in_msg_2,*input_pcl_2);
    //Combine point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_pcl = add_point_clouds(input_pcl_1,input_pcl_2);
    //Publish combined point cloud
    publish_point_cloud(*output_pcl);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "join_pcl");
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  p_nh.getParam("resolution",resolution);
  p_nh.getParam("pcl2_input_1",input_point_cloud2_node_1);
  p_nh.getParam("pcl2_input_2",input_point_cloud2_node_2);
  p_nh.getParam("pcl2_output",output_point_cloud2_node);
  p_nh.getParam("pcl_output",output_point_cloud_node);

  pub_full_map_pcl2 = nh.advertise<sensor_msgs::PointCloud2>(output_point_cloud2_node, 1);
  pub_full_map_pcl = nh.advertise<sensor_msgs::PointCloud>(output_point_cloud_node, 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_1(nh, input_point_cloud2_node_1, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_2(nh, input_point_cloud2_node_2, 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_input_1, sub_input_2);

  sync.registerCallback(boost::bind(&joint_input_cloud_callback, _1, _2));

  ros::spin();
}
