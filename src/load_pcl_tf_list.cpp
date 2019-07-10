#include <ros/ros.h>
#include <dirent.h>
#include <iostream>
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
#include <tf/transform_broadcaster.h>

class PCLPointCloud2TF {
  pcl::PCLPointCloud2::Ptr point_cloud;
  geometry_msgs::TransformStamped stamped_transform;
public:
  PCLPointCloud2TF (pcl::PCLPointCloud2::Ptr,geometry_msgs::TransformStamped);
  void setPointCloud(pcl::PCLPointCloud2::Ptr pc){
    point_cloud = pc;
  }
  void setTF(geometry_msgs::TransformStamped tf){
    stamped_transform = tf;
  }
  pcl::PCLPointCloud2::Ptr getPointCloud(){
    return(point_cloud);
  }
  geometry_msgs::TransformStamped getTF(){
    return(stamped_transform);
  }
};

PCLPointCloud2TF::PCLPointCloud2TF (pcl::PCLPointCloud2::Ptr pc,geometry_msgs::TransformStamped tf){
  point_cloud = pc;
  stamped_transform = tf;
}

ros::Publisher pub_output_pcl2;
bool include_tf;

pcl::PCLPointCloud2::Ptr load_point_cloud(std::string pcl_filepath,std::string frame_id){
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  // Fill in the cloud data
  pcl::PLYReader reader;
  ROS_INFO("Reading point cloud...");
  reader.read (pcl_filepath, *cloud);
  cloud->header.frame_id = frame_id;
  pcl_conversions::toPCL(ros::Time::now(),cloud->header.stamp);
  ROS_INFO("point cloud loaded.");
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
  odom_trans.transform.translation.x = (float)pose[0];
  odom_trans.transform.translation.y = (float)pose[1];
  odom_trans.transform.translation.z = (float)pose[2];
  odom_trans.transform.rotation.x = (float)pose[3];
  odom_trans.transform.rotation.y = (float)pose[4];
  odom_trans.transform.rotation.z = (float)pose[5];
  odom_trans.transform.rotation.w = (float)pose[6];

  ROS_INFO("TF loaded from file.");
  return(odom_trans);
}

int count_filetype(std::string folderpath,std::string filename,std::string extension){
  int count = 0;
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (folderpath.c_str())) != NULL) {
    //find all file/directorys in folder
    while ((ent = readdir (dir)) != NULL) {
      //count only files with extension that contain the filename string
      if (ent->d_type == DT_REG){
        std::string full_filename = std::string(ent->d_name);
        if(full_filename.substr(full_filename.find_last_of(".") + 1) == extension) {
          if (full_filename.find(filename)!=std::string::npos){
            count++;
            ROS_INFO("filename: %s, count: %d", full_filename.c_str(),count);
          }
        }
      }
    }
    closedir (dir);
  } else {
    //could not open directory
    ROS_ERROR("Unable to open directory");
    ros::shutdown();
  }
  return(count);
}

std::vector<PCLPointCloud2TF> load_point_cloud_list(std::string folderpath,std::string filename,std::string point_cloud_extension,std::string transform_extension,std::string frame_id,std::string ref_frame_id){
  std::vector<PCLPointCloud2TF> point_cloud_tf_vector;
  ROS_INFO("Loading point cloud's & TF's from files");
  //count files in folder with point cloud extension
  int point_clouds_in_folder = count_filetype(folderpath,filename,point_cloud_extension);
  //count files in folder with tf extension
  int tf_in_folder = count_filetype(folderpath,filename,transform_extension);
  if (point_clouds_in_folder <= 0){
    ROS_ERROR("No point cloud files found in directory with extension '%s'",point_cloud_extension.c_str());
    ros::shutdown();
  }
  if (tf_in_folder <= 0){
    ROS_ERROR("No transform files found in directory with extension '%s'",transform_extension.c_str());
    ros::shutdown();
  }
  if (point_clouds_in_folder != tf_in_folder){
    ROS_ERROR("Number of TF files does not match number of Point Cloud file");
    ros::shutdown();
  } else {
    //load point cloud and tf's in folder
    for (int p=0; p < point_clouds_in_folder; p++){
      //load point cloud from file
      std::stringstream pc_ss;
      pc_ss << folderpath << filename << p << "." << point_cloud_extension;
      pcl::PCLPointCloud2::Ptr point_cloud = load_point_cloud(pc_ss.str(),frame_id);
      //load tf from file
      std::stringstream tf_ss;
      tf_ss << folderpath << filename << p << "." << transform_extension;
      geometry_msgs::TransformStamped tf = load_tf(tf_ss.str(),frame_id,ref_frame_id);
      //combine point cloud and tf to PCLPointCloud2TF class
      PCLPointCloud2TF point_cloud_tf(point_cloud,tf);
      //add pcl to vector
      point_cloud_tf_vector.push_back(point_cloud_tf);
    }
  }
  return(point_cloud_tf_vector);
}

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZ> input_pcl){
  sensor_msgs::PointCloud2 pcl2_msg;
  pcl_conversions::toPCL(ros::Time::now(),input_pcl.header.stamp);
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl,pcl2_msg);

  //Publish ROS msg
  pub_output_pcl2.publish(pcl2_msg);
}

void publish_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> input_pcl){
  sensor_msgs::PointCloud2 pcl2_msg;
  pcl_conversions::toPCL(ros::Time::now(),input_pcl.header.stamp);
  //Convert point cloud to ros msg
  pcl::toROSMsg(input_pcl,pcl2_msg);

  //Publish ROS msg
  pub_output_pcl2.publish(pcl2_msg);
}

void publish_tf(geometry_msgs::TransformStamped stamped_tf){
  static tf::TransformBroadcaster br;
  stamped_tf.header.stamp = ros::Time::now();
  //ROS_INFO("publshing tf");
  if (stamped_tf.child_frame_id != stamped_tf.header.frame_id){
    // update uStepper transform
    br.sendTransform(stamped_tf);
  }
}

void publish_point_cloud_tf_list(std::vector<PCLPointCloud2TF> point_cloud_tf_list,ros::Rate loop_rate){
  int file_count = 0;
  for (std::vector<PCLPointCloud2TF>::iterator it = point_cloud_tf_list.begin() ; it != point_cloud_tf_list.end(); ++it){
    if (ros::ok()){
      bool isRGB = false;
      for (int i = 0; i < it->getPointCloud()->fields.size(); i++){
        if (it->getPointCloud()->fields[i].name == "rgb"){
          isRGB = true;
          break;
        }
      }
      if (isRGB){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr loaded_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(*it->getPointCloud(),*loaded_pcl);
        publish_point_cloud(*loaded_pcl);
      } else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_pcl (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*it->getPointCloud(),*loaded_pcl);
        publish_point_cloud(*loaded_pcl);
      }

      publish_tf(it->getTF());
      loop_rate.sleep();
    } else {
      break;
    }
    file_count++;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "load_pcl_tf_list");
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  std::string pcl2_output,tf_extension,pcl_extension,folderpath,filename;
  std::string point_cloud_frame_id,world_frame_id;
  double rate;
  bool loop = false;

  p_nh.getParam("pcl2_output",pcl2_output);
  p_nh.getParam("folderpath",folderpath);
  p_nh.getParam("filename",filename);
  p_nh.getParam("tf_extension",tf_extension);
  p_nh.getParam("pcl_extension",pcl_extension);
  p_nh.getParam("world_frame_id",world_frame_id);
  p_nh.getParam("point_cloud_frame_id",point_cloud_frame_id);
  p_nh.getParam("rate",rate);
  p_nh.getParam("loop",loop);

  ros::Rate loop_rate(rate);
  pub_output_pcl2 = nh.advertise<sensor_msgs::PointCloud2>(pcl2_output, 1);

  std::vector<PCLPointCloud2TF> point_cloud_tf_list = load_point_cloud_list(folderpath,filename,pcl_extension,tf_extension,point_cloud_frame_id,world_frame_id);

  if (loop){
    ROS_INFO("Publishing point cloud & tf's, rate: %f, looping: true",rate);
    while(ros::ok()){
      publish_point_cloud_tf_list(point_cloud_tf_list,loop_rate);
    }
  } else {
    ROS_INFO("Publishing point cloud & tf's, rate: %f, looping: false",rate);
    publish_point_cloud_tf_list(point_cloud_tf_list,loop_rate);
    ROS_INFO("Sequence complete. Shutting down node...");
  }
  return 0;
}
