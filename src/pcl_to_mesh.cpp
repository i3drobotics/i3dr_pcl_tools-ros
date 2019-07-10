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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>

std::string pcl2_input,filepath,algorithm;
bool isMovingLeastSquares;
bool isNormalOMP;
bool isPassthoughFilter;
double Normal_search_radius;
double MLS_search_radius,MLS_polynomial_order,MLS_upsampling_radius,MLS_upsampling_step_size;
double GPT_search_radius, GPT_mu, GPT_nearest_neighbour, GPT_surface_angle,GPT_min_angle,GPT_max_angle;
double Poisson_depth;
int threads;

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_moving_least_squares(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl,double search_radius, int polynomial_order,double upsampling_radius,double upsampling_step_size){
  std::cerr << "moving least squares..." << std::endl;
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
  mls.setInputCloud(input_pcl);
  mls.setSearchRadius(search_radius);
  mls.setPolynomialFit(true);
  mls.setPolynomialOrder(polynomial_order);
  mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
  mls.setUpsamplingRadius(upsampling_radius);
  mls.setUpsamplingStepSize(upsampling_step_size);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ>());
  mls.process(*cloud_smoothed);
  std::cerr << "moving least squares complete" << std::endl;
  return(cloud_smoothed);
}

pcl::PointCloud<pcl::PointNormal>::Ptr pcl_estimate_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl,double search_radius){
  std::cerr << "estimating normals..." << std::endl;
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (input_pcl);
  n.setInputCloud (input_pcl);
  n.setSearchMethod (tree);
  n.setRadiusSearch(search_radius);
  n.compute (*normals);

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*input_pcl, *normals, *cloud_with_normals);
  std::cerr << "normal estimation complete" << std::endl;
  return(cloud_with_normals);
}

pcl::PointCloud<pcl::PointNormal>::Ptr pcl_estimate_normalsOMP(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl,double search_radius){
  std::cerr << "estimating normals OMP..." << std::endl;
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(input_pcl);
  ne.setRadiusSearch(search_radius);
  ne.setNumberOfThreads(threads);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*input_pcl, centroid);
  ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
  ne.compute(*cloud_normals);
  std::cerr << "normal estimation complete" << std::endl;
  std::cerr << "reverse normals' direction..." << std::endl;

  for(size_t i = 0; i < cloud_normals->size(); ++i){
    cloud_normals->points[i].normal_x *= -1;
    cloud_normals->points[i].normal_y *= -1;
    cloud_normals->points[i].normal_z *= -1;
  }
  std::cerr << "reverse normals' direction complete" << std::endl;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
  pcl::concatenateFields(*input_pcl, *cloud_normals, *cloud_with_normals);
  return(cloud_with_normals);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl){
  std::cerr << "passthrough filter..." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PassThrough<pcl::PointXYZ> filter;
  filter.setInputCloud(input_pcl);
  filter.filter(*filtered);
  std::cerr << "passthrough filter complete" << std::endl;
  return(filtered);
}

pcl::PolygonMesh pcl_to_mesh_Poisson(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals,double depth){
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud (cloud_with_normals);

  std::cerr << "Poisson Reconstruction..." << std::endl;
  pcl::Poisson<pcl::PointNormal> poisson;
  pcl::PolygonMesh mesh;

  poisson.setDepth(depth);

  poisson.setInputCloud(cloud_with_normals);
  poisson.setSearchMethod(tree);
  poisson.reconstruct(mesh);
  std::cerr << "poisson reconstruction complete" << std::endl;

  return(mesh);
}

pcl::PolygonMesh pcl_to_mesh_MC(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals){
  //NOT CURRENTLY WORKING (error:
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud (cloud_with_normals);

  std::cerr << "Marching Cubes Reconstruction..." << std::endl;

  pcl::MarchingCubesRBF<pcl::PointNormal> mc;
  pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
  mc.setInputCloud (cloud_with_normals);
  mc.setSearchMethod (tree);
  mc.reconstruct (*triangles);

  std::cerr << "marching cubes reconstruction complete" << std::endl;

  return(*triangles);
}

pcl::PolygonMesh pcl_to_mesh_GPT(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, double search_radius, double mu, double nearest_neighbors, double surface_angle, double min_angle, double max_angle){
  std::cerr << "Greedy Projection Triangulation..." << std::endl;
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud (cloud_with_normals);

  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (search_radius);

  // Set typical values for the parameters
  gp3.setMu (mu);
  gp3.setMaximumNearestNeighbors (nearest_neighbors);
  gp3.setMaximumSurfaceAngle(surface_angle);
  gp3.setMinimumAngle(min_angle);
  gp3.setMaximumAngle(max_angle);
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree);
  gp3.reconstruct (triangles);

  std::cerr << "greedy projection triangulation complete" << std::endl;

  return(triangles);
}

void input_cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_in_msg){
  ros::Time timestamp = cloud_in_msg->header.stamp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl (new pcl::PointCloud<pcl::PointXYZ>); //Point cloud from ROS msg

    std::cerr << "msg received @ " << timestamp << std::endl;

    //Read point cloud from ROS msg
    pcl::fromROSMsg(*cloud_in_msg,*input_pcl);

    if (input_pcl->size() > 0){
      bool isValidAlgorithm = true;
      pcl::PolygonMesh triangles_mesh;

      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZ>);
      if (isPassthoughFilter){
        filtered = pcl_passthrough_filter(input_pcl);
      } else {
        filtered = input_pcl;
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr MLS (new pcl::PointCloud<pcl::PointXYZ>);
      if (isMovingLeastSquares){
        //radius: 0.08
        //polynomial order: 2
        //upsampling radius: 0.005
        //upsampling step size: 0.003
        MLS = pcl_moving_least_squares(input_pcl,
              MLS_search_radius,
              MLS_polynomial_order,
              MLS_upsampling_radius,
              MLS_upsampling_step_size);
      } else {
        MLS = filtered;
      }

      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
      if (isNormalOMP){
        //radius: 0.08
        cloud_with_normals = pcl_estimate_normalsOMP(filtered,Normal_search_radius);
      } else {
        //radius: 0.08
        cloud_with_normals = pcl_estimate_normals(filtered,Normal_search_radius);
      }

      if (algorithm == "GPT"){
        //radius: 0.08
        //mu: 2.5
        //nearest neightbour: 200
        //surface: 45 degrees
        //min: 10 degrees
        //max: 120 degrees
        triangles_mesh = pcl_to_mesh_GPT(cloud_with_normals,
              GPT_search_radius,
              GPT_mu,
              GPT_nearest_neighbour,
              GPT_surface_angle,
              GPT_min_angle,
              GPT_max_angle);
      } else if (algorithm == "Poisson"){
        triangles_mesh = pcl_to_mesh_Poisson(cloud_with_normals,Poisson_depth);
      } else if (algorithm == "MarchingCubes"){
        triangles_mesh = pcl_to_mesh_MC(cloud_with_normals);
      } else {
        isValidAlgorithm = false;
      }

      if (isValidAlgorithm){
        std::cerr << "saving mesh..." << std::endl;
        pcl::io::saveOBJFile(filepath,triangles_mesh);
      } else {
        std::cerr << "invalid algorithm name" << std::endl;
      }

      std::cerr << "done." << std::endl;
      ros::shutdown();
    } else {
      std::cerr << "Empty point cloud!" << std::endl;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_to_mesh");
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  p_nh.getParam("pcl2_input",pcl2_input);
  p_nh.getParam("filepath",filepath);
  p_nh.getParam("algorithm",algorithm);
  isNormalOMP = false;
  p_nh.getParam("isNormalOMP",isNormalOMP);
  isPassthoughFilter = false;
  p_nh.getParam("isPassthoughFilter",isPassthoughFilter);
  isMovingLeastSquares = false;
  p_nh.getParam("isMovingLeastSquares",isMovingLeastSquares);

  p_nh.getParam("GPT_search_radius",GPT_search_radius);
  p_nh.getParam("GPT_mu",GPT_mu);
  p_nh.getParam("GPT_nearest_neighbour",GPT_nearest_neighbour);
  p_nh.getParam("GPT_surface_angle",GPT_surface_angle);
  p_nh.getParam("GPT_min_angle",GPT_min_angle);
  p_nh.getParam("GPT_max_angle",GPT_max_angle);

  p_nh.getParam("Poisson_depth",Poisson_depth);

  p_nh.getParam("MLS_search_radius",MLS_search_radius);
  p_nh.getParam("MLS_polynomial_order",MLS_polynomial_order);
  p_nh.getParam("MLS_upsampling_radius",MLS_upsampling_radius);
  p_nh.getParam("MLS_upsampling_step_size",MLS_upsampling_step_size);

  p_nh.getParam("Normal_search_radius",Normal_search_radius);

  threads = 0;
  p_nh.getParam("threads",threads);

  ros::Subscriber sub = nh.subscribe (pcl2_input, 1, input_cloud_callback);

  ros::spin();
}
