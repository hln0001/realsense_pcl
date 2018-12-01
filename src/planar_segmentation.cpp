#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ros/console.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


double wall_threshold;
int wall_point_threshold;
int object_point_threshold;
int max_iterations;
double distance_threshold;

ros::Publisher ground_pub;
ros::Publisher object_pub;
ros::Publisher obstacle_pub; 

void seg_callback(const sensor_msgs::PointCloud2& cloud_msg)
{
  //make container for converted cloud
//  pcl::PCLPointCloud2::Ptr cloud_temp (new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2 cloud_temp;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  //convert cloud to pcl
  pcl_conversions::toPCL(cloud_msg, cloud_temp);
  pcl::fromPCLPointCloud2(cloud_temp, *cloud);

  pcl::ModelCoefficients::Ptr coefficents (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iterations);
  seg.setDistanceThreshold(distance_threshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficents);

  //Set up extraction of ground and objects
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  //Extract ground
  extract.setNegative(false);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  extract.filter(*ground_filtered);

  //Extract objects
  extract.setNegative(true);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  extract.filter(*object_filtered);

  //Publish
  ground_pub.publish(ground_filtered);
  object_pub.publish(object_filtered);

  //Check for obstacles in two stages: high obstacles in ground cloud, amount of points in object cloud
  //Check for walls, etc in the ground plane
  std_msgs::Bool obstacle_state;
  int point_count = 0;

  for (int i = 0; i <= ground_filtered->points.size(); i++) {
    if (ground_filtered->points[i].z >= wall_threshold) {
      point_count ++;
    }
  }

  ROS_INFO("points: %u", point_count);
  ROS_INFO("test");

  if (point_count > wall_point_threshold) {
    obstacle_state.data = true;
  }
  else 
  {
  obstacle_state.data = false;
  }

  //Check for the amount of obstacles in the object cloud
  if (object_filtered->points.size() >= wall_point_threshold && obstacle_state.data == false) {
    obstacle_state.data = true;
  }
  
  obstacle_pub.publish(obstacle_state);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planar_segmentation");
  ros::NodeHandle nh;

  nh.param("wall_threshold", wall_threshold, -0.2);
  nh.param("wall_point_threshold", wall_point_threshold, 50);
  nh.param("object_point_threshold", object_point_threshold, 100);
  nh.param("distance_threshold", distance_threshold, 0.1);
  nh.param("max_iterations", max_iterations, 1000);

  ros::Subscriber sub = nh.subscribe("cloud_filtered", 1, seg_callback);

  ground_pub = nh.advertise<pcl::PointCloud <pcl::PointXYZRGB> >("ground_points", 1);
  object_pub = nh.advertise<pcl::PointCloud <pcl::PointXYZRGB> >("object_points", 1);
  obstacle_pub = nh.advertise<std_msgs::Bool>("obstacle_state", 1);

  ros::spin();
}
