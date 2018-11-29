#include <ros/ros.h>

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

ros::Publisher ground_pub;
ros::Publisher object_pub;

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
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(.01);

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
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planar_segmentation");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("cloud_filtered", 1, seg_callback);

  ground_pub = nh.advertise<pcl::PointCloud <pcl::PointXYZRGB> >("ground_points", 1);
  object_pub = nh.advertise<pcl::PointCloud <pcl::PointXYZRGB> >("object_points", 1);

  ros::spin();
}
