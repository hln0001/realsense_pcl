#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

ros::Publisher cloud_pub;
//ros::Publisher pcl_pub;

void cloud_callback (const sensor_msgs::PointCloud2& cloud_msg)
{
  //make container for converted cloud
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  //convert cloud to pcl
  pcl_conversions::toPCL(cloud_msg, *cloud);

  //other pointclouds for filtering
//  pcl::PCLPointCloud2::Ptr pass_z (new pcl::PCLPointCloud2 ());
//  pcl::PCLPointCloud2::Ptr pass_zx (new pcl::PCLPointCloud2 ());

  sensor_msgs::PointCloud2 cloud_out;

  //Passthrough filter the points in z
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-1,1);
  pass.filter(*cloud);

  //Passthough in y
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.5,0.5);
  pass.filter(*cloud);

  //passthrough filter the points in x
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-1,5);
  pass.filter(*cloud);

  //Voxel Grid Filter the points
  pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(0.01,0.01,0.01);
  vox.filter(*cloud_filtered);

//  pcl_pub.publish(cloud_filtered);

  //Publisher
  pcl_conversions::fromPCL(*cloud_filtered, cloud_out);
  cloud_pub.publish(cloud_out);
  //cloud_pub.publish(*cloud_filtered);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_downsample");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("camera/depth_registered/points", 1, cloud_callback);

  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 1);
//  pcl_pub = nh.advertise<pcl::PCLPointCloud2>("cloud_filtered_pcl",1);

  ros::spin();
}
