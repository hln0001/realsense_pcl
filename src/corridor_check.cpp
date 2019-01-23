#include <ros/ros.h>
#include <ros/console.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>


double camera_height; //camera_height off the ground for ground removal
double corridor_width; //robot footprint width
double corridor_length_slowdown; //distance to slowdown on collision
double corridor_length_avoid; //distance to stop/search on collision
double reactive_height; //height of obstacles handled by drivetrain

double hazard_map_size_y;
double hazard_map_size_x_neg;
double hazard_map_size_x_pos;

int collision_slowdown; //counter for points within the slowdown region
int collision_avoid; //counter for points within avoid region
int collision_right; //counter for points to the left
int collision_left; //counter for points to the left

int collision_threshold; //number of points in collision to trigger avoid/slowdown


void trajectoryPlanner()

void checkEnvelope(pcl::PCLPointCloud2 cloud_in)
{

}

void hazardMapCallback(const sensor_msgs::PointCloud2& cloud_msg)
{
  //make container for converted cloud
  //  pcl::PCLPointCloud2::Ptr cloud_temp (new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  //convert cloud to pcl
  pcl_conversions::toPCL(cloud_msg, *cloud);

  //Passthrough filter the points in z
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(- camera_height + reactive_height , camera_height * 1.1);
  pass.filter(*cloud);

  //Passthough in y
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(- hazard_map_size_y,  hazard_map_size_y);
  pass.filter(*cloud);

  //passthrough filter the points in x
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(- hazard_map_size_x_neg, hazard_map_size_x_pos);
  pass.filter(*cloud);

  //Voxel Grid Filter the points
  pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(0.01,0.01,0.01);
  vox.filter(*cloud_filtered); //I should make it so I don't have to pass these as arguments. 

  //check the hazard map for points in Envelope:
  //(This is pseudocode to get the thought down.
  //please don't try to compile this...
  //that would be embarassing for all of us.

  //for(all points in map)
  //{
  //  if (point_far)
  //  {
  //    collision_slowdown++;
  //    if(point_on_left)
  //    {
  //      collision_left++;
  //    }
  //    if(point_on_right)
  //    {
  //      collision_right++;
  //    }
  //  }
  //
  //  if (point_close)
  //  {
  //    collision_avoid++;
  //  }
  //}
  //if (collision_slowdown > collision_threshold)
  //{
  //  do the slowing down thing;
  //}
  //if (collision_avoid > collision_threshold)
  //{
  //  generateHazardMap(collision_left, collision_right)
  //}
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
