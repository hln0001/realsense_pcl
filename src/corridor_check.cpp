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

std::vector<float> hazard_x;
std::vector<float> hazard_y;
double hazard_map_size_y;
double hazard_map_size_x_neg;
double hazard_map_size_x_pos;

int collision_slowdown; //counter for points within the slowdown region
int collision_avoid; //counter for points within avoid region
int collision_right; //counter for points to the left
int collision_left; //counter for points to the left

int collision_threshold; //number of points in collision to trigger avoid/slowdown

void generatePolarHist()
{
  
}

void generateHistGrid(pcl::PCLPointCloud2 cloud_filtered)
{
  //define variables used in this section
 std::vector<float> point;
 std::vector<std::vector<std::vector<float> > > hazard_map_cells((hazard_map_size_x_pos + hazard_map_size_x_neg)*(hazard_map_size_y*2));
 int index = 0;

 //hazard_map_cells is a vector of vectors, each element of it is a grid in the hazard map that includes 0-N points
 for (int i = 0; i< cloud_filtered->points.size(); i++)
 {
     point.push_back(cloud_filtered->points[i].x);
     point.push_back(cloud_filtered->points[i].y);
     point.push_back(cloud_filtered->points[i].z);

     //the index checks which gird a point belongs to

     index = floor(cloud_filtered->points[i].x + hazard_map_size_x_neg) * (2 * hazard_map_size_y) + floor(cloud_filtered->points[i].y + hazard_map_size_y);

     hazard_map_cells[index].push_back(point);
     point.clear();
 }

 //do the calculation
 hazard_x.clear();
 hazard_y.clear();
 for (int i = 0; i < hazard_map_cells.size(); i++) // for every cell
 {
   //cout << i << endl;
   //define variables used to calculate mean x y z and variance of z
   float total_x = 0;
   float total_y = 0;
   float total_z = 0;
   float average_x = 0;
   float average_y = 0;
   float average_z = 0;
   float variance_z = 0;

     for (int j = 0; j < hazard_map_cells[i].size(); j++)
     {
         total_x += hazard_map_cells[i][j][0];
         total_y += hazard_map_cells[i][j][1];
         total_z += hazard_map_cells[i][j][2];
     }
     average_x = total_x/hazard_map_cells[i].size();
     average_y = total_y/hazard_map_cells[i].size();
     average_z = total_z/hazard_map_cells[i].size();
     for (int j = 0; j < hazard_map_cells[i].size(); j++)
     {
         variance_z = (hazard_map_cells[i][j][2]-average_z) * (hazard_map_cells[i][j][2]-average_z);
     }
     variance_z = sqrt(variance_z);

     //the point should have at least one of the x, y or z not equal to 0 inorder to be included in the local map

     if (total_x || total_y || total_z)
     {

       hazard_x.push_back(average_x);
       hazard_y.push_back(average_y);
     }

     generatePolarHist();
}

void corridorCallback(const sensor_msgs::PointCloud2& cloud_msg)
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
  pass.setFilterLimits(- hazard_map_size_y,  hazard_map_size_y); //not sure what these should be
  pass.filter(*cloud);

  //passthrough filter the points in x
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(- hazard_map_size_x_neg, hazard_map_size_x_pos); //again, not sure what to choose
  pass.filter(*cloud);

  //Voxel Grid Filter the points
  pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(0.01,0.01,0.01);
  vox.filter(*cloud_filtered); //I should make it so I don't have to pass these as arguments.

  for(i = 0; i < cloud_filtered->points.size(); i++)
  {
    if (cloud_filtered->points[i].y < corridor_width * -0.5 && cloud_filtered->points[i].y > corridor_width * 0.5 )
      {
        if (cloud_filtered->points[i].x > 0 && cloud_filtered->points[i].x < corridor_length_slowdown)
        {
          collision_slowdown++;
        }

        if (cloud_filtered->points[i].x > 0 && cloud_filtered->points[i].x < corridor_length_avoid)
        {
          collision_avoid++;
          /*
          if (cloud_filtered->points[i].y > 0)
          {
            collision_right++;
          }
          else
          {
            collision_left++;
          }
          */
        }
      }
    }

  if (collision_slowdown > collision_threshold)
  {
    slowdown = true;
  }
  if (collision_avoid > collision_threshold)
  {
    generateHistGrid();
  }
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
