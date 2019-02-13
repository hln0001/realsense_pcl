#include <ros/ros.h>
#include <ros/console.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_messages/CollisionTrajectory.h>
#include <math.h>

double camera_height; //camera_height off the ground for ground removal
double corridor_width; //robot footprint width
double corridor_length_slowdown; //distance to slowdown on collision
double corridor_length_avoid; //distance to stop/search on collision
double reactive_height; //height of obstacles handled by drivetrain
double threshold_obstacle_distance;
int threshold_obstacle_number;
double near_dist;
double far_dist;

std::vector<float> hazard_x;
std::vector<float> hazard_y;
double hazard_map_size_y;
double hazard_map_size_x_neg;
double hazard_map_size_x_pos;

int collision_slowdown; //counter for points within the slowdown region
int collision_avoid; //counter for points within avoid region
int collision_right; //counter for points to the left
int collision_left; //counter for points to the left
int collision_status = 0;
int threshold_min_angle;

int collision_threshold; //number of points in collision to trigger avoid/slowdown

ros::Publisher pub;
ros::Publisher pub2;

pcl_messages::CollisionTrajectory traj_msg;

int collisionCounter(double angle, double distance)
{
  int count = 0;
	for(int i = 0; i < hazard_x.size(); i++)
	{
		if(hazard_x[i] > - 1 / tan(angle) * hazard_y[i] && hazard_x[i] < - 1 / tan(angle) * hazard_y[i] + distance / sin(angle))
		{
			if(fabs(cos(angle) * (hazard_x[i] - hazard_y[i] * tan(angle))) < threshold_obstacle_distance)
			{
				count++;
			}
		}
	}
  return count;

}

bool generateTrajectory(double left, double right, int count_left, int count_right)
{
  int left_far_count = -1;
  int right_far_count = -1;
  int left_near_count = -1;
  int right_near_count = -1;
  int left_status = 0;

  if (count_left < count_right)
  {
    left_near_count = collisionCounter(left, near_dist);

    if(left_near_count < threshold_obstacle_number && left_near_count > 0)
    {
      left_far_count = collisionCounter(left, far_dist);

      if (left_far_count < threshold_obstacle_number && left_far_count > 0) //if far is clear
      {
        //go left_far
        left_status = 2;
        traj_msg.angle = left;
        traj_msg.distance = far_dist;
      }
      else //far is no good but near is
      {
        //go left_near
        left_status = 1;
        traj_msg.angle = left;
        traj_msg.distance = near_dist;
      }
    }
    else
    {
      //dont go left
      left_status = 0;
    }
  }

  if(count_right <= count_left || left_status != 2)
  {
    right_near_count = collisionCounter(right, near_dist);

    if(right_near_count < threshold_obstacle_number && right_near_count > 0)
    {
      right_far_count = collisionCounter(right, far_dist);
      if (right_far_count < threshold_obstacle_number && right_far_count > 0) //if far is clear
      {
        //go right_far
        traj_msg.angle = right;
        traj_msg.distance = far_dist;
      }
      else //far is no good
      {
        if(right_near_count < left_near_count)//if right is better
        {
          //go right_near
          traj_msg.angle = right;
          traj_msg.distance = near_dist;
        }
      }
    }
    else
    {
      //dont go right
      return false; //no good trajectory.
    }
  }

}

void generateHazardMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  //define variables used in this section
 std::vector<float> point;
 std::vector<std::vector<std::vector<float> > > hazard_map_cells((hazard_map_size_x_pos + hazard_map_size_x_neg)*(hazard_map_size_y*2));
 int index = 0;

 //hazard_map_cells is a vector of vectors, each element of it is a grid in the hazard map that includes 0-N points
 for (int i = 0; i< cloud->points.size(); i++)
 {
     point.push_back(cloud->points[i].x);
     point.push_back(cloud->points[i].y);
     point.push_back(cloud->points[i].z);

     //the index checks which gird a point belongs to

     index = floor(cloud->points[i].x + hazard_map_size_x_neg) * (2 * hazard_map_size_y) + floor(cloud->points[i].y + hazard_map_size_y);

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

   }
}

void corridorCallback(const sensor_msgs::PointCloud2& cloud_msg)
{
  //make container for converted cloud
  pcl::PCLPointCloud2::Ptr cloud_temp (new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  //convert cloud to pcl
  pcl_conversions::toPCL(cloud_msg, *cloud_temp);
  pcl::fromPCLPointCloud2(*cloud_temp, *cloud);

  //Passthrough filter the points in z
  pcl::PassThrough<pcl::PointXYZRGB> pass;

  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-reactive_height , camera_height * 1.1);
  pass.filter(*cloud);

  //Passthough in y
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-hazard_map_size_y,  hazard_map_size_y);
  //pass.setFilterLimits(-10,  10);
  pass.filter(*cloud);

  //passthrough filter the points in x
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-hazard_map_size_x_neg, hazard_map_size_x_pos); 
  pass.filter(*cloud);

  //Voxel Grid Filter the points
  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(0.02,0.02,0.02);
  vox.filter(*cloud); 

  int cloudsize = cloud->points.size();
  ROS_INFO("%i",cloudsize);

  pub2.publish(cloud);

  collision_left = 0;
  collision_right = 0;
  collision_avoid = 0;
  collision_slowdown = 0;
  //collision_status = 0;


  for(int i = 0; i < cloud->points.size(); i++)
  {
    if (cloud->points[i].y < corridor_width * -0.5 && cloud->points[i].y > corridor_width * 0.5 )
      {
        if (cloud->points[i].x > 0 && cloud->points[i].x < corridor_length_slowdown)
        {
          collision_slowdown++;
        }

        if (cloud->points[i].x > 0 && cloud->points[i].x < corridor_length_avoid)
        {
          collision_avoid++;

          if (cloud->points[i].y > 0)
          {
            collision_right++;
          }
          else
          {
            collision_left++;
          }

        }
      }
    }
  if (collision_slowdown > collision_threshold)
  {
    traj_msg.slowdown = true;
  }
  else
  {
    collision_slowdown = 0;
  }

  if (collision_avoid > collision_threshold)
  {
    traj_msg.collision_status += 1;

    hazard_x.clear();
    hazard_y.clear();

    if (cloud->points.size() > 0)
    {

      double right_angle = threshold_min_angle * M_PI / 180;
      double left_angle = -threshold_min_angle * M_PI / 180;

      double right_angle_final = 0;
      double left_angle_final = 0;

      generateHazardMap(cloud);
      bool traj_available = false;

      for(int j = 0; j < 5; j++)
      {
        right_angle_final = right_angle + j * (15 * M_PI / 180);
        left_angle_final = left_angle - j * (15 * M_PI / 180);

        traj_available = generateTrajectory(-left_angle_final + 0.5 * M_PI, 0.5 * M_PI - right_angle_final, collision_left, collision_right);

        if(traj_available)
        {
          break;
        }
      }

      if(!traj_available)
      {
        //go straight to the side
        traj_msg.angle = 110*M_PI/180;
        traj_msg.distance = 3;
      }
    }
  }
  else
  {
    collision_avoid = 0;
  }
  pub.publish(traj_msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_detection");
  ros::NodeHandle nh;

  //double hazard_map_size_y;
  //double hazard_map_size_x_neg;
  //double hazard_map_size_x_pos;

  //int collision_slowdown; //counter for points within the slowdown region
  //int collision_avoid; //counter for points within avoid region
  //int collision_right; //counter for points to the left
  //int collision_left; //counter for points to the left

  nh.param("collision_threshold", collision_threshold, 100); //number of points in collision to trigger avoid/slowdown
  nh.param("camera_height", camera_height, 0.2);
  nh.param("corridor_width", corridor_width, 0.72);
  nh.param("corridor_length_slowdown", corridor_length_slowdown, 8.0);
  nh.param("corridor_length_avoid", corridor_length_avoid, 1.5);
  nh.param("reactive_height", reactive_height, 0.1);
  nh.param("hazard_map_size_y", hazard_map_size_y, 5.0);
  nh.param("threshold_min_angle", threshold_min_angle, 5);
  nh.param("threshold_obstacle_distance", threshold_obstacle_distance, 1.0);
  nh.param("threshold_obstacle_number", threshold_obstacle_number, 100);
  nh.param("near_dist", near_dist, 4.0);
  nh.param("far_dist", far_dist, 8.0);
  nh.param("hazard_map_size_x_neg", hazard_map_size_x_neg, 1.0);
  nh.param("hazard_map_size_x_pos", hazard_map_size_x_pos, 10.0);

  ros::Subscriber sub = nh.subscribe("camera/depth_registered/points", 1, corridorCallback);

  pub = nh.advertise<pcl_messages::CollisionTrajectory>("collision_trajectory", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2>("filtered",1);

  ros::spin();
}
