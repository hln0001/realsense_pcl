#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <pcl_messages/CollisionTrajectory.h>
//#include <>
//#include <>
//#include <>
//#include <>
#include <cmath>

double kh = 1;
double kd = 1;
double goal_threshold = 1;

int collision_status;
geometry_msgs::Twist cmd_vel;
geometry_msgs::Pose2D x_0; //current pose
geometry_msgs::Pose2D x_g; //goal pose

class FeedbackController
{
  ros::NodeHandle n_;
  ros::Publisher cmd_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber goal_sub_ = n_.subscribe("goal_pose", 1, &FeedbackController::goalCallback, this);
  ros::Subscriber pose_sub_ = n_.subscribe("pose", 1, &FeedbackController::poseCallback, this);
  ros::Subscriber haz_sub_ = n_.subscribe("collision_trajectory", 1, &FeedbackController::hazardCallback, this);
  public:
    FeedbackController();
    void poseCallback(const geometry_msgs::Pose2D& x0);
    void goalCallback(const geometry_msgs::Pose2D& msg);
    void hazardCallback(const pcl_messages::CollisionTrajectory& traj);
    void cmdPub(const geometry_msgs::Twist cmd_vel);
  private:
};

FeedbackController::FeedbackController()
{
}

void FeedbackController::poseCallback(const geometry_msgs::Pose2D& x0)
{
  x_0.x = x0.x;
  x_0.y = x0.y;
  x_0.theta = x0.theta;

  float v = kd*pow(pow((x_g.x-x_0.x),2)+pow((x_g.y-x_0.y),2), 0.5);

  float bearing = atan2((x_g.y-x_0.y), (x_g.x-x_0.x));
  float diff_ang = fmin((2 * M_PI) - abs(bearing - x_0.theta), 1.0*abs(bearing - x_0.theta)); //diff_ang equivalent

  float turn_rate = -kh*diff_ang;

  //Implement hard limit on vel and angular rates

  cmd_vel.linear.x = v;
  cmd_vel.angular.z = turn_rate;

  FeedbackController::cmdPub(cmd_vel);

  if((x_g.x - x_0.x) < goal_threshold && (x_g.y-x_0.y) < goal_threshold)
  {
    collision_status == 0;
  }
}

void FeedbackController::goalCallback(const geometry_msgs::Pose2D& msg)
{
  if(collision_status==0)
  {
    x_g.x = msg.x;
    x_g.y = msg.y;
    x_g.theta = msg.theta;
  }
}

void FeedbackController::hazardCallback(const pcl_messages::CollisionTrajectory &traj)
{
  if(traj.collision_status != 0)
  {
    float angle = traj.angle;
    float dist = traj.distance;

    x_g.x = dist * cos(angle);
    x_g.y = dist * sin(angle);
    x_g.theta = x_0.theta + angle;

    collision_status++;
  }
}

void FeedbackController::cmdPub(const geometry_msgs::Twist cmd_vel)
{
  cmd_pub_.publish(cmd_vel);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feedback_controller");

  FeedbackController a;

  ros::spin();
}
