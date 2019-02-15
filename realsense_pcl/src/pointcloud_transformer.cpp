#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

tf2_ros::Buffer tf_buffer_;
tf2_ros::TransformListener tf_listener_;

class TransformPointCloud
{
	ros::NodeHandle nh_;
  ros::Subscriber sub_;
	ros::Publisher pub_;
	
	//tf2_ros::Buffer tf_buffer_;
	//tf2_ros::TransformListener tf_listener_;
	
	
	void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		geometry_msgs::TransformStamped transform;
		
		try
		{		
		
		transform = tf_buffer_.lookupTransform(
				      "base_link",
				      "camera_link",
				      msg->header.stamp,
							ros::Duration(10));
		
		sensor_msgs::PointCloud2 cloud_out;
		tf2::doTransform(*msg, cloud_out, transform);
		pub_.publish(cloud_out);
		}
		
		catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
		}
	}
	
	public:
		TransformPointCloud()
		{
				pub_ = nh_.advertise<sensor_msgs::PointCloud2>("rover_cloud", 3);
				sub_ = nh_.subscribe("camera/depth_registered/points", 1, &TransformPointCloud::pointCloudCallback, this);
		}
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_point_cloud");
	tf_listener_(tf_buffer_);
  TransformPointCloud transform_point_cloud;
  ros::spin();
}



