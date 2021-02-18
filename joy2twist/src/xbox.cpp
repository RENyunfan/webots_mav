#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
class Xbox
{
public:
	Xbox(	ros::NodeHandle & nh);
private:
	void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
	ros::NodeHandle nh_;
    bool isStamped;
	float axesL1,axesL2,axesR1,axesR2;
	ros::Publisher pub;
	ros::Subscriber sub;
};

void Xbox::JoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(isStamped){
        geometry_msgs::TwistStamped twist;
        twist.header.frame_id = "world";
        twist.header.stamp = ros::Time::now();

        twist.twist.angular.x=0;
        twist.twist.angular.y=0;
        twist.twist.angular.z=msg->axes[0];
        twist.twist.linear.x=msg->axes[4];
        twist.twist.linear.y=msg->axes[3];
        twist.twist.linear.z=msg->axes[1];

        pub.publish(twist);

	}else{
        geometry_msgs::Twist twist;
        twist.angular.x=0;
        twist.angular.y=0;
        twist.angular.z=msg->axes[0];
        twist.linear.x=msg->axes[4];
        twist.linear.y=msg->axes[3];
        twist.linear.z=0;

        pub.publish(twist);
	}


}

Xbox::Xbox(	ros::NodeHandle & nh)
{
    nh_ = nh;
    nh.param<bool>("/xbox/use_stamped",isStamped,false);
    if(!isStamped)
	    pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    else
        pub = nh_.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);
	sub = nh_.subscribe<sensor_msgs::Joy>("/joy",10,&Xbox::JoyCallback,this);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "~");
	ros::NodeHandle nh;
	Xbox xbox(nh);

	ros::spin();
}
