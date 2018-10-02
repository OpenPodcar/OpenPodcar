#include <stdio.h>
#include <math.h>
#include <string>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
using namespace std;

//Podcar Gazebo-to-ROS plugin.   Simulates sensors and recieves wheel commands.
//Build with:  cmake . ; make    (NB not catkin -- just including it to get ROS libraries in)

namespace gazebo
{
  class MyPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
			int argc=0;
			char** argv;		
			ros::init(argc, argv, "ROSsetWheels");

			this->model = _parent;   //*er to my physical model, that I'm controlling
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MyPlugin::OnUpdate, this, _1));      //GAZEBO (not ROS) callback
 			this->_sub  = _n.subscribe("cmd_motors",  10, &MyPlugin::cmdMotorCallback,  (MyPlugin*)this);        //wheelspeeds. TODO could add noise here
			this->_pub_gnd  = _n.advertise<nav_msgs::Odometry>("odometry/groundTruth", 1000);   //TODO more acurate to sim the IMU and GPS ten fuse in EKF later on
			this->wL = 0.0;
			this->wR = 0.0;
			this->ticker = 0;
    }

    public: void cmdMotorCallback(const geometry_msgs::Twist& cmd_motor_msg) 
    {
      this->wL  = cmd_motor_msg.linear.x;     //not really twist, using struct forwheel speeds
      this->wR  = cmd_motor_msg.linear.y;
      cout << "odomCallback: " << this->wL<<" "<<this->wR <<endl;
    }

    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			//cout << "update " << endl;

			physics::JointPtr jointBL = this->model->GetJoint("back_left_joint");   //is z axis is still
			physics::JointPtr jointBR = this->model->GetJoint("back_right_joint");   //is z axis is still
			physics::JointPtr jointFL = this->model->GetJoint("front_left_joint");   //is z axis is still
		  	physics::JointPtr jointFR = this->model->GetJoint("front_right_joint");   //is z axis is still

			//TESTING
			//this->wL = 1.5;
			//this->wR = 2.0;
			
			double sf = 3.0;   //scale factor to speed up the sim!

			jointBL->SetVelocity(0, sf*this->wL);   
			jointBR->SetVelocity(0, sf*this->wR);  
			jointFL->SetVelocity(0, sf*this->wL); 
			jointFR->SetVelocity(0, sf*this->wR);

			//plugin is called a LOT -- reduce number of msgs out to stop ROS getting clogged
			this->ticker++;
			if (this->ticker>100)
			{
				this->ticker=0;
				//broadcast Odom mesg and tf containing robots ground truth pose (position and quaternion)
				//read pose of robot in 3D world
				math::Pose pose = this->model->GetWorldPose();
				math::Vector3& pos = pose.pos;
				math::Quaternion& quat = pose.rot;
				double x = pos.x;
				double y = pos.y;
				double z = pos.z;
				double qx = quat.x;
				double qy = quat.y;
				double qz = quat.z;
				double qw = quat.w;
				nav_msgs::Odometry odomOut;
				odomOut.header.stamp = ros::Time::now(); 
				odomOut.header.frame_id = "map";
				odomOut.pose.pose.position.x = x;
				odomOut.pose.pose.position.y = y;
				odomOut.pose.pose.position.z = z;    				//z=height
				odomOut.pose.pose.orientation.x = qx;       //from TF quat to ROS msg (yuk!)
				odomOut.pose.pose.orientation.y = qy;
				odomOut.pose.pose.orientation.z = qz;
				odomOut.pose.pose.orientation.w = qw;
				this->_pub_gnd.publish(odomOut);
			}
    }
	public: double wL;
	public: double wR;
	public: int ticker;
	public:   ros::NodeHandle _n;
    protected: ros::Subscriber _sub;
    protected: ros::Publisher _pub_gnd;
    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
  };
  GZ_REGISTER_MODEL_PLUGIN(MyPlugin)   //tell Gazebo I'm here
}
