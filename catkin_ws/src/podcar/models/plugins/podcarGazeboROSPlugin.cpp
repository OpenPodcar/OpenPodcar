#include <stdio.h>
#include <math.h>
#include <string>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <hardware_interface/joint_command_interface.h>
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
			ros::init(argc, argv, "GazeboPlugin");

			this->model = _parent;   //*er to my physical model, that I'm controlling
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MyPlugin::OnUpdate, this, _1));      //GAZEBO (not ROS) callback
			this->speedSub  = _n.subscribe("speedcmd_meterssec",  1, &MyPlugin::wheelSpeedCallback,  (MyPlugin*)this);        
			this->wheelAngleSub  = _n.subscribe("wheelAngleCmd",  1, &MyPlugin::wheelAngleCallback,  (MyPlugin*)this);
			this->_pub_gnd  = _n.advertise<nav_msgs::Odometry>("odometry/groundTruth", 1000);   //TODO more acurate to sim the IMU and GPS ten fuse in EKF later on
			this->ticker = 0;
			
			this-> timeOut = 2;
			this-> lastSpeedCmdTime = time(NULL);
			this-> lastAngleCmdTime = time(NULL);
			this-> timedOut = false;

			physics::JointPtr fR = this->model->GetJoint("front_right_joint");
			physics::JointPtr fL = this->model->GetJoint("front_left_joint");

			this->desiredEndAngle = 0;

			this->angleController = common::PID(1, 0, 1);

			physics::JointPtr trackingFrontRight = this->model->GetJoint("tracking_right_pivot_joint");
			physics::JointPtr trackingFrontLeft = this->model->GetJoint("tracking_left_pivot_joint");

			this->model->GetJointController()->SetPositionPID(trackingFrontLeft->GetScopedName(), this->angleController);
			this->model->GetJointController()->SetPositionPID(trackingFrontRight->GetScopedName(), this->angleController);
		}

		public: void wheelSpeedCallback(const std_msgs::Float64& msg){
			this-> timedOut = false;
			this->lastSpeedCmdTime = time(NULL);

			physics::JointPtr jointBL = this->model->GetJoint("back_left_joint");
			physics::JointPtr jointBR = this->model->GetJoint("back_right_joint");
			
			jointBL->SetVelocity(0, msg.data/0.17775); //divide by wheel radius
			jointBR->SetVelocity(0, msg.data/0.17775);
		}

		public: void wheelAngleCallback(const std_msgs::Float64& msg) {
			this-> timedOut = false;
			this->lastAngleCmdTime = time(NULL);

			physics::JointPtr jointFL = this->model->GetJoint("tracking_left_pivot_joint");
			physics::JointPtr jointFR = this->model->GetJoint("tracking_right_pivot_joint");

			this->model->GetJointController()->SetPositionTarget(jointFL->GetScopedName(), msg.data);
			this->model->GetJointController()->SetPositionTarget(jointFR->GetScopedName(), msg.data);
		}

		public: void brake(){
			physics::JointPtr jointBL = this->model->GetJoint("back_left_joint");
			physics::JointPtr jointBR = this->model->GetJoint("back_right_joint");

			jointBL->SetVelocity(0, 0);   
			jointBR->SetVelocity(0, 0);  

			physics::JointPtr jointFL = this->model->GetJoint("front_left_joint");
			physics::JointPtr jointFR = this->model->GetJoint("front_right_joint");

			jointFL->SetVelocity(0, 0);   
			jointFR->SetVelocity(0, 0);  
		}

		public: void OnUpdate(const common::UpdateInfo & /*_info*/){
			//cout << "update " << endl;

			std::time_t timeNow = time(NULL);
			if(timeNow >= this->lastSpeedCmdTime+this->timeOut && !this->timedOut){
				this-> timedOut = true;
				brake();
			}
			else if(timeNow >= this->lastAngleCmdTime+this->timeOut && !this->timedOut){
				this-> timedOut = true;
				brake();
			}

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

		public: int ticker;
		public: ros::NodeHandle _n;
		protected: ros::Subscriber speedSub;
		protected: ros::Subscriber wheelAngleSub;
		protected: ros::Publisher _pub_gnd;
		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
		private: std::time_t lastSpeedCmdTime;
		private: std::time_t lastAngleCmdTime;
		private: std::time_t timeOut;
		private: bool timedOut;

		public: common::PID speedController;
		public: common::PID angleController;

		private: float desiredEndAngle;
		private: common::Time prevUpdateTime;
		private: physics::JointPtr fR;
		private: physics::JointPtr fL;
	};

	GZ_REGISTER_MODEL_PLUGIN(MyPlugin)   //tell Gazebo I'm here
}
