#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class PoseGridMap {
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber pose_sub_;
  ros::Publisher grid_map_pub_;
  
  std::string frame_id_;
  float xmin_, ymin_, xmax_, ymax_, delta_;
  int width_, height_;
  int **presence_map_;
  nav_msgs::OccupancyGrid grid_map_;
  
public:
  PoseGridMap();
  ~PoseGridMap();
  
  void poseCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array);
};

PoseGridMap::PoseGridMap() {
  pose_sub_ = node_handle_.subscribe<geometry_msgs::PoseArray>("/people_tracker/trajectory_acc", 1, &PoseGridMap::poseCallback, this);
  
  ros::NodeHandle private_nh("~");
  grid_map_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
  
  private_nh.param<std::string>("frame_id", frame_id_, "odom");
  private_nh.param<float>("xmin", xmin_, -10.0);
  private_nh.param<float>("ymin", ymin_, -10.0);
  private_nh.param<float>("xmax", xmax_, 10.0);
  private_nh.param<float>("ymax", ymax_, 10.0);
  private_nh.param<float>("delta", delta_, 0.2);
  
  width_ = abs(xmin_ > 0 ? ceil(xmin_/delta_) : floor(xmin_/delta_)) + abs(xmax_ > 0 ? ceil(xmax_/delta_) : floor(xmax_/delta_));
  height_ = abs(ymin_ > 0 ? ceil(ymin_/delta_) : floor(ymin_/delta_)) + abs(ymax_ > 0 ? ceil(ymax_/delta_) : floor(ymax_/delta_));
  std::cerr << "width_ = " << width_ << ", height_ = " << height_ << std::endl;
  
  presence_map_ = new int *[height_];
  for(int i = 0; i < height_; i++) {
    presence_map_[i] = new int [width_];
  }
  for(int i = 0; i < height_; i++) {
    for(int j = 0; j < width_; j++) {
      presence_map_[i][j] = 0;
    }
  }
}

PoseGridMap::~PoseGridMap() {
  for(int i = 0; i < height_; i++) {
    delete [] presence_map_[i];
  }
  delete presence_map_;
}

void PoseGridMap::poseCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array) {
  for(int i = 0; i < pose_array->poses.size(); i++) {
    if(pose_array->poses[i].position.x >= xmin_ && pose_array->poses[i].position.x <= xmax_ &&
       pose_array->poses[i].position.y >= ymin_ && pose_array->poses[i].position.y <= ymax_) {
      int x = trunc(pose_array->poses[i].position.x/delta_) + abs(xmin_ > 0 ? ceil(xmin_/delta_) : floor(xmin_/delta_));
      int y = trunc(pose_array->poses[i].position.y/delta_) + abs(ymin_ > 0 ? ceil(ymin_/delta_) : floor(ymin_/delta_));
      presence_map_[y][x]++;
    }
  }
  
  int min_p = INT_MAX, max_p = -INT_MAX;
  for(int x = 0; x < width_; x++) {
    for(int y = 0; y < height_; y++) {
      min_p = (std::min)(min_p, presence_map_[y][x]);
      max_p = (std::max)(max_p, presence_map_[y][x]);
    }
  }
  
  grid_map_.header.stamp = ros::Time::now();
  grid_map_.header.frame_id = frame_id_;
  grid_map_.info.resolution = delta_;
  grid_map_.info.width = width_;
  grid_map_.info.height = height_;
  grid_map_.info.origin.position.x = -((fabs(xmin_)+fabs(xmax_))/2.0);
  grid_map_.info.origin.position.y = -((fabs(ymin_)+fabs(ymax_))/2.0);
  grid_map_.info.origin.position.z = 0.0;
  grid_map_.info.origin.orientation.x = 0.0;
  grid_map_.info.origin.orientation.y = 0.0;
  grid_map_.info.origin.orientation.z = 0.0;
  grid_map_.info.origin.orientation.w = 1.0;
  
  grid_map_.data.resize(grid_map_.info.width * grid_map_.info.height);
  for(int x = 0; x < grid_map_.info.width; x++) {
    for(int y = 0; y < grid_map_.info.height; y++) {
      if(presence_map_[y][x] > 0) {
	grid_map_.data[MAP_IDX(grid_map_.info.width, x, y)] = int(100 * float(presence_map_[y][x] - min_p) / float(max_p - min_p));
      } else {
	grid_map_.data[MAP_IDX(grid_map_.info.width, x, y)] = -1;
      }
    }
  }
  grid_map_pub_.publish(grid_map_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_grid_map");
  PoseGridMap m;
  ros::spin();
  return 0;
}
