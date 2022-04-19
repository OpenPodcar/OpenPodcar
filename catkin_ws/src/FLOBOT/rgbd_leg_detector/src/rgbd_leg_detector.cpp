// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
// PCL
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

class RGBDLegDetector {
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher marker_array_pub_;
  ros::Publisher pose_array_pub_;
  ros::Publisher point_cloud_pub_;
  
  std::string frame_id_;
  bool print_fps_;
  float leaf_size_;
  float min_z_dist_;
  float max_z_dist_;
  float min_y_dist_;
  float max_y_dist_;
  float min_leg_size_;
  float max_leg_size_;
  int min_cluster_size_;
  int max_cluster_size_;
  int bins_;
  float cosine_similarity_threshold_;
  float pair_legs_min_dist_;
  
public:
  RGBDLegDetector();
  ~RGBDLegDetector();
  
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2);
  
  void downSampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);
  void planeRemoving(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);
  void clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);
  void clustering2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);
};

RGBDLegDetector::RGBDLegDetector() {
  point_cloud_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1, &RGBDLegDetector::pointcloudCallback, this);
  
  ros::NodeHandle private_nh("~");
  marker_array_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("markers", 10);
  pose_array_pub_   = private_nh.advertise<geometry_msgs::PoseArray>("poses", 10);
  point_cloud_pub_  = private_nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
  
  private_nh.param<std::string>("frame_id", frame_id_, "camera_rgb_optical_frame");
  private_nh.param<bool>("print_fps", print_fps_, false);
  private_nh.param<float>("leaf_size", leaf_size_, 0.02);
  private_nh.param<float>("min_z_dist", min_z_dist_, 0);
  private_nh.param<float>("max_z_dist", max_z_dist_, 7.0);
  private_nh.param<float>("min_y_dist", min_y_dist_, 0.2);
  private_nh.param<float>("max_y_dist", max_y_dist_, 0.55);
  private_nh.param<float>("min_leg_size", min_leg_size_, 0.05);
  private_nh.param<float>("max_leg_size", max_leg_size_, 0.5);
  private_nh.param<int>("min_cluster_size", min_cluster_size_, 3);
  private_nh.param<int>("max_cluster_size", max_cluster_size_, 640*480);
  private_nh.param<int>("bins", bins_, 4); // A histogram of an image is produced first by discretization of the colors in the image into N bins.
  private_nh.param<float>("cosine_similarity_threshold", cosine_similarity_threshold_, 0.6);
  private_nh.param<float>("pair_legs_min_dist", pair_legs_min_dist_, 1.0);
}

RGBDLegDetector::~RGBDLegDetector() {
}

int frames;clock_t start_time;bool reset = true;//fps
void RGBDLegDetector::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2) {
  if(print_fps_)if(reset){frames=0;start_time=clock();reset=false;}//fps
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*ros_pc2, *pcl_pc);
  
  if(pcl_pc->size() == 0)
    return;
  
  downSampling(pcl_pc);
  planeRemoving(pcl_pc);
  clustering(pcl_pc);
  
  if(point_cloud_pub_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 ros_pc2_pub;
    pcl::toROSMsg(*pcl_pc, ros_pc2_pub);
    point_cloud_pub_.publish(ros_pc2_pub);
  }
  
  if(print_fps_)if(++frames>10){std::cerr<<"[rgbd_leg_detector]: fps = "<<float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC)<<std::endl;reset = true;}//fps
}

void RGBDLegDetector::downSampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud(pc);
  vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  vg.setFilterFieldName("z");
  vg.setFilterLimits(min_z_dist_, max_z_dist_);
  vg.filter(*cloud);
  pcl::copyPointCloud(*cloud, *pc);
}

void RGBDLegDetector::planeRemoving(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(leaf_size_);
  seg.setInputCloud(pc);
  seg.segment(*inliers, *coefficients);
  pcl::ExtractIndices<pcl::PointXYZRGB> ei;
  ei.setInputCloud(pc);
  ei.setIndices(inliers);
  ei.setNegative(true);
  ei.filterDirectly(pc);
}

void RGBDLegDetector::clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) {
  visualization_msgs::MarkerArray marker_array;
  geometry_msgs::PoseArray pose_array;
  
  pcl::IndicesPtr pc_indices(new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(pc);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(min_y_dist_, max_y_dist_);
  pass.filter(*pc_indices);
  
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(pc);
  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(leaf_size_*5);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pc);
  ec.setIndices(pc_indices);
  ec.extract(cluster_indices);
  
  float color_histograms[cluster_indices.size()][bins_*bins_*bins_];
  int cluster_id = 0;
  std::vector<Eigen::Vector4f> clusters_min, clusters_max, clusters_centroid;
  Eigen::Vector4f min, max, centroid;
  
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cluster->points.push_back(pc->points[*pit]);
    cluster->width = cluster->size();
    cluster->height = 1;
    cluster->is_dense = true;
    
    pcl::getMinMax3D(*cluster, min, max);
    pcl::compute3DCentroid(*cluster, centroid);
    
    // Suppose that legs are upright rectangles.
    if((max[1] - min[1]) / (max[0] - min[0]) < 1.0)
      continue;
    // Suppose that legs are within a reasonable size.
    if(max[0]-min[0] < min_leg_size_ || max[0]-min[0] > max_leg_size_ ||
       max[1]-min[1] < min_leg_size_ || max[1]-min[1] > max_leg_size_ ||
       max[2]-min[2] < min_leg_size_ || max[2]-min[2] > max_leg_size_)
      continue;
    
    // Using color histogram to find pair of legs.
    for(int i = 0; i < bins_*bins_*bins_; i++)
      color_histograms[cluster_id][i] = 0;
    for(int i = 0; i < cluster->size(); i++)
      color_histograms[cluster_id][cluster->points[i].r/bins_ + cluster->points[i].g/(bins_*bins_) + cluster->points[i].b/(bins_*bins_*bins_)]++;
    for(int i = 0; i < bins_*bins_*bins_; i++)
      color_histograms[cluster_id][i] /= cluster->size();
    
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id_;
    marker.ns = "cluster";
    marker.id = cluster_id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    geometry_msgs::Point p[24];
    p[0].x = max[0]; p[0].y = max[1]; p[0].z = max[2];
    p[1].x = min[0]; p[1].y = max[1]; p[1].z = max[2];
    p[2].x = max[0]; p[2].y = max[1]; p[2].z = max[2];
    p[3].x = max[0]; p[3].y = min[1]; p[3].z = max[2];
    p[4].x = max[0]; p[4].y = max[1]; p[4].z = max[2];
    p[5].x = max[0]; p[5].y = max[1]; p[5].z = min[2];
    p[6].x = min[0]; p[6].y = min[1]; p[6].z = min[2];
    p[7].x = max[0]; p[7].y = min[1]; p[7].z = min[2];
    p[8].x = min[0]; p[8].y = min[1]; p[8].z = min[2];
    p[9].x = min[0]; p[9].y = max[1]; p[9].z = min[2];
    p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
    p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
    p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
    p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
    p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
    p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
    p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
    p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
    p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
    p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
    p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
    p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
    p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
    p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
    for(int i = 0; i < 24; i++)
      marker.points.push_back(p[i]);
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration(0.1);
    marker_array.markers.push_back(marker);
    
    clusters_min.push_back(min);
    clusters_max.push_back(max);
    clusters_centroid.push_back(centroid);
    cluster_id++;
  }
  
  // Computing cosine similiarity of each cluster pair.
  float dot = 0, denom_a = 0, denom_b = 0;
  float closest_leg_dist;
  int closest_leg_id;
  std::vector<int> leg_id_list;
  
  for(int i = 0; i < cluster_id; i++) {
    if(std::find(leg_id_list.begin(), leg_id_list.end(), i) != leg_id_list.end())
      continue;
    closest_leg_id = i;
    closest_leg_dist = FLT_MAX;
    for(int j = 0; j < cluster_id-1-i; j++) {
      if(std::find(leg_id_list.begin(), leg_id_list.end(), i+1+j) != leg_id_list.end())
       	continue;
      for(int k = 0; k < bins_*bins_*bins_; k++) {
  	dot += color_histograms[i][k] * color_histograms[i+1+j][k];
  	denom_a += color_histograms[i][k] * color_histograms[i][k];
  	denom_b += color_histograms[i+1+j][k] * color_histograms[i+1+j][k];
      }
      if(dot / (std::sqrt(denom_a) * std::sqrt(denom_b)) > cosine_similarity_threshold_) {
	Eigen::Vector3f diff = clusters_centroid[i].head<3>() - clusters_centroid[i+1+j].head<3>();
	if(diff.norm() < pair_legs_min_dist_) {
	  if(diff.norm() < closest_leg_dist) {
	    closest_leg_id = i+1+j;
	    closest_leg_dist = diff.norm();
	  }
	}
      }
    }
    if(closest_leg_id != i) {
      leg_id_list.push_back(i);
      leg_id_list.push_back(closest_leg_id);
      
      geometry_msgs::Pose pose;
      pose.position.x = (clusters_centroid[i][0]+clusters_centroid[closest_leg_id][0])/2;
      pose.position.y = (clusters_centroid[i][1]+clusters_centroid[closest_leg_id][1])/2;
      pose.position.z = (clusters_centroid[i][2]+clusters_centroid[closest_leg_id][2])/2;
      pose.orientation.w = 1;
      pose_array.poses.push_back(pose);
      
      visualization_msgs::Marker leg0;
      leg0.header.stamp = ros::Time::now();
      leg0.header.frame_id = frame_id_;
      leg0.ns = "leg0";
      leg0.id = i;
      leg0.type = visualization_msgs::Marker::CUBE;
      leg0.pose.position.x = clusters_centroid[i][0];
      leg0.pose.position.y = clusters_centroid[i][1];
      leg0.pose.position.z = clusters_centroid[i][2];
      leg0.scale.x = clusters_max[i][0] - clusters_min[i][0];
      leg0.scale.y = clusters_max[i][1] - clusters_min[i][1];
      leg0.scale.z = clusters_max[i][2] - clusters_min[i][2];
      leg0.color.a = 0.3;
      leg0.color.r = 1.0;
      leg0.lifetime = ros::Duration(0.1);
      marker_array.markers.push_back(leg0);
      
      visualization_msgs::Marker leg1;
      leg1.header.stamp = ros::Time::now();
      leg1.header.frame_id = frame_id_;
      leg1.ns = "leg1";
      leg1.id = closest_leg_id;
      leg1.type = visualization_msgs::Marker::CUBE;
      leg1.pose.position.x = clusters_centroid[closest_leg_id][0];
      leg1.pose.position.y = clusters_centroid[closest_leg_id][1];
      leg1.pose.position.z = clusters_centroid[closest_leg_id][2];
      leg1.scale.x = clusters_max[closest_leg_id][0] - clusters_min[closest_leg_id][0];
      leg1.scale.y = clusters_max[closest_leg_id][1] - clusters_min[closest_leg_id][1];
      leg1.scale.z = clusters_max[closest_leg_id][2] - clusters_min[closest_leg_id][2];
      leg1.color.a = 0.3;
      leg1.color.r = 1.0;
      leg1.lifetime = ros::Duration(0.1);
      marker_array.markers.push_back(leg1);
      
      visualization_msgs::Marker leg_link;
      leg_link.header.stamp = ros::Time::now();
      leg_link.header.frame_id = frame_id_;
      leg_link.ns = "leg_link";
      leg_link.id = i+100;
      leg_link.type = visualization_msgs::Marker::LINE_STRIP;
      geometry_msgs::Point p0, p1;
      p0.x = clusters_centroid[i][0];
      p0.y = clusters_centroid[i][1];
      p0.z = clusters_centroid[i][2];
      leg_link.points.push_back(p0);
      p1.x = clusters_centroid[closest_leg_id][0];
      p1.y = clusters_centroid[closest_leg_id][1];
      p1.z = clusters_centroid[closest_leg_id][2];
      leg_link.points.push_back(p1);
      leg_link.scale.x = 0.03;
      leg_link.color.a = 1.0;
      leg_link.color.g = 1.0;
      leg_link.lifetime = ros::Duration(0.1);
      marker_array.markers.push_back(leg_link); 
    }
  }
  
  if(marker_array.markers.size()) {
    marker_array_pub_.publish(marker_array);
  }
  if(pose_array.poses.size()) {
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = frame_id_;
    pose_array_pub_.publish(pose_array);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rgbd_leg_detector");
  RGBDLegDetector d;
  ros::spin();
  return 0;
}
