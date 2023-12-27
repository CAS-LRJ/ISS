#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

class ObjectDetector {
 public:
  ObjectDetector(ros::NodeHandle& nh, image_transport::ImageTransport& it)
      : _listener(ros::Duration(10)) {
    _name_space = nh.getNamespace();
    _name_space.erase(0, 1);
    _cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
        "object_detector/filtered_cloud", 1);
    _marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "object_detector/markers", 1);
    _lidar_points_sub = nh.subscribe(
        "/velodyne_points", 1, &ObjectDetector::lidarPointsCallback, this);
    _camera_info_sub = nh.subscribe("depth_camera/rgb/camera_info", 1,
                                    &ObjectDetector::cameraInfoCallback, this);
    _image_sub = it.subscribe("depth_camera/rgb/image_raw", 1,
                              &ObjectDetector::imageCallback, this);
    _projected_image_pub = it.advertise("object_detector/projected_image", 1);
  }

 private:
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info) {
    fx = camera_info->K[0];
    fy = camera_info->K[4];
    cx = camera_info->K[2];
    cy = camera_info->K[5];
    camera_info_received = true;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!camera_info_received) return;
    try {
      received_image =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      image_received = true;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  void lidarPointsCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert ROS message to PCL point cloud
    PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Transform PointCloud to base_link frame
    PointCloudXYZ::Ptr transformed_cloud(new PointCloudXYZ);
    if (!pcl_ros::transformPointCloud(_name_space + "/base_link", *cloud,
                                      *transformed_cloud, _listener)) {
      ROS_ERROR("Error transforming point cloud to base_link frame");
      return;
    }

    // Filter the point cloud to the region in front of the robot
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(transformed_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-2, 2.0);  // Modify these limits as needed
    pass.filter(*transformed_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.5, 0.5);  // Modify these limits as needed
    pass.filter(*transformed_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.02, 1);  // Modify these limits as needed
    pass.filter(*transformed_cloud);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*transformed_cloud, output);
    output.header.frame_id = _name_space + "/base_link";
    _cloud_pub.publish(output);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(transformed_cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*transformed_cloud);

    if (transformed_cloud->points.size() < 5) return;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(transformed_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05);  // 5cm, modify as needed
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(transformed_cloud);
    ec.extract(cluster_indices);

    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    for (const auto& indices : cluster_indices) {
      PointCloudXYZ::Ptr cluster(new PointCloudXYZ);
      for (const auto& idx : indices.indices) {
        cluster->points.push_back(transformed_cloud->points[idx]);
      }
      // Calculate bounding box
      pcl::PointXYZ min_pt, max_pt;
      pcl::getMinMax3D(*cluster, min_pt, max_pt);
      visualization_msgs::Marker marker;
      marker.header.frame_id = _name_space + "/base_link";
      marker.header.stamp = ros::Time::now();
      marker.ns = "object_clusters";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
      marker.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
      marker.pose.position.z = (min_pt.z + max_pt.z) / 2.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = (max_pt.x - min_pt.x);
      marker.scale.y = (max_pt.y - min_pt.y);
      marker.scale.z = (max_pt.z - min_pt.z);
      marker.color.a = 0.5;  // Transparency
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker_array.markers.push_back(marker);
      tf::Vector3 lidar_point(marker.pose.position.x, marker.pose.position.y,
                              marker.pose.position.z);
      if (image_received)
        lidar_point_to_image(lidar_point);
    }
    _marker_pub.publish(marker_array);
    if (image_received)
      _projected_image_pub.publish(received_image->toImageMsg());
  }

  void lidar_point_to_image(tf::Vector3& lidar_point) {
    tf::Vector3 camera_point;
    try {
      tf::StampedTransform transform;
      _listener.lookupTransform(_name_space + "/depth_camera",
                               _name_space + "/base_link", ros::Time(0),
                               transform);
      camera_point = transform * lidar_point;
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    if (camera_point.z() > 0) {
      int u = static_cast<int>((camera_point.x() * fx) / camera_point.z() + cx);
      int v = static_cast<int>((camera_point.y() * fy) / camera_point.z() + cy);
      int width = received_image->image.cols;   // Image width
      int height = received_image->image.rows;  // Image height
      cv::circle(received_image->image, cv::Point(u, v), 5, CV_RGB(255, 0, 0),
                 -1);
    }
  }

  ros::Publisher _cloud_pub;
  ros::Publisher _marker_pub;
  ros::Subscriber _lidar_points_sub;
  ros::Subscriber _camera_info_sub;
  image_transport::Subscriber _image_sub;
  image_transport::Publisher _projected_image_pub;
  tf::TransformListener _listener;

  std::string _name_space;
  bool camera_info_received = false;
  bool image_received = false;
  double fx, fy, cx, cy;
  cv_bridge::CvImagePtr received_image;
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_detector");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ObjectDetector object_detector(nh, it);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}