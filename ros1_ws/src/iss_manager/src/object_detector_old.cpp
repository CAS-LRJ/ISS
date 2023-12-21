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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

// Global variables to store camera info
double fx, fy, cx, cy;
bool camera_info_received = false;

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info) {
  fx = camera_info->K[0];
  fy = camera_info->K[4];
  cx = camera_info->K[2];
  cy = camera_info->K[5];
  camera_info_received = true;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg,
                   image_transport::Publisher& image_pub,
                   tf::TransformListener& listener) {
  if (!camera_info_received) return;

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Example: process a single LiDAR point (replace with actual data)
  tf::Vector3 lidar_point(1.0, 0.1, 0.1);  // Replace with actual LiDAR point
  tf::Vector3 camera_point;

  try {
    tf::StampedTransform transform;
    listener.lookupTransform("/ego_vehicle/depth_camera", "/ego_vehicle/base_link",
                             ros::Time(0), transform);
    camera_point = transform * lidar_point;
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

//   std::cout << "Camera point: " << camera_point.x() << " " << camera_point.y()
//             << " " << camera_point.z() << std::endl;

  if (camera_point.z() > 0) {
    int u = static_cast<int>((camera_point.x() * fx) / camera_point.z() + cx);
    int v = static_cast<int>((camera_point.y() * fy) / camera_point.z() + cy);
    int width = cv_ptr->image.cols;   // Image width
    int height = cv_ptr->image.rows;  // Image height
    
    // std::cout << "Image size: " << width << "x" << height << std::endl;
    // std::cout << u << " " << v << std::endl;
    // Draw the point on the image
    cv::circle(cv_ptr->image, cv::Point(u, v), 5, CV_RGB(255, 0, 0), -1);
  }

  // Publish the modified image
  image_pub.publish(cv_ptr->toImageMsg());
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                   ros::Publisher& cloud_pub, ros::Publisher& marker_pub,
                   tf::TransformListener& listener) {
  PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // Transform PointCloud to base_link frame
  PointCloudXYZ::Ptr transformed_cloud(new PointCloudXYZ);
  if (!pcl_ros::transformPointCloud("ego_vehicle/base_link", *cloud,
                                    *transformed_cloud, listener)) {
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

  // Convert the transformed PointCloud back to sensor_msgs::PointCloud2
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*transformed_cloud, output);
  output.header.frame_id =
      "ego_vehicle/base_link";  // Set the frame ID to base_link

  // Publish the transformed cloud for visualization in RViz
  cloud_pub.publish(output);

  // Voxel Grid filter for downsampling
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(transformed_cloud);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*transformed_cloud);

  // KdTree for clustering
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(transformed_cloud);

  // Euclidean clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.05);  // 5cm, modify as needed
  ec.setMinClusterSize(50);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(transformed_cloud);
  ec.extract(cluster_indices);

  // Create markers for each cluster
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
    marker.header.frame_id = "ego_vehicle/base_link";
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
  }

  marker_pub.publish(marker_array);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_detection_node");
  ros::NodeHandle nh;

  tf::TransformListener listener(ros::Duration(10));  // Buffer size for tf

  ros::Publisher cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1);
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>("bounding_box_markers", 1);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "/velodyne_points", 1,
      boost::bind(cloudCallback, _1, boost::ref(cloud_pub),
                  boost::ref(marker_pub), boost::ref(listener)));

  ros::Subscriber camera_info_sub = nh.subscribe(
      "/ego_vehicle/depth_camera/rgb/camera_info", 1, cameraInfoCallback);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise("/projected_image", 1);
  image_transport::Subscriber image_sub =
      it.subscribe("/ego_vehicle/depth_camera/rgb/image_raw", 1,
                   boost::bind(imageCallback, _1, boost::ref(image_pub),
                               boost::ref(listener)));
  ros::spin();
  return 0;
}
