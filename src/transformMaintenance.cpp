#include <cmath>
#include <ctime>
#include <cstdio>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

const double PI = 3.1415926;
const double rad2deg = 180 / PI;
const double deg2rad = PI / 180;

double timeOdomBefMapped;
double timeOdomAftMapped;

float transformSum[6] = {0};
float transformIncre[6] = {0};
float transformMapped[6] = {0};
float transformBefMapped[6] = {0};
float transformAftMapped[6] = {0};

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry2Pointer=nullptr;
tf2_ros::TransformBroadcaster* tfBroadcaster2Pointer=nullptr;
nav_msgs::msg::Odometry laserOdometry2;
geometry_msgs::msg::TransformStamped laserOdometryTrans2;

void transformAssociateToMap()
{
  float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
           - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
  float y1 = transformBefMapped[4] - transformSum[4];
  float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
           + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

  float x2 = x1;
  float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
  float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

  transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
  transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
  transformIncre[5] = z2;

  float sbcx = sin(transformSum[0]);
  float cbcx = cos(transformSum[0]);
  float sbcy = sin(transformSum[1]);
  float cbcy = cos(transformSum[1]);
  float sbcz = sin(transformSum[2]);
  float cbcz = cos(transformSum[2]);

  float sblx = sin(transformBefMapped[0]);
  float cblx = cos(transformBefMapped[0]);
  float sbly = sin(transformBefMapped[1]);
  float cbly = cos(transformBefMapped[1]);
  float sblz = sin(transformBefMapped[2]);
  float cblz = cos(transformBefMapped[2]);

  float salx = sin(transformAftMapped[0]);
  float calx = cos(transformAftMapped[0]);
  float saly = sin(transformAftMapped[1]);
  float caly = cos(transformAftMapped[1]);
  float salz = sin(transformAftMapped[2]);
  float calz = cos(transformAftMapped[2]);

  float srx = -sbcx * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly)
            - cbcx * cbcz * (calx * saly * (cbly * sblz - cblz * sblx * sbly)
            - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx)
            - cbcx * sbcz * (calx * caly * (cblz * sbly - cbly * sblx * sblz)
            - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz);
  transformMapped[0] = -asin(srx);

  float srycrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * saly * (cbly * sblz - cblz * sblx * sbly)
               - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx)
               - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz)
               - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz)
               + cbcx * sbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
  float crycrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz)
               - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz)
               - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * saly * (cbly * sblz - cblz * sblx * sbly)
               - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx)
               + cbcx * cbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
  transformMapped[1] = atan2(srycrx / cos(transformMapped[0]), crycrx / cos(transformMapped[0]));
    float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz)
               - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx)
               - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly)
               + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx)
               - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz
               + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz)
               + calx*cblx*salz*sblz);
  float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly)
               - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx)
               + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
               + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly)
               + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly
               - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz)
               - calx*calz*cblx*sblz);
  transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]), crzcrx / cos(transformMapped[0]));

  x1 = cos(transformMapped[2]) * transformIncre[3] - sin(transformMapped[2]) * transformIncre[4];
  y1 = sin(transformMapped[2]) * transformIncre[3] + cos(transformMapped[2]) * transformIncre[4];
  z1 = transformIncre[5];

  x2 = x1;
  y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
  z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

  transformMapped[3] = transformAftMapped[3]
                     - (cos(transformMapped[1]) * x2 + sin(transformMapped[1]) * z2);
  transformMapped[4] = transformAftMapped[4] - y2;
  transformMapped[5] = transformAftMapped[5]
                     - (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
}
void laserOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr laserOdometry)
{
  if (fabs(timeOdomBefMapped - timeOdomAftMapped) < 0.005) {
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
    tf2::Quaternion quat(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    transformSum[0] = -pitch;
    transformSum[1] = -yaw;
    transformSum[2] = roll;

    transformSum[3] = laserOdometry->pose.pose.position.x;
    transformSum[4] = laserOdometry->pose.pose.position.y;
    transformSum[5] = laserOdometry->pose.pose.position.z;

    transformAssociateToMap();

    quat.Quaternion::setEuler(-transformMapped[1], -transformMapped[0], transformMapped[2]);  

    laserOdometry2.header.stamp = laserOdometry->header.stamp;
    laserOdometry2.pose.pose.orientation.x = -quat.y();
    laserOdometry2.pose.pose.orientation.y = -quat.z();
    laserOdometry2.pose.pose.orientation.z = quat.x();
    laserOdometry2.pose.pose.orientation.w = quat.w();
    laserOdometry2.pose.pose.position.x = transformMapped[3];
    laserOdometry2.pose.pose.position.y = transformMapped[4];
    laserOdometry2.pose.pose.position.z = transformMapped[5];
    pubLaserOdometry2Pointer->publish(laserOdometry2);

    laserOdometryTrans2.header.stamp = laserOdometry->header.stamp;

    laserOdometryTrans2.transform.rotation.x = -quat.y();
    laserOdometryTrans2.transform.rotation.y= -quat.z();
    laserOdometryTrans2.transform.rotation.z = quat.x();
    laserOdometryTrans2.transform.rotation.w = quat.w();

    laserOdometryTrans2.transform.translation.x = transformMapped[3];
    laserOdometryTrans2.transform.translation.y = transformMapped[4];
    laserOdometryTrans2.transform.translation.z = transformMapped[5];


    tfBroadcaster2Pointer->sendTransform(laserOdometryTrans2);
  }
}
void odomBefMappedHandler(const nav_msgs::msg::Odometry::SharedPtr odomBefMapped)
{
  timeOdomBefMapped = odomBefMapped->header.stamp.sec + odomBefMapped->header.stamp.nanosec * 1e-9;

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomBefMapped->pose.pose.orientation;
  tf2::Quaternion quat(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  transformBefMapped[0] = -pitch;
  transformBefMapped[1] = -yaw;
  transformBefMapped[2] = roll;

  transformBefMapped[3] = odomBefMapped->pose.pose.position.x;
  transformBefMapped[4] = odomBefMapped->pose.pose.position.y;
  transformBefMapped[5] = odomBefMapped->pose.pose.position.z;
}

void odomAftMappedHandler(const nav_msgs::msg::Odometry::SharedPtr odomAftMapped)
{
  timeOdomAftMapped = odomAftMapped->header.stamp.sec + odomAftMapped->header.stamp.nanosec * 1e-9;

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
  tf2::Quaternion quat(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  transformAftMapped[0] = -pitch;
  transformAftMapped[1] = -yaw;
  transformAftMapped[2] = roll;

  transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
  transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
  transformAftMapped[5] = odomAftMapped->pose.pose.position.z;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("transformMaintenance");

  auto subLaserOdometry = node->create_subscription<nav_msgs::msg::Odometry>(        
    "/cam_to_init", rclcpp::QoS(5), laserOdometryHandler);

  auto subOdomBefMapped = node->create_subscription<nav_msgs::msg::Odometry>(        
    "/bef_mapped_to_init_2", rclcpp::QoS(5), odomBefMappedHandler);

  auto subOdomAftMapped = node->create_subscription<nav_msgs::msg::Odometry>(        
    "/aft_mapped_to_init_2", rclcpp::QoS(5), odomAftMappedHandler);

  auto pubLaserOdometry2 = node->create_publisher<nav_msgs::msg::Odometry>("/cam_to_init_2", rclcpp::SystemDefaultsQoS());     
  pubLaserOdometry2Pointer = pubLaserOdometry2;

  laserOdometry2.header.frame_id = "camera_init_2";
  laserOdometry2.child_frame_id = "camera";

  tf2_ros::TransformBroadcaster tfBroadcaster2(node);
  tfBroadcaster2Pointer = &tfBroadcaster2;
  laserOdometryTrans2.header.frame_id = "camera_init_2";
  laserOdometryTrans2.child_frame_id = "camera";

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}


  ros::spin();

  return 0;
}
