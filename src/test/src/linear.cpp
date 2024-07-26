#include <cmath>
#include <iostream>
#include <laser_geometry/laser_geometry.h>
#include <pcl/common/intersections.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <utility>
#include <vector>

ros::Publisher pointPub;
ros::Publisher pointfPub;
ros::Publisher linearPub;

// 定义一个结构来存储点
struct Point {
  double x, y;
  int idx;
};

// 计算两点之间的距离
double distance(const Point &p1, const Point &p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

//计算两个向量之间的夹角
double angleBetweenVectorsXY(const Eigen::VectorXf &line1,
                             const Eigen::VectorXf &line2) {
  // std::cout << "size = " << line1.size() << std::endl;
  double temp1 = line1[3] * line1[1] - line1[0] * line1[4];
  double temp2 = line2[3] * line2[1] - line2[0] * line2[4];
  Eigen::Vector2f line1_2d(temp1 / line1[4], temp1 / line1[3]);
  Eigen::Vector2f line2_2d(temp2 / line2[4], temp2 / line2[3]);

  auto line_product = line1_2d.dot(line2_2d);
  return acos(line_product / (line1_2d.norm() * line2_2d.norm())) *
         (180 / M_PI);
}

// 计算两个向量之间的夹角
double angleBetweenVectors(const Point &p1, const Point &p2, const Point &p3) {
  double dotProduct =
      (p2.x - p1.x) * (p3.x - p2.x) + (p2.y - p1.y) * (p3.y - p2.y);
  double magnitude1 = distance(p1, p2);
  double magnitude2 = distance(p2, p3);
  return acos(dotProduct / (magnitude1 * magnitude2)) *
         (180.0 / M_PI); // 返回角度
}

// 计算点到直线的距离
double distanceBetweenPointAndLine(const Point &point,
                                   const Eigen::VectorXf &line) {
  // ax+by+c = 0
  auto a = line[4];
  auto b = -1 * line[3];
  auto c = line[3] * line[1] - line[0] * line[4];
  return std::fabs((a * point.x + b * point.y + c) / sqrt(a * a + b * b));
}

// 检测点云中的直角特征
std::vector<std::pair<Point, Point>>
detectRightAngles(const std::vector<Point> &points) {
  std::vector<std::pair<Point, Point>> rightAngles;
  double threshold = 0.5; // 角度误差容忍度

  for (auto it = points.begin(); it != points.end() - 2; it++) {
    double angle = angleBetweenVectors(*it, *(it + 1), *(it + 2));
    if (it->x > 0.1 && (it + 2)->x > 0.1 && it->y > 0.1 && (it + 2)->y > 0.1) {
      if (std::abs(angle - 90.0) < threshold) {
        std::cout << " angle : " << angle << std::endl;
        rightAngles.emplace_back(*it, *(it + 2));
      }
    }
  }
  return rightAngles;
}

void passThroughFilter(const pcl::PointCloud<pcl::PointXYZ> &inputCloud,
                       pcl::PointCloud<pcl::PointXYZ> &outputCloud,
                       const std::string &field, float limitMin,
                       float limitMax) {
  pcl::PassThrough<pcl::PointXYZ> passFilter;
  passFilter.setInputCloud(inputCloud.makeShared());
  passFilter.setFilterFieldName(field);
  passFilter.setFilterLimits(limitMin, limitMax);
  passFilter.filter(outputCloud);
}

void SORFilter(const pcl::PointCloud<pcl::PointXYZ> &inputCloud,
               pcl::PointCloud<pcl::PointXYZ> &outputCloud, int k,
               double mult) {
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
  statFilter.setInputCloud(inputCloud.makeShared()); // cloud传入
  statFilter.setMeanK(k);
  statFilter.setStddevMulThresh(mult);
  statFilter.filter(outputCloud); // cloud_filtered传出
}

void laserCB(const sensor_msgs::LaserScan::ConstPtr &scan) {
  ros::Time nowTime = ros::Time::now();
  std::vector<Point> points{};
  laser_geometry::LaserProjection projection;
  tf::TransformListener listener;
  if (!listener.waitForTransform(
          scan->header.frame_id, "/laser",
          scan->header.stamp + ros::Duration().fromSec(scan->ranges.size() *
                                                       scan->time_increment),
          ros::Duration(1.0))) {
    return;
  }
  sensor_msgs::PointCloud2 pointCloud2;
  projection.transformLaserScanToPointCloud("laser", *scan, pointCloud2,
                                            listener, 3.0);
  pointPub.publish(pointCloud2);

  //创建接收点云 发出点云 发出消息的变量
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> clouldPassFiltered;
  pcl::PointCloud<pcl::PointXYZ> cloudFiltered;
  sensor_msgs::PointCloud2 output;
  //把ROS消息转化为pcl
  pcl::fromROSMsg(pointCloud2, cloud);

  // pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer");
  // viewer1.showCloud(cloud.makeShared());
  // while (!viewer1.wasStopped()) {
  // }
  //直通滤波
  passThroughFilter(cloud, clouldPassFiltered, "y", -0.4, 0.4);
  // pcl::PassThrough<pcl::PointXYZ> passFilter;
  // passFilter.setInputCloud(cloud.makeShared());
  // passFilter.setFilterFieldName("y");
  // passFilter.setFilterLimits(-0.4, 0.4);
  // passFilter.filter(clouldPassFiltered);

  // pcl::visualization::CloudViewer viewer2("Simple Cloud Viewer");
  // viewer2.showCloud(clouldPassFiltered.makeShared());
  // while (!viewer2.wasStopped()) {
  // }

  //定义一个滤波分析算法
  SORFilter(clouldPassFiltered, cloudFiltered, 20, 1.0);
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
  // statFilter.setInputCloud(clouldPassFiltered.makeShared()); // cloud传入
  // statFilter.setMeanK(20);
  // statFilter.setStddevMulThresh(1.0);
  // statFilter.filter(cloudFiltered); // cloud_filtered传出

  pcl::toROSMsg(cloudFiltered, output);
  pointfPub.publish(output);

  pcl::PointCloud<pcl::PointXYZ>::Ptr testcloud(
      new pcl::PointCloud<pcl::PointXYZ>(cloudFiltered));
  // 剩余点云的初始设置
  pcl::PointCloud<pcl::PointXYZ>::Ptr remainingCloud(
      new pcl::PointCloud<pcl::PointXYZ>(*testcloud));
  std::cout << "加载点云个数：" << testcloud->points.size() << std::endl;
  std::vector<Eigen::VectorXf> vectorXf;
  Point originPoint;
  originPoint.x = 0;
  originPoint.y = 0;
  while (remainingCloud->points.size() > 20) {
    std::cout << "剩余点云个数：" << remainingCloud->points.size() << std::endl;
    // pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer");
    // viewer1.showCloud(remainingCloud);
    // while (!viewer1.wasStopped()) {
    // }

    std::vector<int> inliers; // 存储内点索引的向量

    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr linearModel(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(remainingCloud));
    pcl::LeastMedianSquares<pcl::PointXYZ> lms(linearModel, 0.01);
    lms.computeModel();
    lms.getInliers(inliers); // 提取内点对应的索引

    // 从剩余点云中移除检测到的内点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices);
    inliers_ptr->indices = inliers;
    extract.setInputCloud(remainingCloud);
    extract.setIndices(inliers_ptr);
    extract.setNegative(true); // 设为true以提取剩余点
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*tempCloud);
    remainingCloud = tempCloud;

    if (inliers.size() < 60 ||
        inliers.size() > 2000) // 如果内点数小于阈值，结束检测
    {
      std::cout << "continue" << std::endl;
      continue;
    }
    // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    // viewer.showCloud(remainingCloud);
    // while (!viewer.wasStopped()) {
    // }

    std::cout << "first idx " << inliers.front() << ", end idx "
              << inliers.back() << " , size "
              << inliers.back() - inliers.front() + 1 << std::endl;
    Eigen::VectorXf coefficient;
    lms.getModelCoefficients(coefficient);
    vectorXf.push_back(coefficient);
    // std::cout << "直线点向式方程为：\n"
    //           << "   (x - " << coefficient[0] << ") / " << coefficient[3]
    //           << " = (y - " << coefficient[1] << ") / " << coefficient[4]
    //           << std::endl;
    double k = (std::abs(coefficient[3]) > 1e-6)
                   ? (coefficient[4] / coefficient[3])
                   : std::numeric_limits<double>::infinity();
    auto theta = std::atan(k) * 180 / M_PI;

    if (std::fabs(theta) >= 90 || std::fabs(theta) <= 80) {
      continue;
    }
    std::cout << " K : " << k << std::endl;
    std::cout << " theta : " << theta << std::endl;
    auto distance = distanceBetweenPointAndLine(originPoint, coefficient);
    std::cout << "distance : " << distance << std::endl;
    std_msgs::String str;
    str.data = std::to_string(theta) + "," + std::to_string(distance);
    linearPub.publish(str);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ransac");
  ros::NodeHandle nh;
  ros::Subscriber laserSub;
  laserSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, laserCB);
  pointPub = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 10);
  pointfPub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_filter", 10);
  linearPub = nh.advertise<std_msgs::String>("/linear", 10);

  ros::spin();
  return 0;
}
