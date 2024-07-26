#include <cmath>
#include <iostream>
#include <laser_geometry/laser_geometry.h>
// #include <pcl/common/intersections.h>
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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <utility>
#include <vector>

ros::Publisher pointPub;
ros::Publisher pointfPub;

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
  pcl::PointCloud<pcl::PointXYZ> clouldPassFiltered;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  sensor_msgs::PointCloud2 output;
  //把ROS消息转化为pcl
  pcl::fromROSMsg(pointCloud2, cloud);

  // pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer");
  // viewer1.showCloud(cloud.makeShared());
  // while (!viewer1.wasStopped()) {
  // }
  //直通滤波
  pcl::PassThrough<pcl::PointXYZ> passFilter;
  passFilter.setInputCloud(cloud.makeShared());
  passFilter.setFilterFieldName("y");
  passFilter.setFilterLimits(-0.4, 0.4);
  passFilter.filter(clouldPassFiltered);
  // pcl::visualization::CloudViewer viewer2("Simple Cloud Viewer");
  // viewer2.showCloud(clouldPassFiltered.makeShared());
  // while (!viewer2.wasStopped()) {
  // }

  //定义一个滤波分析算法
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
  statFilter.setInputCloud(clouldPassFiltered.makeShared()); // cloud传入
  statFilter.setMeanK(20);
  statFilter.setStddevMulThresh(1.0);
  statFilter.filter(cloud_filtered); // cloud_filtered传出

  pcl::toROSMsg(cloud_filtered, output);
  pointfPub.publish(output);

  pcl::PointCloud<pcl::PointXYZ>::Ptr testcloud(
      new pcl::PointCloud<pcl::PointXYZ>(cloud_filtered));
  // 剩余点云的初始设置
  pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(
      new pcl::PointCloud<pcl::PointXYZ>(*testcloud));
  std::cout << "加载点云个数：" << testcloud->points.size() << std::endl;
  std::vector<Eigen::VectorXf> vectorXf;

  while (remaining_cloud->points.size() > 20) {
    std::cout << "剩余点云个数：" << remaining_cloud->points.size()
              << std::endl;
    //... populate cloud
    // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    // viewer.showCloud(remaining_cloud);
    // while (!viewer.wasStopped()) {
    // }
    // std::cout << std::endl;
    std::vector<int> inliers; // 存储内点索引的向量
#if 0
    // 空间直线拟合
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr modelLine(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(
            remaining_cloud)); // 指定拟合点云与几何模型
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(
        modelLine);                    // 创建随机采样一致性对象
    ransac.setDistanceThreshold(0.05); // 内点到模型的最大距离
    ransac.setMaxIterations(3000);     // 最大迭代次数
    // ransac.setMinInliers(30);          // 调整最小内点数目
    // ransac.setProbability(0.99); // 增加成功概率

    ransac.computeModel();      // 执行RANSAC空间直线拟合
    // std::vector<int> inliers;   // 存储内点索引的向量
    ransac.getInliers(inliers); // 提取内点对应的索引
    if (inliers.size() < 20)    // 如果内点数小于阈值，结束检测
    {
      std::cout << "break" << std::endl;
      break;
    }
    /// 根据索引提取内点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLine(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud<pcl::PointXYZ>(*testcloud, inliers, *cloudLine);
    std::cout << "first idx " << inliers.front() << ", end idx "
              << inliers.back() << " , size "
              << inliers.back() - inliers.front() + 1 << std::endl;
    // 模型参数
    Eigen::VectorXf coefficient;
    ransac.getModelCoefficients(coefficient);
    vectorXf.push_back(coefficient);

#else
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr p_model(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(remaining_cloud));
    // 100 代表点到直线的距离阈值, 根据实际情况调整
    pcl::LeastMedianSquares<pcl::PointXYZ> lms(p_model, 0.01);
    lms.computeModel();
    lms.getInliers(inliers); // 提取内点对应的索引
    if (inliers.size() < 20) // 如果内点数小于阈值，结束检测
    {
      std::cout << "break" << std::endl;
      break;
    }
    std::cout << "first idx " << inliers.front() << ", end idx "
              << inliers.back() << " , size "
              << inliers.back() - inliers.front() + 1 << std::endl;
    Eigen::VectorXf coefficient;
    lms.getModelCoefficients(coefficient);
    vectorXf.push_back(coefficient);

#endif
    // std::cout << "直线点向式方程为：\n"
    //           << "   (x - " << coefficient[0] << ") / " << coefficient[3]
    //           << " = (y - " << coefficient[1] << ") / " << coefficient[4]
    //           << " = (z - " << coefficient[2] << ") / " << coefficient[5]
    //           << std::endl;
    if (vectorXf.size() >= 2) {
      for (auto it = vectorXf.begin(); it != std::prev(vectorXf.end()); it++) {
        auto test = angleBetweenVectorsXY(*it, *(std::next(it)));
        std::cout << "angle : " << test << std::endl;
        // Eigen::Vector4f point;
        // bool result =
        //     pcl::lineWithLineIntersection(*it, *(std::next(it)), point);
        // std::cout << "result : " << result << std::endl;
        Eigen::Vector4f pointa;
        Eigen::Vector4f pointb;
        pcl::lineToLineSegment(*it, *(std::next(it)), pointa, pointb);
        std::cout << "pointa : " << pointa[0] << " , " << pointa[1] << " , "
                  << pointa[2] << std::endl;
        std::cout << "pointb : " << pointb[0] << " , " << pointb[1] << " , "
                  << pointb[2] << std::endl;
      }
    }
    // 从剩余点云中移除检测到的内点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices);
    inliers_ptr->indices = inliers;
    extract.setInputCloud(remaining_cloud);
    extract.setIndices(inliers_ptr);
    extract.setNegative(true); // 设为true以提取剩余点
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud_new(
        new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*remaining_cloud_new);
    remaining_cloud = remaining_cloud_new;
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(remaining_cloud);
    while (!viewer.wasStopped()) {
    }
  }

#if 0
  for (size_t i = 0; i < scan->ranges.size(); i++) {
    Point tempPoint{};
    tempPoint.x =
        cos(scan->angle_min + i * scan->angle_increment) * scan->ranges[i];
    tempPoint.y =
        sin(scan->angle_min + i * scan->angle_increment) * scan->ranges[i];
    tempPoint.idx = i;
    points.push_back(tempPoint);
  }

  auto rightAngles = detectRightAngles(points);
  // 打印检测到的直角特征
  if (!rightAngles.empty()) {
    for (const auto &pair : rightAngles) {
      std::cout << "idx : " << pair.first.idx << " , " << pair.second.idx
                << std::endl;
      std::cout << "Right angle between points: (" << pair.first.x << ", "
                << pair.first.y << ") and (" << pair.second.x << ", "
                << pair.second.y << ")\n";
    }
    ros::Time endTime = ros::Time::now();
    std::cout << "time : " << endTime.toSec() - nowTime.toSec() << std::endl;

    //根据角点反向推算V形另外两个端点(斜率)，计算距离

  } else {
    // std::cout << "failed !" << std::endl;
  }
#endif
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ransac");
  ros::NodeHandle nh;
  ros::Subscriber laserSub;
  laserSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, laserCB);
  pointPub = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 10);
  pointfPub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_filter", 10);

  ros::spin();

  // // 示例点云数据
  // std::vector<Point> points = {{0, 0}, {1, 0}, {1, 1}, {0, 1},
  //                              {2, 2}, {3, 2}, {3, 3}, {2, 3}};

  // std::vector<std::pair<Point, Point>> rightAngles =
  // detectRightAngles(points);

  return 0;
}
