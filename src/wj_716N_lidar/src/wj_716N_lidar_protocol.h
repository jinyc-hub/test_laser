#ifndef WJ_716N_LIDAR_PROTOCOL_H
#define WJ_716N_LIDAR_PROTOCOL_H
#include <iostream>
#include "string.h"
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/system/error_code.hpp>
#include <boost/bind/bind.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <wj_716N_lidar/wj_716N_lidarConfig.h>
using namespace std ;
namespace wj_lidar
{
#define MAX_LENGTH_DATA_PROCESS 200000
typedef struct TagDataCache
{
    unsigned char m_acdata[MAX_LENGTH_DATA_PROCESS];
    unsigned int m_u32in;
    unsigned int m_u32out;
}DataCache;

class wj_716N_lidar_protocol
{
public:
    wj_716N_lidar_protocol();
    bool dataProcess(unsigned char *data,const int reclen);
    bool protocl(unsigned char *data,const int len);
    bool OnRecvProcess(unsigned char *data, int len);
    bool checkXor(unsigned char *recvbuf, int recvlen);
    void send_scan(const char *data,const int len);
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    sensor_msgs::LaserScan scan;
    bool setConfig(wj_716N_lidar::wj_716N_lidarConfig &new_config,uint32_t level);
    bool heartstate;
private:
    void movedata(DataCache &sdata);
    DataCache   m_sdata;
    wj_716N_lidar::wj_716N_lidarConfig config_;
    unsigned int m_u32PreFrameNo;
    unsigned int m_u32ExpectedPackageNo;
    int m_n32currentDataNo;
    float scandata[1081];
    float scanintensity[1081];
    int total_point;
    int index_start;
    int index_end;
    int freq_scan;
};

}
#endif // WJ_716N_LIDAR_PROTOCOL_H
