#include <ros/ros.h>
#include "async_client.h"
#include "wj_716N_lidar_protocol.h"
using namespace wj_lidar;

/* ------------------------------------------------------------------------------------------
 *  show demo --
 * ------------------------------------------------------------------------------------------ */
wj_716N_lidar_protocol *protocol;
Async_Client *client;

void callback(wj_716N_lidar::wj_716N_lidarConfig &config,uint32_t level)
{
    protocol->setConfig(config,level);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wj_716N_lidar_01");
    ros::NodeHandle nh("~");
    std::string hostname;
    nh.getParam("hostname",hostname);
    std::string port;
    nh.getParam("port",port);
    cout << "laser ip: " << hostname << ", port:" << port <<endl;

    protocol = new wj_716N_lidar_protocol();
    dynamic_reconfigure::Server<wj_716N_lidar::wj_716N_lidarConfig> server;
    dynamic_reconfigure::Server<wj_716N_lidar::wj_716N_lidarConfig>::CallbackType f;
    f = boost::bind(&callback,_1,_2);
    server.setCallback(f);

    client = new Async_Client(protocol);
    protocol->heartstate = false;
    while(!client->m_bConnected)
    {
        ROS_INFO("Start connecting laser!");
        if(client->connect(hostname.c_str(),atoi(port.c_str())))
        {
            ROS_INFO("Succesfully connected. Hello wj_716N_lidar!");
        }
        else
        {
            ROS_INFO("Failed to connect to laser. Waiting 5s to reconnect!");
        }
        ros::Duration(5).sleep();
    }
    while(ros::ok())
    {
        ros::spinOnce();
        ros::Duration(2).sleep();
        if(client->m_bConnected)
        {
            if(protocol->heartstate)
            {
                protocol->heartstate = false;
            }
            else
            {
                client->m_bConnected = false;
            }
        }
        else
        {
            //reconnect
            if(!client->m_bReconnecting)
            {
                boost::thread t(boost::bind(&Async_Client::reconnect, client));
            }
        }
    }
}
