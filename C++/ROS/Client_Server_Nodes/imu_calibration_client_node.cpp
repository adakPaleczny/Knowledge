#include <ros/ros.h>
#include <cstdlib>

#include <std_srvs/SetBool.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_script_client");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("/imu_script_node/run_calibration_imu");

    std_srvs::SetBool setBool;
    setBool.request.data = true;

    if(client.call(setBool)){
        ROS_INFO("Service called");
        ROS_INFO("%s",setBool.response.message.c_str());
    }
    else{
        ROS_INFO("Service did not call");
    }

    return 0;
}