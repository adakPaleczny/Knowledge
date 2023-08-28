#include <ros/ros.h>
#include <ros/package.h>
#include <cstdlib>

#include <std_srvs/SetBool.h>

bool run_service(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response){
    if(request.data){
        std::string two_axis = "2";
        std::string three_axis = "3";

        // Getting path to my package
        std::string package_path = ros::package::getPath("my_calibration_pkg");
        // Specify the command to execute the bash script
        std::string file = package_path + "/scripts/calibration.sh";
        
        int result = system((file + " " + two_axis).c_str());

        if (result == 0){
            ROS_INFO("Bash script executed successfully.");
        }
        
        else{
            ROS_ERROR("Error executing bash script.");
            response.success = true;
            response.message = "Calibration done";
        }
        

    }
    else{
        response.success = false;
        response.message = "Doesn't work calibration";
    }
    
    

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_script_node");
    ros::NodeHandle n("~");

    ros::ServiceServer server = n.advertiseService("run_calibration_imu",run_service);

    ROS_INFO("Service started!");
    ros::spin();

    return 0;
}