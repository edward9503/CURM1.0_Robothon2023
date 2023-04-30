/*
    Flexiv Impedance Control: Initial Version with Primitive Control
*/

#include <ros/ros.h>
#include <std_msgs/String.h>

std_msgs::String task_status;

void CallbackFunc(const std_msgs::StringConstPtr msg){
    // subscribe the task status
    task_status.data = msg->data;
    return ;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "flexiv_init_imp");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    ros::Publisher primitive_cmd_publisher = nh.advertise<std_msgs::String>("/arm_primitive_cmd", 1);
    // need to check the returned status
    ros::Subscriber primitive_status_subscriber = nh.subscribe("/arm_task_status", 1, CallbackFunc);

    ROS_INFO("Be Careful!!! Ready to send primitive cmd [Contact] for flexiv.");

    // primitive command
    std_msgs::String arm_primitive_cmd;

    while (ros::ok())
    {
        arm_primitive_cmd.data = "Contact(contactVel=0.02, coord=world, movingDir=(1,0,0), maxContactForce=5.0)";
        primitive_cmd_publisher.publish(arm_primitive_cmd);
        ros::spinOnce();

        loop_rate.sleep();
    }
    
}


