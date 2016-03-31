#include <ros/ros.h>
#include <cargate_udp.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cargate");
    ros::NodeHandle nh;
    aa::modules::io::cargate::CarGate node(nh,argc,argv);

    while(ros::ok())
    {
        node.updateHook();
        ros::spinOnce();
    }
     node.stopHook();
    return 0;
}
