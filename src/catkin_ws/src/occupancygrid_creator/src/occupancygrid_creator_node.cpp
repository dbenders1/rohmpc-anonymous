
#include <ros/ros.h>

#include <occupancygrid_creator/occupancygrid_creator.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gridmap_creator_node");
    ros::NodeHandle nh("~");

    OccupancygridCreator occupancygrid_creator(nh);

    ros::spin();
    return 0;
}







