#ifndef OCCUPANCYGRID_CREATOR_OCCUPANCYGRID_CREATOR_H
#define OCCUPANCYGRID_CREATOR_OCCUPANCYGRID_CREATOR_H

#include <math.h>
#include <string>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/TransformStamped.h>

#include <occupancygrid_creator/ros_visuals.h>

class OccupancygridCreator
{
public:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    int n_runs_ = -1, run_idx_ = 0;

    // ros messages
    nav_msgs::OccupancyGrid gridmap_;
    std::vector<geometry_msgs::TransformStamped> state_msgs_stored_;
    std::vector<geometry_msgs::TransformStamped> state_msgs_lines_stored_;

    // Map ID
    std::string map_id_;

    // Image
    cv::Mat occupancy_image_;

    // Static obstacles
    bool static_obstacle_use;
    std::vector<double> static_circles_x_;
    std::vector<double> static_circles_y_;
    std::vector<double> static_circles_radius_;
    std::vector<double> static_squares_x_;
    std::vector<double> static_squares_y_;
    std::vector<double> static_squares_length_;
    std::vector<double> static_squares_width_;
    std::vector<double> static_squares_orientation_;
    std::vector<double> static_squares_line_thickness_;
    bool inflate_;
    double inflation_thickness_;

    // Dynamic obstacles
    std::vector<double> receiving_obstacle_squares_length_;
    std::vector<double> receiving_obstacle_squares_width_;
    std::vector<double> receiving_obstacle_squares_line_thickness_;
    std::vector<double> receiving_obstacle_squares_orientation_;
    std::vector<double> receiving_obstacle_lines_length_;
    std::vector<double> receiving_obstacle_lines_line_thickness_;

    // ros publisher
    ros::Publisher pub_map_;

    // ros subscribers
    std::vector<ros::Subscriber> subs_vector_;
    std::vector<ros::Subscriber> subs_vector_lines_;
    ros::Subscriber sub_gazebo_;


    // stored parameters
    //// received obstacle
    bool receiving_obstacle_position_use_;
    std::vector<double> receiving_obstacle_radius_;
    std::vector<bool>state_received_;
    std::vector<bool>state_received_lines_;
    //// received gazebo obstacle
    bool receiving_obstacle_position_gazebo_use_;
    double receiving_obstacle_gazebo_radius_;
    bool state_gazebo_received_;
    gazebo_msgs::ModelStates state_msgs_gazebo_stored_;

    // Recording
    std::string recording_topic_base_ = "/grid/obs/rec/";
    std::vector<ros::Publisher> obs_rec_circle_pubs_, obs_rec_square_pubs_;
    std::vector<double> static_circle_indices_to_record_, static_square_indices_to_record_;
    std::vector<std::string> static_circle_names_, static_square_names_;

    // Visualization
    std::unique_ptr<ROSMarkerPublisher> static_obstacles_marker_pub_;
    std::vector<double> static_circle_indices_to_visualize_, static_square_indices_to_visualize_;
    std::vector<double> marker_color_;
    double marker_z_;


    OccupancygridCreator(ros::NodeHandle &node_handle);

    bool loadConfig();

    bool errorFunc(const std::string name);

    void defaultFunc(const std::string name);

    void createMap(const ros::TimerEvent& event);

    void createStaticObstacles(std::vector<double> x, std::vector<double> y, std::vector<double> radius);

    void placeObstacleInGrid(nav_msgs::OccupancyGrid &gridmap, double x_cur, double y_cur, double radius_cur);

    void placeSquareInImage(nav_msgs::OccupancyGrid &gridmap, cv::Mat &occupancy_image, double x_cur, double y_cur, double length, double width, double orientation, double line_thickness);

    void placeInflatedSquareInImage(nav_msgs::OccupancyGrid &gridmap, cv::Mat &occupancy_image, double x_cur, double y_cur, double length, double width, double orientation, double inflation_thickness);

    void drawRoundedRect(cv::Mat &image, cv::Point center, int width, int height, int cornerRadius, double angle, cv::Scalar color, int thickness);

    void placeLineInImage(nav_msgs::OccupancyGrid &gridmap, cv::Mat &occupancy_image, double x_cur, double y_cur, double length, double orientation, double line_thickness);

    void callbackPositionObstacleSquares(const geometry_msgs::TransformStamped::ConstPtr &msg, const long unsigned int i);
    void callbackPositionObstacleLines(const geometry_msgs::TransformStamped::ConstPtr &msg, const long unsigned int i);

    void callbackPositionObstacleGazebo(const gazebo_msgs::ModelStates &msg);

    void publishStaticObstacles();
    void createStaticObstacleVisualizations();
    void publishStaticObstacleVisualizations();
};

#endif
