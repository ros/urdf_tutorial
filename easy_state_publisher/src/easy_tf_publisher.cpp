#include <string>
#include <ros/ros.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <urdf/model.h>
#include <urdf/joint.h>
#include <kdl_parser/kdl_parser.hpp>
#include <map>

int main(int argc, char** argv) {
    ros::init(argc, argv, "easy_state_publisher");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);

    urdf::Model my_model;
    if (!my_model.initParam("/robot_description")) {
        ROS_ERROR("Failed to parse urdf robot model");
        return false;
    }
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)) {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    robot_state_publisher::RobotStatePublisher* publisher = new robot_state_publisher::RobotStatePublisher(my_tree);

    typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joint_map;

    std::map<std::string, double> positions;
    joint_map joints = my_model.joints_;

    for (joint_map::const_iterator it =  joints.begin(); it !=  joints.end(); ++it) {
        std::string name = it->first;
        urdf::Joint* joint = it->second.get();
        if (joint->type==urdf::Joint::REVOLUTE || joint->type==urdf::Joint::CONTINUOUS ||
                joint->type==urdf::Joint::PRISMATIC)
            positions[name] = 0;
    }
    while (ros::ok()) {
        publisher->publishTransforms(positions, ros::Time::now());

        // Process a round of subscription messages
        ros::spinOnce();

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
