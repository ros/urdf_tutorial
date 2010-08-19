#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <urdf/joint.h>
#include <map>

int main(int argc, char** argv) {
    ros::init(argc, argv, "easy_state_publisher");
    ros::NodeHandle n;
    ros::Publisher  m_joint_pub = n.advertise
                                  <sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(2);

    urdf::Model my_model;
    if (!my_model.initParam("/robot_description")) {
        ROS_ERROR("Failed to parse urdf robot model");
        return false;
    }

    typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joint_map;
    typedef std::map<std::string, double> position_map;
    joint_map joints = my_model.joints_;
    position_map positions;

    int count = 0;
    for (joint_map::const_iterator it =  joints.begin(); it !=  joints.end(); ++it) {
        std::string name = it->first;
        urdf::Joint* joint = it->second.get();
        if (joint->type!=urdf::Joint::FIXED) {
            positions[name] = 0;
            count++;
            ROS_INFO("%s", name.c_str());
        }
    }

    sensor_msgs::JointState joint_state;
    joint_state.name.resize(count);
    joint_state.position.resize(count);
    joint_state.velocity.resize(count);
    int i = 0;
    for (position_map::const_iterator it =  positions.begin(); it !=  positions.end(); ++it) {
        joint_state.name[i++] = it->first.c_str();
    }

    while (ros::ok()) {

        joint_state.header.stamp = ros::Time::now();
        m_joint_pub.publish(joint_state);


        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
