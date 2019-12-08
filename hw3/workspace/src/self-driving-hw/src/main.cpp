#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

#include <cmath>

#define MOVE_TURTLE2_SPEED   1
#define ROTATE_TURTLE2_SPEED 4
#define EPS 1e-2

const double PI = 3.14159265;

turtlesim::Pose pose1;
turtlesim::Pose pose2;

ros::Publisher pub2;

void move_turtle2() {
    geometry_msgs::Twist move;

    float x = pose1.x - pose2.x;
    float y = pose1.y - pose2.y;

    if (x*x + y*y < EPS) {
        ROS_INFO("Target reached!\n");
        pub2.publish(move);
        return;
    }

    move.linear.x = MOVE_TURTLE2_SPEED;

    float need_angle = std::atan2(y, x);
    float have_angle = pose2.theta;
    while (std::abs(have_angle) > PI + EPS) {
        if (have_angle > 0)
            have_angle -= 2 * PI;
        else
            have_angle += 2 * PI;
    }

    float rotate_angle = (need_angle - have_angle) * ROTATE_TURTLE2_SPEED;
    if (std::abs(rotate_angle) < EPS) {
        rotate_angle = 0;
    }

    move.angular.z = rotate_angle;

    ROS_INFO("Moving turtle2\n"
             "x = %f, theta = %f",
              x, rotate_angle);
    pub2.publish(move);
}

void turtle1_pose_callback(const turtlesim::Pose& pos) {
    ROS_INFO("Turtle1 position:\n"
        "x = %f\ny = %f\ntheta=%f",
        pos.x, pos.y, pos.theta
    );

    pose1 = pos;
    move_turtle2();
}

void turtle2_pose_callback(const turtlesim::Pose& pos) {
    ROS_INFO("Turtle2 position:\n"
        "x = %f\ny = %f\ntheta=%f",
        pos.x, pos.y, pos.theta
    );
    pose2 = pos;
}

void spawn_turtle2(ros::NodeHandle& node) {
     ros::ServiceClient client = node.serviceClient<turtlesim::Spawn>("spawn");

     turtlesim::Spawn srv;
     srv.request.x = 2;
     srv.request.y = 7;
     srv.request.theta = 0.2;
     srv.request.name = "turtle2";

     if (client.call(srv)) {
        ROS_ERROR("Failed to create turtle2");
     } else {
        ROS_INFO("turtle2 was spawned");
     }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "turtle_follower");

    ros::NodeHandle node;

    spawn_turtle2(node);

    ros::Subscriber sub1 = node.subscribe("/turtle1/pose", 1, turtle1_pose_callback);
    ros::Subscriber sub2 = node.subscribe("/turtle2/pose", 1, turtle2_pose_callback);
    pub2 = node.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
