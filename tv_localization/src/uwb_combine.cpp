#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <cmath>

class combine{
    public:
        combine(){
            sub_position = n.subscribe("uwb_odom_positionOnly", 1000, &combine::clbk_odom_Position, this);
            sub_orientation = n.subscribe("uwb_odom_orientationOnly", 1000, &combine::clbk_odom_Orientation, this);
            odom_pub = n.advertise<nav_msgs::Odometry>("uwb_odom", 1000);
            std::cout<<"Class Initializer\n";
        }

        void send_tf(){
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            tf::Quaternion q(quat_x_,quat_y_,quat_z_,quat_w_);

            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            q.setRPY(0, 0, yaw);

            float l1 = 1.2679;
            // l1 = 1.2679;
            transform.setOrigin( tf::Vector3(position_x_ + l1 * cos(yaw), position_y_ + l1 * sin(yaw), 1.0) );
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
        }

        void clbk_odom_Position(const nav_msgs::Odometry::ConstPtr& msg)
        {   
            ROS_INFO("Position Callback");
            ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
            position_x_ = msg->pose.pose.position.x;
            position_y_ = msg->pose.pose.position.y;
            
            odom_combined.header = msg->header;
            odom_combined.pose.pose.position.x = position_x_ + tf_pos_x_;
            odom_combined.pose.pose.position.y = position_y_ + tf_pos_y_;
            odom_combined.pose.pose.position.z = 1;
            ready_clbk_odom = 1;
        }

        void clbk_odom_Orientation(const nav_msgs::Odometry::ConstPtr& msg)
        {   
            quat_x_ = msg->pose.pose.orientation.x;
            quat_y_ = msg->pose.pose.orientation.y;
            quat_z_ = msg->pose.pose.orientation.z;
            quat_w_ = msg->pose.pose.orientation.w;
            tf_pos_x_ = msg->pose.pose.position.x;
            tf_pos_y_ = msg->pose.pose.position.y;
            ROS_INFO("Orientation Callback");
            ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", quat_x_, quat_y_, quat_z_, quat_w_);
            odom_combined.pose.pose.orientation.x = quat_x_;
            odom_combined.pose.pose.orientation.y = quat_y_;
            odom_combined.pose.pose.orientation.z = quat_z_;
            odom_combined.pose.pose.orientation.w = quat_w_;
        }

        void loop(){
            ros::Time current_time;
            // ros::Time begin = ros::Time::now();
            current_time = ros::Time::now();
            ros::Rate r(50); // 10 hz
            while(ros :: ok()){

                if (ready_clbk_odom == 1){
                    ready_clbk_odom = 0;
                    odom_pub.publish(odom_combined);
                    send_tf();
                }
                ros::spinOnce();               
                r.sleep();
            }
        }



    private:
        ros::NodeHandle n;
        ros::Subscriber sub_orientation;
        ros::Subscriber sub_position;
        ros::Publisher odom_pub;

        double position_x_ = 0, position_y_ = 0;
        double tf_pos_x_ = 0, tf_pos_y_ = 0;
        double quat_x_ = 0, quat_y_ = 0, quat_z_ = 0, quat_w_ = 0;
        bool ready_clbk_odom = 0;
        nav_msgs::Odometry odom_combined;

        
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_combine_raw_and_publish");
    ROS_INFO("Node Initialized");
    combine uwb;
    uwb.loop();
    ros::spin();
    return 0;
}