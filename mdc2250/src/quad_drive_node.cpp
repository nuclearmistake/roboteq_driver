/*
 * mdc2250_node.cpp
 *
 *  Created on: Feb 14, 2013
 *      Author: mccanne
 */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <mdc2250/StampedEncoders.h>
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

#include <string>
#include <cmath>

#include <mdc2250/decode.h>
#include <mdc2250/mdc2250.h>

using namespace mdc2250;

typedef MDC2250* pMDC2250;

pMDC2250 mc[2];
int NUM_VALID_CONTROLLER_PORTS=2;
ros::Publisher odom_pub;
ros::Publisher encoder_pub;
tf::TransformBroadcaster *odom_broadcaster;

static double ENCODER_RESOLUTION = 250*4;
double wheel_circumference = 0.0;
double wheel_base_length = 0.0;
double wheel_diameter = 0.0;
std::string odom_frame_id;

double rot_cov = 0.0;
double pos_cov = 0.0;

static double A_MAX = 20.0;
static double B_MAX = 20.0;

bool _1_is_left_2_is_right;

// Persistent variables
double prev_x = 0, prev_y = 0, prev_w = 0;
ros::Time prev_time;

bool isConnected() { for(int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++) if (mc[i] == NULL || !mc[i]->isConnected()) return false; return true; }


void telemetry_callback1(const std::string &telemetry) {
	//insert meat of tf math into one of these, or dump values in globals and have a thread crunch nubahz
	ROS_DEBUG("Got telemetry1: %s", telemetry.c_str());
}
void telemetry_callback2(const std::string &telemetry) {
	//insert meat of tf math into one of these, or dump values in globals and have a thread crunch nubahz
	ROS_DEBUG("Got telemetry2: %s", telemetry.c_str());
}

void init(MDC2250 *m)
{
	// Setup telemetry
	size_t period = 25;
	try{
		m->setTelemetry("C,V,C,A", period, m == mc[0] ? telemetry_callback1 : telemetry_callback2);
		m->setTelemetry("C,V,C,A", period, m == mc[0] ? telemetry_callback1 : telemetry_callback2);
	}
	catch(std::exception &e)
	{
		ROS_ERROR(e.what());
	}
}

/*  _1_is_left_2_is_right
        <!-- ====================== -->
        <!-- if true -->
        <!--     FRONT
           [m1]         [m1]
            ^            ^
           mc1          mc2
            v            v
           [m2]         [m2]        
        -->
        <!-- ====================== -->
        <!-- if false -->
        <!--     FRONT
           [m1] < mc1 > [m2]



           [m1] < mc2 > [m2]
        -->
        <!-- ====================== -->
*/
void quad_move(double left, double right)
{
    if (NUM_VALID_CONTROLLER_PORTS == 1)
    {
        //mc[0]->
        return;
    }    
}

double wrapToPi(double angle) {
    angle += M_PI;
    bool is_neg = (angle < 0);
    angle = fmod(angle, (2.0*M_PI));
    if (is_neg) {
        angle += (2.0*M_PI);
    }
    angle -= M_PI;
    return angle;
}

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if(!isConnected())
        return;
    // Convert mps to rpm
    double A = msg->linear.x;
    double B = msg->angular.z * (wheel_base_length/2.0);

    double A_rpm = A * (60.0 / (M_PI*wheel_diameter));
    double B_rpm = B * (60.0 / (M_PI*wheel_diameter));

    // Convert rpm to relative
    double A_rel = (A_rpm * 250 * 64) / 58593.75;
    double B_rel = (B_rpm * 250 * 64) / 58593.75;

    // ROS_INFO("Arpm: %f, Arel: %f, Brpm: %f, Brel: %f", A_rpm, A_rel, B_rpm, B_rel);

    // Bounds check
    if(A_rel > A_MAX)
        A_rel = A_MAX;
    if(A_rel < -1*A_MAX)
        A_rel = -1*A_MAX;
    if(B_rel > B_MAX)
        B_rel = B_MAX;
    if(B_rel < -1*B_MAX)
        B_rel = -1*B_MAX;

    // ROS_INFO("%f %f", A_rel, B_rel);

    quad_move(A_rel, B_rel);
}

void errorMsgCallback(const std::exception &ex) {
    ROS_WARN("%s", ex.what());
    for(int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++)
    {
		free(mc[i]);
		mc[i] = NULL;
    }
}

void infoMsgCallback(const std::string &msg) {
    ROS_INFO("%s", msg.c_str());
}

/*void encoderCallback(const ros::TimerEvent& e) {
    // Make sure we are connected
    if(!ros::ok() || !isConnected())
        return;

    AX2550_ENCODER ax2550_encoder(0,0);
    ros::Time now = ros::Time::now();
    // Retreive the data
    try {
        ax2550_encoder = mc->readEncoders();
        // ROS_INFO("Encoder Data: %d, %d", ax2550_encoder.encoder1, ax2550_encoder.encoder2);
    } catch(std::exception &e) {
        ROS_ERROR("Error reading the Encoders: %s", e.what());
        if(!mc->ping()) {
            ROS_ERROR("No response from the motor controller, disconnecting.");
            mc->disconnect();
        }
        return;
    }

    double delta_time = (now - prev_time).toSec();
    prev_time = now;

    // Convert to mps for each wheel from delta encoder ticks
    double left_v = ax2550_encoder.encoder1 * 2*M_PI / ENCODER_RESOLUTION;
    left_v /= delta_time;
    // left_v *= encoder_poll_rate;
    double right_v = ax2550_encoder.encoder2 * 2*M_PI / ENCODER_RESOLUTION;
    right_v /= delta_time;
    // right_v *= encoder_poll_rate;

    StampedEncoders encoder_msg;

    encoder_msg.header.stamp = now;
    encoder_msg.header.frame_id = "base_link";
    encoder_msg.encoders.left_wheel = left_v;
    encoder_msg.encoders.right_wheel = right_v;

    encoder_pub.publish(encoder_msg);

    double v = 0.0;
    double w = 0.0;

    double r_L = wheel_diameter/2.0;
    double r_R = wheel_diameter/2.0;

    v += r_L/2.0 * left_v;
    v += r_R/2.0 * right_v;

    w += r_R/wheel_base_length * right_v;
    w -= r_L/wheel_base_length * left_v;


    // Update the states based on model and input
    prev_x += delta_time * v
                          * cos(prev_w + delta_time * (w/2.0));

    prev_y += delta_time * v
                          * sin(prev_w + delta_time * (w/2.0));
    prev_w += delta_time * w;
    prev_w = wrapToPi(prev_w);

    // ROS_INFO("%f", prev_w);

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(prev_w);

    // Populate the msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_id;
    odom_msg.pose.pose.position.x = prev_x;
    odom_msg.pose.pose.position.y = prev_y;
    odom_msg.pose.pose.orientation = quat;
    odom_msg.pose.covariance[0] = pos_cov;
    odom_msg.pose.covariance[7] = pos_cov;
    odom_msg.pose.covariance[14] = 1e100;
    odom_msg.pose.covariance[21] = 1e100;
    odom_msg.pose.covariance[28] = 1e100;
    odom_msg.pose.covariance[35] = rot_cov;

    // odom_msg.twist.twist.linear.x = v/delta_time;
    odom_msg.twist.twist.linear.x = v;
    // odom_msg.twist.twist.angular.z = w/delta_time;
    odom_msg.twist.twist.angular.z = w;

    odom_pub.publish(odom_msg);

    // TODO: Add TF broadcaster
    // geometry_msgs::TransformStamped odom_trans;
    //     odom_trans.header.stamp = now;
    //     odom_trans.header.frame_id = "odom";
    //     odom_trans.child_frame_id = "base_footprint";
    //
    //     odom_trans.transform.translation.x = prev_x;
    //     odom_trans.transform.translation.y = prev_y;
    //     odom_trans.transform.translation.z = 0.0;
    //     odom_trans.transform.rotation = quat;
    //
    //     odom_broadcaster->sendTransform(odom_trans);
}*/

int main(int argc, char **argv) {
    // Node setup
    ros::init(argc, argv, "quad_node");
    ros::NodeHandle n;
    ros::NodeHandle priv("~");
    prev_time = ros::Time::now();

    // Serial port parameter
    std::string port[2];
    priv.param("serial_port1", port[0], std::string("/dev/motor_controller1"));
    priv.param("serial_port2", port[1], std::string(""));

    //if no 2nd port is specified, assume it's intentional for testing
    if (port[1].size()==0)
    	NUM_VALID_CONTROLLER_PORTS=1;

    // Wheel diameter parameter
    priv.param("wheel_diameter", wheel_diameter, 0.3048);

    wheel_circumference = wheel_diameter * M_PI;

    // Wheel base length
    priv.param("wheel_base_length", wheel_base_length, 0.9144);

    // Odom Frame id parameter
    priv.param("odom_frame_id", odom_frame_id, std::string("odom"));

    // Load up some covariances from parameters
    priv.param("rotation_covariance",rot_cov, 1.0);
    priv.param("position_covariance",pos_cov, 1.0);

    n.param("left_is_1_right_is_2", _1_is_left_2_is_right, true);

    // Odometry Publisher
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 5);

    // Encoder Publisher
    encoder_pub = n.advertise<StampedEncoders>("encoders", 5);

    // TF Broadcaster
    odom_broadcaster = new tf::TransformBroadcaster;

    // cmd_vel Subscriber
    ros::Subscriber sub = n.subscribe("cmd_vel", 1, cmd_velCallback);

    while(ros::ok()) {
    	for (int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++){
			ROS_INFO("MDC2250[%d] connecting to port %s", i, port[i].c_str());
			try {
				mc[i] = new MDC2250();

	            mc[i]->setExceptionHandler(errorMsgCallback);
	            mc[i]->setInfoHandler(infoMsgCallback);

	            //                      10000, true
				mc[i]->connect(port[i], 0, false);

				init(mc[i]);
			} catch(std::exception &e) {
				ROS_ERROR("Failed to connect to the MDC2250: %s", e.what());
			    if (!mc[i]->isConnected())
                {                
                    ROS_WARN("UH OH! NOT CONNECTED!");
				    for(int j=i-1;j>=0;j++)
					    mc[j]->disconnect();
                }
			}
			catch(ConnectionFailedException &e) {
				ROS_ERROR("Failed to connect to the MDC2250: %s", e.what());				
			    if (!mc[i]->isConnected())
                {                
                    ROS_WARN("UH OH! NOT CONNECTED!");
				    for(int j=i-1;j>=0;j++)
					    mc[j]->disconnect();
                }
			}
    	}
        while(isConnected() && ros::ok()) {
            ros::spinOnce();
            ros::Duration(0.001).sleep();
        }
        for(int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++)
        {
            if (mc[i]->isConnected())
                mc[i]->disconnect();
			free(mc[i]);
			mc[i] = NULL;
        }
        if(!ros::ok())        
            break;
        ROS_INFO("Will try to reconnect to the MCD2250 in 5 seconds.");
        ros::Duration(5).sleep();
    }

    ROS_WARN("Broke out of infinite loop");

    return 0;
}
