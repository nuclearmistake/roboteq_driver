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
#include <mdc2250/MotorRaw.h>
#include <mdc2250/estop.h>
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

#include <string>
#include <cmath>

#include <mdc2250/decode.h>
#include <mdc2250/mdc2250.h>

using namespace mdc2250;

typedef MDC2250* pMDC2250;

struct encoder
{
	unsigned long int lastC[2]; //previous tick count
	unsigned long int C[2]; //encoder_count_absolute
	unsigned long int V[3]; //motor volts
	unsigned long int A[2]; //motor amps
};

typedef struct encoder ENCODER;

ENCODER enc[2];
pMDC2250 mc[2];
bool enc_init[2][3];

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

static double A_MAX = 1000.0;
static double B_MAX = 1000.0;

bool _1_is_left_2_is_right;

// Persistent variables
double prev_x = 0, prev_y = 0, prev_w = 0;
ros::Time prev_time;

// http://stackoverflow.com/questions/236129/splitting-a-string-in-c
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    return split(s, delim, elems);
}

bool isConnected() { for(int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++) if (mc[i] == NULL || !mc[i]->isConnected()) return false; return true; }

double _left, _right;

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
	_left = left;
	_right = right;
	//ROS_INFO("(L=%f, R=%f)",left,right);
    if (NUM_VALID_CONTROLLER_PORTS == 1)
    {
        mc[0]->commandMotors(left, right);
        return;
    }    
    if (_1_is_left_2_is_right)
    {
    	mc[0]->commandMotors(left, left);
    	mc[1]->commandMotors(right, right);
    }
    else
    {
    	mc[0]->commandMotors(left, right);
    	mc[1]->commandMotors(left, right);
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
    double A_rel = (A_rpm * 250 * 64) / 1171.875; //58593.75;
    double B_rel = (B_rpm * 250 * 64) / 1171.875; //58593.75;

    //ROS_INFO("Arpm: %f, Arel: %f, Brpm: %f, Brel: %f", A_rpm, A_rel, B_rpm, B_rel);

    // Bounds check
    if(A_rel > A_MAX)
        A_rel = A_MAX;
    if(A_rel < -1*A_MAX)
        A_rel = -1*A_MAX;
    if(B_rel > B_MAX)
        B_rel = B_MAX;
    if(B_rel < -1*B_MAX)
        B_rel = -1*B_MAX;

    //ROS_INFO("%f %f", A_rel, B_rel);

    //A_rel is RELATIVE POSITION!, B_rel is direction, so let's get per-wheel speeds at that angle... need a motion model or encoders to use this bit
    double l, r;
    //A_rel /= cos(B_rel);
    l = A_rel * sin(M_PI/4 - B_rel) + B_rel;
    r = A_rel * cos(M_PI/4 - B_rel) + B_rel;

    //fix domain, but BREAK SANITY!
    //l *= 82.653241797;
    //r *= 82.653241797;

    quad_move(l, r);
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

void encode(int left, int right) {
    // Make sure we are connected
    if(!ros::ok() || !isConnected())
        return;

    ros::Time now = ros::Time::now();

    double delta_time = (now - prev_time).toSec();
    prev_time = now;

    // Convert to mps for each wheel from delta encoder ticks
    double left_v = left * 2*M_PI / ENCODER_RESOLUTION;
    left_v /= delta_time;
    // left_v *= encoder_poll_rate;
    double right_v = right * 2*M_PI / ENCODER_RESOLUTION;
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
    geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = now;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = prev_x;
	odom_trans.transform.translation.y = prev_y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = quat;

	odom_broadcaster->sendTransform(odom_trans);
}

void telemetry_callback(int i, const std::string &telemetry) {
	//insert meat of tf math into one of these, or dump values in globals and have a thread crunch nubahz
	std::vector<std::string> TandVal = split(telemetry, '=');
	if (TandVal.size() != 2)
	{
		ROS_WARN("Invalid telemetry format");
		return;
	}
	char T = TandVal[0][0];
	std::vector<std::string> vals = split(TandVal[1], ':');
	int s = vals.size();
	switch(T)
	{
		case 'C': for(int j=0;j<s;j++) { enc[i].lastC[j] = enc[i].C[j]; enc[i].C[j] = atoi(vals[j].c_str()); } enc_init[i][0] = true; break;
		case 'V': for(int j=0;j<s;j++) enc[i].V[j] = atoi(vals[j].c_str()); enc_init[i][1] = true; break;
		case 'A': for(int j=0;j<s;j++) enc[i].A[j] = atoi(vals[j].c_str()); enc_init[i][2] = true; break;
	}
	printf("L=%f, R=%f, mc[0]{ C=%d:%d, V=%d:%d:%d, A=%d:%d }  mc[1]{ C=%d:%d, V=%d:%d:%d, A=%d:%d } \r", _left, _right,
			(int)(enc[0].lastC[0] - enc[0].C[0]), (int)(enc[0].lastC[1] - enc[0].C[1]),
			(int)enc[0].V[0], (int)enc[0].V[1], (int)enc[0].V[2],
			(int)enc[0].A[0], (int)enc[0].A[1],
			(int)(enc[1].lastC[0] - enc[1].C[0]), (int)(enc[1].lastC[1] - enc[1].C[1]),
			(int)enc[1].V[0], (int)enc[1].V[1], (int)enc[1].V[2],
			(int)enc[1].A[0], (int)enc[1].A[1]);
	fflush(stdout);
	if (NUM_VALID_CONTROLLER_PORTS==1 && enc_init[0][0] && enc_init[0][1] && enc_init[0][2])
	{
		encode(enc[0].lastC[0] - enc[0].C[0], enc[0].lastC[1] - enc[0].C[1]);
	}
	else if (enc_init[0][0] && enc_init[0][1] && enc_init[0][2] && enc_init[1][0] && enc_init[1][1] && enc_init[1][2])
	{
		if (_1_is_left_2_is_right)
			encode((enc[0].lastC[0] - enc[0].C[0]+enc[0].lastC[1] - enc[0].C[1])/2, (enc[1].lastC[0] - enc[1].C[0]+enc[1].lastC[1] - enc[1].C[1])/2);
		else
			encode((enc[0].lastC[0] - enc[0].C[0]+enc[1].lastC[0] - enc[1].C[0])/2, (enc[0].lastC[1] - enc[0].C[1]+enc[1].lastC[1] - enc[1].C[1])/2);
	}
}

void telemetry_callback1(const std::string &telemetry) {
	telemetry_callback(0, telemetry);
}
void telemetry_callback2(const std::string &telemetry) {
	telemetry_callback(1, telemetry);
}

void init(MDC2250 *m)
{
	// Setup telemetry
	size_t period = 25;
	try{
		for(int i=0;i<3;i++)
			enc_init[(m==mc[0]?0:1)][i]=false;
		enc[m==mc[0]?0:1].C[0]=0;
		enc[m==mc[0]?0:1].C[1]=0;
		enc[m==mc[0]?0:1].V[0]=0;
		enc[m==mc[0]?0:1].V[1]=0;
		enc[m==mc[0]?0:1].V[2]=0;
		enc[m==mc[0]?0:1].A[0]=0;
		enc[m==mc[0]?0:1].A[1]=0;
		m->setTelemetry("C,V,C,A", period, m == mc[0] ? telemetry_callback1 : telemetry_callback2);
		m->setTelemetry("C,V,C,A", period, m == mc[0] ? telemetry_callback1 : telemetry_callback2);
	}
	catch(std::exception &e)
	{
		ROS_ERROR("%s", e.what());
	}
}

void raw_callback(int i, const mdc2250::MotorRaw::ConstPtr& msg) {
	mc[i]->commandMotors(msg->left, msg->right);
}
void raw_callback1(const mdc2250::MotorRaw::ConstPtr& msg) {
	raw_callback(0, msg);
}
void raw_callback2(const mdc2250::MotorRaw::ConstPtr& msg) {
	raw_callback(1,msg);
}

void SetEStop(bool status)
{
	for (int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++){
		if (status)
			mc[i]->estop();
		else {
			mc[i]->clearEstop();
			init(mc[i]);
		}
	}
}

bool estop_callback(mdc2250::estop::Request  &req,
					mdc2250::estop::Response &res)
{
	SetEStop(req.state);
	return true;
}

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

    // raw motor sub for mc[0]
    ros::Subscriber rawsub1 = n.subscribe("mc1/raw", 1, raw_callback1);
    // raw motor sub for mc[1]
    ros::Subscriber rawsub2 = n.subscribe("mc2/raw", 1, raw_callback2);

    ros::ServiceServer estopper = n.advertiseService("estop", estop_callback);

    while(ros::ok()) {
    	for (int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++){
			ROS_INFO("MDC2250[%d] connecting to port %s", i, port[i].c_str());
			try {
				mc[i] = new MDC2250();

	            mc[i]->setExceptionHandler(errorMsgCallback);
	            mc[i]->setInfoHandler(infoMsgCallback);

	            //                      10000, true
				mc[i]->connect(port[i], 10000, true);

				init(mc[i]);
			} catch(ConnectionFailedException &e) {
				ROS_ERROR("Failed to connect to the MDC2250: %s", e.what());
			    if (!mc[i]->isConnected())
                {                
                    ROS_WARN("UH OH! NOT CONNECTED!");
				    for(int j=i-1;j>=0;j++)
					    mc[j]->disconnect();
                }
			} catch(std::exception &e) {
				ROS_ERROR("SOME exception occured while connecting to the MDC2250: %s", e.what());
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
