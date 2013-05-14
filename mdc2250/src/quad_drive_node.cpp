/*
 * mdc2250_node.cpp
 *
 *  Created on: Feb 14, 2013
 *      Author: mccanne
 */
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <mdc2250/StampedEncoders.h>
#include <mdc2250/MotorRaw.h>
#include <mdc2250/estop.h>
#include <mdc2250/setspeed.h>
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

#include <string>
#include <cmath>

#include <mdc2250/decode.h>
#include <mdc2250/mdc2250.h>

#define DIAF( x ) std::stringstream ss; \
    	ss << "__func__ FAILED - " << x.what(); \
    	ROS_INFO("%s", ss.str().c_str());

#define ENCODER_CPR 360                //  360 clicks / rotation encoders
unsigned int ENCODER_RPM_AT_1000_EFFORT = 120; //  ~2 m/s

using namespace mdc2250;

typedef MDC2250* pMDC2250;

typedef std::map<std::string, std::string> ENCODER;

ENCODER enc[2];
pMDC2250 mc[2];
bool enc_init[2][3];
bool configonly;
bool configured[2];
ros::Time lasttick[2];
bool erroroccurred;
bool spam;
float speed_coefficient = 1.0;

int NUM_VALID_CONTROLLER_PORTS=2;
ros::Publisher odom_pub;
ros::Publisher encoder_pub;
ros::Publisher estoppub;
ros::Publisher voltpub;
tf::TransformBroadcaster *odom_broadcaster;

double wheel_circumference = 0.0;
double robot_width = 0.0;
double wheel_diameter = 0.0;
std::string odom_frame_id;

double rot_cov = 0.0;
double pos_cov = 0.0;

unsigned long int lastC[2];
unsigned long int newc[2][2];

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

bool isConnected() { for(int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++) { if(mc[i] == NULL || !mc[i]->isConnected()) return false; } return true; }

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
    try
    {
		if (_1_is_left_2_is_right)
		{
	#ifdef ROVER_1
			mc[0]->commandMotors(left, left);
			mc[1]->commandMotors(-right, -right);
	#else
			mc[0]->commandMotors(-left, left);
			mc[1]->commandMotors(-right, right);
	#endif
		}
		else
		{
	#ifdef ROVER_1
			mc[0]->commandMotors(left, -right);
			mc[1]->commandMotors(left, -right);
	#else
		mc[0]->commandMotors(left, right);
			mc[1]->commandMotors(-left, -right);
	#endif
		}
    }
    catch(std::exception &ex)
    {
    	DIAF(ex)
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
    double A = msg->linear.x * speed_coefficient - msg->angular.z * (robot_width/2.0);
    double B = msg->linear.x * speed_coefficient + msg->angular.z * (robot_width/2.0);

    double A_rpm = A * (60.0 / (M_PI*wheel_diameter));
    double B_rpm = B * (60.0 / (M_PI*wheel_diameter));

    // Convert RPM to effort
    double A_eff = A_rpm * 1000 / ENCODER_RPM_AT_1000_EFFORT;
    double B_eff = B_rpm * 1000 / ENCODER_RPM_AT_1000_EFFORT;

    // Bounds check
    if(A_eff > A_MAX)
    	A_eff = A_MAX;
    if(A_eff < -1*A_MAX)
    	A_eff = -1*A_MAX;
    if(B_eff > B_MAX)
    	B_eff = B_MAX;
    if(B_eff < -1*B_MAX)
    	B_eff = -1*B_MAX;

    //ROS_INFO("Arpm: %f, Aeff: %f, Brpm: %f, Beff: %f", A_rpm, -A_eff, B_rpm, B_eff);

    quad_move(A_eff, B_eff);
}
void errorMsgCallback(int m, const std::exception &ex) {
    ROS_ERROR("mc[%d] -- %s", m, ex.what());
    erroroccurred = true;
}
void errorMsgCallback1(const std::exception &ex) {
    errorMsgCallback(1, ex);
}
void errorMsgCallback2(const std::exception &ex) {
    errorMsgCallback(2, ex);
}

void infoMsgCallback(int m, const std::string &msg) {
    ROS_INFO("mc[%d] -- %s", m, msg.c_str());
}
void infoMsgCallback1(const std::string &ex) {
	infoMsgCallback(1, ex);
}
void infoMsgCallback2(const std::string &ex) {
	infoMsgCallback(2, ex);
}

long long int total_left;
long long int total_right;
void encode(int left, int right) {
    // Make sure we are connected
    if(!ros::ok() || !isConnected())
        return;

    lastC[0] = lastC[0] + left;
    lastC[1] = lastC[1] + right;
    left = lastC[0];
    right = lastC[1];

    ros::Time now = ros::Time::now();

    double delta_time = (now - prev_time).toSec();
    prev_time = now;

    // Convert to mps for each wheel from delta encoder ticks
    double left_r = left / ENCODER_CPR;
    double right_r = right / ENCODER_CPR;

    double left_d = left_r / wheel_circumference;
	double right_d = right_r / wheel_circumference;

    StampedEncoders encoder_msg;

    encoder_msg.header.stamp = now;
    encoder_msg.header.frame_id = "base_link";
    encoder_msg.encoders.left_wheel = left;
    encoder_msg.encoders.right_wheel = right;

    encoder_pub.publish(encoder_msg);

    double v = (left_d+right_d)/2.0;
    double w = (right_d - left_d) / robot_width; //approximate rotation

    double dv = v / delta_time;
    double dw = w / delta_time;

    double x,y;
    if (v != 0)
    {
		// calculate distance traveled in x and y
		x = cos( w ) * v;
		y = -sin( w ) * v;
		// calculate the final position of the robot
		prev_x += ( cos( prev_w ) * x - sin( prev_w ) * y );
		prev_y += ( sin( prev_w ) * x + cos( prev_w ) * y );
    }
	if( w != 0)
		prev_w += w;

    // ROS_INFO("%f", prev_w);

    geometry_msgs::Quaternion quat;
    quat.x=quat.y=0;
    quat.z=quat.w = w/2;

    // Populate the msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_id;
    odom_msg.child_frame_id = "/base_link";
    odom_msg.pose.pose.position.x = prev_x;
    odom_msg.pose.pose.position.y = prev_y;
    odom_msg.pose.pose.orientation = quat;
    odom_msg.pose.covariance[0] = pos_cov;
    odom_msg.pose.covariance[7] = pos_cov;
    odom_msg.pose.covariance[14] = 1e100;
    odom_msg.pose.covariance[21] = 1e100;
    odom_msg.pose.covariance[28] = 1e100;
    odom_msg.pose.covariance[35] = rot_cov;
    odom_msg.twist.twist.linear.x = dv;
    odom_msg.twist.twist.angular.z = dw;
    odom_msg.twist.covariance[0] = pos_cov;
    odom_msg.twist.covariance[7] = pos_cov;
    odom_msg.twist.covariance[14] = 1e100;
    odom_msg.twist.covariance[21] = 1e100;
    odom_msg.twist.covariance[28] = 1e100;
    odom_msg.twist.covariance[35] = rot_cov;

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
	lasttick[i]=ros::Time::now();
	//insert meat of tf math into one of these, or dump values in globals and have a thread crunch nubahz
	std::vector<std::string> TandVal = split(telemetry, '=');
	if (TandVal.size() != 2)
	{
		ROS_WARN("Invalid telemetry format");
		return;
	}
	enc[i][TandVal[0]] = TandVal[1];
	char T = TandVal[0][0];
	std::vector<std::string> vals = split(TandVal[1], ':');
	switch(T)
	{
		case 'C': enc_init[i][0] = true; break;
		case 'V': enc_init[i][1] = true; break;
		case 'A': enc_init[i][2] = true; break;
	}
	if (NUM_VALID_CONTROLLER_PORTS==1 && enc_init[0][0] && enc_init[0][1] && enc_init[0][2])
	{
		if (spam)
		{
			std::stringstream ss;
			for(ENCODER::iterator i=enc[0].begin();i!=enc[0].end();i++)
			{
				ss << i->first << "=" << i->second << " ";
			}
			printf("L=%f, R=%f, mc[0]{ %s } \r", _left, _right,	ss.str().c_str());
			fflush(stdout);
		}
		if (TandVal[0][0]=='C')
		{
			std::vector<std::string> vc = split(enc[0]["C"], ':');
			newc[i][0] = atoi(vc[0].c_str());
			newc[i][1] = atoi(vc[1].c_str());
			encode(newc[0][0], newc[0][1]);
		}
	}
	else if (enc_init[0][0] && enc_init[0][1] && enc_init[0][2] && enc_init[1][0] && enc_init[1][1] && enc_init[1][2])
	{
		if (spam)
		{
			std::stringstream ss0;
			for(ENCODER::iterator it=enc[0].begin();it!=enc[0].end();it++)
			{
				ss0 << it->first << "=" << it->second << " ";
			}
			std::stringstream ss1;
			for(ENCODER::iterator it=enc[1].begin();it!=enc[1].end();it++)
			{
				ss1 << it->first << "=" << it->second << " ";
			}
			printf("L=%f, R=%f, mc[0]{ %s } mc[1]{ %s }\r", _left, _right,	ss0.str().c_str(), ss1.str().c_str());
			fflush(stdout);
		}
		if (TandVal[0][0]=='C')
		{
			std::vector<std::string> vc = split(enc[i]["C"], ':');
			newc[i][0] = atoi(vc[0].c_str());
			newc[i][1] = atoi(vc[1].c_str());
			if (i==0)
			{
				if (enc[i].count("V")>0)
				{
					static float vf;
					std::vector<std::string> vv = split(enc[i]["V"], ':');
					if (vv.size() > 0)
					{
						vf = (float)(atoi(vv[0].c_str()))/10.0;
						static std_msgs::Float32 v;
						v.data = vf;
						voltpub.publish(v);
					}
				}
				encode((newc[0][0]+newc[1][0])/2.0, (newc[0][1]+newc[1][1])/2.0);
			}
		}
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
	lasttick[m==mc[0]?0:1] = ros::Time::now();
	// Setup telemetry
	size_t period = 25;
	//try{
		for(int i=0;i<3;i++)
			enc_init[(m==mc[0]?0:1)][i]=false;
		enc[m==mc[0]?0:1].clear();
		m->setTelemetry("C,V,A,T,S,E,F", period, m == mc[0] ? telemetry_callback1 : telemetry_callback2);
		lasttick[m==mc[0]?0:1] = ros::Time::now();
	/*}
	catch(std::exception &e) { 	ROS_ERROR("%s", e.what()); 	}*/
}

void raw_callback(int i, const mdc2250::MotorRaw::ConstPtr& msg) {
	_left = msg->left;
	_right = msg->right;
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
	static std_msgs::Bool b;
	b.data=status;
	estoppub.publish(b);
}

void estopsub_callback(const std_msgs::Bool::ConstPtr& msg)
{
	SetEStop(msg->data);
}

bool estop_callback(mdc2250::estop::Request  &req,
					mdc2250::estop::Response &res)
{
	SetEStop(req.state);
	return true;
}


bool maxspeed_callback(mdc2250::setspeed::Request  &req,
					mdc2250::setspeed::Response &res)
{
	float mps = req.maxspeed;
	float rpm = 60.0 * mps / wheel_circumference;
	ROS_INFO("Setting max speed to %f m/s -- %f rpm",mps,rpm);
	ENCODER_RPM_AT_1000_EFFORT = rpm;
	for(int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++)
		for(int j=1;j<=2;j++)
			mc[i]->setMaxRPMValue(j, ENCODER_RPM_AT_1000_EFFORT);
	return true;
}

bool speedcoefficient(mdc2250::setspeed::Request  &req,
					mdc2250::setspeed::Response &res)
{
	ROS_WARN("DANGER WILL ROBINSON! Setting this to 3 turns a 1 m/s command into a 3 m/s command. use care with this!");
	speed_coefficient = req.maxspeed;
	ROS_INFO("Set coefficient to %f",speed_coefficient);
	return true;
}

int main(int argc, char **argv) {
    // Node setup
    ros::init(argc, argv, "quad_node");
    ros::NodeHandle n;
    ros::NodeHandle priv("~");
    prev_time = ros::Time::now();

#ifdef ROVER_1
    ROS_ERROR("WHY DOES THIS ROBOT EXIST, AND WHY ARE YOU TESTING CODE ON IT?!");
#endif

    // Serial port parameter
    std::string port[2];
    priv.param("serial_port1", port[0], std::string("/dev/motor_controller1"));
    priv.param("serial_port2", port[1], std::string(""));

    //priv.param("config_only", configonly, false);

    //if no 2nd port is specified, assume it's intentional for testing
    if (port[1].size()==0)
    	NUM_VALID_CONTROLLER_PORTS=1;

    // Wheel diameter parameter
    priv.param("wheel_diameter", wheel_diameter, 0.3048);

    wheel_circumference = wheel_diameter * M_PI;

    // Wheel base length
    priv.param("robot_width", robot_width, 0.9144);

    // Odom Frame id parameter
    priv.param("odom_frame_id", odom_frame_id, std::string("odom"));

    // Load up some covariances from parameters
    priv.param("rotation_covariance",rot_cov, 1.0);
    priv.param("position_covariance",pos_cov, 1.0);

    priv.param("spam",spam, true);

    // Odometry Publisher
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 5);

	// Encoder Publisher
	encoder_pub = n.advertise<StampedEncoders>("encoders", 5);

	// TF Broadcaster
	odom_broadcaster = new tf::TransformBroadcaster;

	estoppub = n.advertise<std_msgs::Bool>("estopState", 1, true);

	voltpub = n.advertise<std_msgs::Float32>("Voltage", 1);

	// cmd_vel Subscriber
	ros::Subscriber sub = n.subscribe("cmd_vel", 1, cmd_velCallback);

	// raw motor sub for mc[0]
	ros::Subscriber rawsub1 = n.subscribe("mc1/raw", 1, raw_callback1);
	// raw motor sub for mc[1]
	ros::Subscriber rawsub2 = n.subscribe("mc2/raw", 1, raw_callback2);

	ros::ServiceServer estopper = n.advertiseService("estop", estop_callback);

	ros::ServiceServer sped = n.advertiseService("maxspeed", maxspeed_callback);

	ros::ServiceServer spedd = n.advertiseService("speedcoefficient", speedcoefficient);

	ros::Subscriber estopsub = n.subscribe("setEstop", 1, estopsub_callback);

    while(ros::ok()) {
		erroroccurred = false;
    	for (int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++){
    		if (configonly && configured[i]) continue;
			ROS_INFO("MDC2250[%d] connecting to port %s", i, port[i].c_str());
			ROS_INFO("config_only = %s", configonly ? "true" : "false");
			try {
				mc[i] = new MDC2250();

	            mc[i]->setExceptionHandler(i==0 ? errorMsgCallback1 : errorMsgCallback2);
	            mc[i]->setInfoHandler(i==0 ? infoMsgCallback1 : infoMsgCallback2);

	            //                      10000, true
				mc[i]->connect(port[i], 500, false);
	            if (configonly)
					for(int j=1;j<=2;j++)
					{
						mc[i]->setEncoderPulsesPerRotation(j, ENCODER_CPR*4);
						mc[i]->setEncoderUsage(j, mdc2250::constants::feedback, j==1, j==2);
						mc[i]->setMaxRPMValue(j, ENCODER_RPM_AT_1000_EFFORT);
						mc[i]->setOperatingMode(j, mdc2250::constants::closedloop_speed);
					}
				if (configonly && !erroroccurred)
				{
					mc[i]->commitConfig();
					configured[i] = true;
				}
				if (!configonly)
				{
					init(mc[i]);
				}
			} catch(ConnectionFailedException &e) {
				ROS_ERROR("Failed to connect to the MDC2250[%d]: %s", i, e.what());
				erroroccurred = true;
			} catch(std::exception &e) {
				ROS_ERROR("SOME exception occured while connecting to the MDC2250[%d]: %s", i, e.what());
				erroroccurred = true;
			}
    	}
    	if (!erroroccurred && isConnected())
    	{
			bool estopped = false;
			for(int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++)
				estopped |= mc[i]->isEstopped();
			std_msgs::Bool b;
			b.data = estopped;
			estoppub.publish(b);
			for(int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++)
				lasttick[i]=ros::Time::now();
			erroroccurred = false;
    	}
    	bool conly = configonly;
    	bool err = erroroccurred;
    	bool conn = isConnected();
    	bool ok = ros::ok();
        while(!conly && !err && conn && ok) {
            ros::spinOnce();
            ros::Duration(0.001).sleep();
            static ros::Time n = ros::Time::now();
            for(int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++)
            	{
            		static ros::Duration d = n - lasttick[i];
            		if (d.toSec() > 0.5)
            		{
            			ROS_WARN("TIMED OUT!");
            			break;
            		}
            	}
        	conly = configonly;
        	err = erroroccurred;
        	conn = isConnected();
        	ok = ros::ok();
        }
        ROS_WARN("FAILURE REASON: configonly=%s errored=%s !connected=%s !ok=%s",configonly?"true":"false",erroroccurred?"true":"false",!conn?"true":"false",!ok?"true":"false");
        for(int i=0;i<NUM_VALID_CONTROLLER_PORTS;i++)
        {
            if (mc[i] != NULL && mc[i]->isConnected())
            {
				mc[i]->disconnect();
                mc[i] = NULL;
            }
        }
        if((configonly && !erroroccurred) || !ros::ok())
        {
            break;
        }
        ROS_INFO("Will try to reconnect to the MCD2250 in 5 seconds.");
        ros::Duration(5).sleep();
    }

    ROS_WARN("Broke out of infinite loop");

    return 0;
}
