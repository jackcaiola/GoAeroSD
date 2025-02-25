//this will process what the leg commands SHOULD be given the plane. It will send those commands to the master node

#include <ros/ros.h>
#include <iomanip>
#include <sensor_msgs/Range.h>
#include <iostream>
#include <cmath>
#include <std_msgs/Float32.h>

bool received_leg1 = false;
bool received_leg2 = false;
bool received_leg3 = false;
bool received_leg4 = false;

double angle_leg1 = 0.0;
double angle_leg2 = 0.0;
double angle_leg3 = 0.0;
double angle_leg4 = 0.0;

uint8_t pmw_leg1 = 0;
uint8_t pmw_leg2 = 0;
uint8_t pmw_leg3 = 0;
uint8_t pmw_leg4 = 0;

int main(int argc, char **argv) {
    ros::init(argc, argv, "comProc");
    ros::NodeHandle n;

	//subscribers for numerical angles
    ros::Subscriber sub_leg1 = n.subscribe("leg1/numAngle", 1000, rangeCallbackLeg1);
    ros::Subscriber sub_leg2 = n.subscribe("leg1/numAngle", 1000, rangeCallbackLeg2);
    ros::Subscriber sub_leg3 = n.subscribe("leg1/numAngle", 1000, rangeCallbackLeg3);
    ros::Subscriber sub_leg4 = n.subscribe("leg1/numAngle", 1000, rangeCallbackLeg4);
    
    	//publishers for PWM commands
    pwm_pub_leg1 = n.advertise<std_msgs::Float32>("leg1/pwm", 10);
    pwm_pub_leg2 = n.advertise<std_msgs::Float32>("leg2/pwm", 10);
    pwm_pub_leg3 = n.advertise<std_msgs::Float32>("leg3/pwm", 10);
    pwm_pub_leg4 = n.advertise<std_msgs::Float32>("leg4/pwm", 10);
    
   ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce(); 
   
   //I need to check to see if I've received for each individual leg, map the new signal to a PWM value, and publish that PWM value to legx/pwm
   
   	if(received_leg1==true){
   	//map legx/numAngle to pwm
   	//pwm_legx = outputMin + ((input - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin);
   	pwm_leg1 = outputMin + ((angle_leg1 - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin);
   	bool received_leg1 = false;
   	}
   
        if(received_leg2==true){
   	//map legx/numAngle to pwm
   	pwm_leg2 = outputMin + ((angle_leg2 - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin);
   	bool received_leg2 = false;
   	}
   	
   	if(received_leg3==true){
   	//map legx/numAngle to pwm
   	pwm_leg3 = outputMin + ((angle_leg3 - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin);
   	bool received_leg3 = false;
   	}
   	
   	if(received_leg4==true){
   	//map legx/numAngle to pwm
   	pwm_leg4 = outputMin + ((angle_leg4 - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin);
   	bool received_leg4 = false;
   	}
   
   }
   }
   
   
   //these callback functions are called by the subscriber. They take the messages and set variables to them
void rangeCallbackLeg1(const std_msgs::Float32::ConstPtr& msg) {
    angle_leg1 = msg->numAngle;
    received_leg1 = true;
}

void rangeCallbackLeg2(const std_msgs::Float32::ConstPtr& msg) {
    angle_leg2 = msg->numAngle;
    received_leg2 = true;
}

void rangeCallbackLeg3(const std_msgs::Float32::ConstPtr& msg) {
    angle_leg3 = msg->numAngle;
    received_leg3 = true;
}

void rangeCallbackLeg4(const std_msgs::Float32::ConstPtr& msg) {
    angle_leg4 = msg->numAngle;
    received_leg4 = true;
}
   

int pwmMap(double input){
	double inputMin = 0;
	double inputMax = 80;
	uint8_t outputMin = 0;
	uint8_t outputMax = 255;
	//full travel is around 4"
   	return static_cast<int>(outputMin + ((input - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin));
}
