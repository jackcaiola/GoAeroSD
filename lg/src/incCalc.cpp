//incline calculator
#include <ros/ros.h>
#include <iomanip>
#include <sensor_msgs/Range.h>
#include <iostream>
#include <cmath>
 
double range_leg1 = 0.0;
double range_leg2 = 0.0;
double range_leg3 = 0.0;
double range_leg4 = 0.0;
 
bool received_leg1 = false;
bool received_leg2 = false;
bool received_leg3 = false;
bool received_leg4 = false;
 
ros::Publisher numAngle_pub_leg1;
ros::Publisher numAngle_pub_leg2;
ros::Publisher numAngle_pub_leg3;
ros::Publisher numAngle_pub_leg4;
 
using namespace std;
 
// Function to calculate the inverse sine in degrees
double toDegrees(double radians) {
    return radians * 180.0 / M_PI;
}
 
// Function to calculate the normal vector (cross product)
void crossProduct(double v1[3], double v2[3], double result[3]) {
    result[0] = v1[1] * v2[2] - v1[2] * v2[1];
    result[1] = v1[2] * v2[0] - v1[0] * v2[2];
    result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}
 
// Function to calculate the plane equation coefficients from points
void calculatePlaneEquation(double P1[3], double P2[3], double P3[3], double &a, double &b, double &c, double &d) {
    // Create vectors
    double v1[3] = {P2[0] - P1[0], P2[1] - P1[1], P2[2] - P1[2]};
    double v2[3] = {P3[0] - P1[0], P3[1] - P1[1], P3[2] - P1[2]};
 
    // Find the normal vector (cross product)
    double normal[3];
    crossProduct(v1, v2, normal);
 
    // Plane equation: a(x - x1) + b(y - y1) + c(z - z1) = 0
    a = normal[0];
    b = normal[1];
    c = normal[2];
    d = -(a * P1[0] + b * P1[1] + c * P1[2]);
}
 
void rangeCallbackLeg1(const sensor_msgs::Range::ConstPtr& msg) {
    range_leg1 = msg->range;
    received_leg1 = true;
}
 
void rangeCallbackLeg2(const sensor_msgs::Range::ConstPtr& msg) {
    range_leg2 = msg->range;
    received_leg2 = true;
}
 
void rangeCallbackLeg3(const sensor_msgs::Range::ConstPtr& msg) {
    range_leg3 = msg->range;
    received_leg3 = true;
}
 
void rangeCallbackLeg4(const sensor_msgs::Range::ConstPtr& msg) {
    range_leg4 = msg->range;
    received_leg4 = true;
}
 
int main(int argc, char **argv) {
    ros::init(argc, argv, "incCalc"); // Initialize the incline calculator
 
    ros::NodeHandle n;
 
    // Subscribe to each of the four range topics
    ros::Subscriber sub_leg1 = n.subscribe("leg1/range", 1000, rangeCallbackLeg1);
    ros::Subscriber sub_leg2 = n.subscribe("leg2/range", 1000, rangeCallbackLeg2);
    ros::Subscriber sub_leg3 = n.subscribe("leg3/range", 1000, rangeCallbackLeg3);
    ros::Subscriber sub_leg4 = n.subscribe("leg4/range", 1000, rangeCallbackLeg4);
   
    // Publishers
    numAngle_pub_leg1 = n.advertise<std_msgs::Float32>("leg1/numAngle", 10);
    numAngle_pub_leg2 = n.advertise<std_msgs::Float32>("leg2/numAngle", 10);
    numAngle_pub_leg3 =     n.advertise<std_msgs::Float32>("leg3/numAngle", 10);
    numAngle_pub_leg4 = 
    n.advertise<std_msgs::Float32>("leg4/numAngle", 10);
   
 ros::Rate loop_rate(10);
while (ros::ok()) {
    ros::spinOnce();  // Process incoming messages
 
    if (received_leg1 && received_leg2 && received_leg3 && received_leg4) {
        // Compute z-values using received sensor data
        double raw_z[4] = {range_leg1, range_leg2, range_leg3, range_leg4};
        double max_z = *std::max_element(raw_z, raw_z + 4);
        double z_values[4];
 
        for (int i = 0; i < 4; i++) {
            z_values[i] = max_z - raw_z[i] - 5;  // Adjust heights as per requirement
        }
 
        // Calculate theta1 and publish to respective topics
        for (int i = 0; i < 4; i++) {
            double sin_theta1 = (z_values[i] + 3.5) / 3.0;
            std_msgs::Float32 angle_msg;
 
            if (sin_theta1 >= -1.0 && sin_theta1 <= 1.0) {
                angle_msg.data = toDegrees(asin(sin_theta1));
                if (angle_msg.data > 30.0) {
                    ROS_WARN("NO LAND");
                    continue; // Skip publishing this value
                }
            } else {
                angle_msg.data = 0.0; // Default if out of range
            }
 
            // Publish to correct topic
            if (i == 0) 
{ numAngle_pub_leg1.publish(angle_msg);
            }else if (i == 1){ numAngle_pub_leg2.publish(angle_msg);
            }else if (i == 2) { numAngle_pub_leg3.publish(angle_msg);
            }else if (i == 3){ numAngle_pub_leg4.publish(angle_msg);
        }}
    }
 
    loop_rate.sleep();
}
 
 
    // Define the points (x, y, z) in a 3D space
    double points[4][3] = {
        {0, 0, 0},      // P1
        {6.5, 0, 0},    // P2
        {0, -18, 0},    // P3
        {6.5, -18, 0}   // P4
    };
 
    // Input raw z-values for the points
    double raw_z[4];
    cout << "Enter raw z-values for the points:" << endl;
    //for (int i = 0; i < 4; i++) {
       
    //    cout << "Enter raw z-value for point (" << points[i][0] << ", " << points[i][1] << "): ";
    //    cin >> raw_z[i];
       
    //}
    raw_z[0] = range_leg1;
    raw_z[1] = range_leg2;
    raw_z[2] = range_leg3;
    raw_z[3] = range_leg4;
   
    // Find the  z-value
    double max_z = raw_z[0];
    for (int i = 1; i < 4; i++) {
        if (raw_z[i] > max_z) {
            max_z = raw_z[i];
        }
    }
   
    // Calculate the final z-values
    for (int i = 0; i < 4; i++) {
        points[i][2] = max_z - raw_z[i];
    }
   
    // Calculate the plane equation from points P1, P2, P3
    double a, b, c, d;
    calculatePlaneEquation(points[0], points[1], points[2], a, b, c, d);
 
    // Calculate z-values for new points
    double newPoints[4][2] = {
        {-3, 0},
        {9.5, 0},
        {-3, -18},
        {9.5, -18}
    };
 
    // Adjust all z-values so the lowest one becomes 0
    double min_z = newZ[0];
    for (int i = 1; i < 4; i++) {
        if (newZ[i] < min_z) {
            min_z = newZ[i];
        }
    }
 
    // Adjust z-values
    double z_values[4];
    for (int i = 0; i < 4; i++) {
        z_values[i] = newZ[i] - min_z;
        z_values[i] -= 5; // Subtract 5 as per requirement
    }
 
 
    // Calculate theta1 for each y2 using the equation y2 = 3sin(theta1) - 3.5
    cout << "\nCalculated theta1 values:" << endl;
    for (int i = 0; i < 4; i++) {
        double y2 = z_values[i];
 
        // Calculate sin(theta1) from y2 = 3sin(theta1) - 3.5
        double sin_theta1 = (y2 + 3.5) / 3.0;
 
        // Check if sin(theta1) is in the valid range [-1, 1]
        if (sin_theta1 >= -1.0 && sin_theta1 <= 1.0) {
            // Calculate theta1 in radians, then convert to degrees
            double theta1_rad = asin(sin_theta1);
            double theta1_deg = toDegrees(theta1_rad);
            cout << "For point (" << newPoints[i][0] << ", " << newPoints[i][1] << "): theta1 = " << theta1_deg << " degrees" << endl;
 
            // Check if theta1 is out of the allowed range
            if (theta1_deg > 30.0) {
                cout << "NO LAND" << endl;
                return 0;
            }
        } else {
            cout << "Invalid value for sin(theta1) at point (" << newPoints[i][0] << ", " << newPoints[i][1] << "), skipping..." << endl;
        }
    }
 
    return 0;
}
