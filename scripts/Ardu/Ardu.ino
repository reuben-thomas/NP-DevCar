#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <SoftwareSerial.h>
#include <ros.h>

#include <std_msgs/UInt16.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"

ros::NodeHandle nh;
Servo servo;
Servo motor; 

void steering( const geometry_msgs::Twist& cmd_msg)
{
  
  int throttle = int(cmd_msg.linear.x)+90;
  int angle = int(cmd_msg.angular.z)+90;
  
  motor.write(throttle);
  servo.write(angle); // Set servo angle (0 - 180)

  digitalWrite(13, HIGH-digitalRead(13));  // Toggle LED  
}

ros::Subscriber<geometry_msgs::Twist> sub("car/cmd_vel", steering);

void setup()
{
  Serial.begin(57600);
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  motor.attach(3); // Attach it to pin 3
  servo.attach(9); // Attach it to pin 9
  motor.write(90);
  servo.write(90); // Set servo angle (0 - 180)
}

void loop()
{
  nh.spinOnce();
  delay(0.1);
}
