#include <Arduino.h>
#include <SPI.h>
#include "Kinematics.h"
#include "Encoder.h"
#include "PID.h"
#include "Motor.h"
#include "Odometry.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Range.h"
#include <NewPing.h>

#define EncoderCSLeft 6
#define EncoderCSRight 5
#define PID_MAX 2048
#define K_P 5.0   // P constant
#define K_I 0.025 // I constant
#define K_D 10.0  // D constant

#define SONAR_NUM 1     // Number of sensors.
#define MAX_DISTANCE 40 // Maximum distance (in cm) to ping.

//define your robot' specs here
#define TICKS_PER_REVOLUTION 64000 // Number of encoder ticks for full rotation
#define MAX_RPM 330                // motor's maximum RPM
#define WHEEL_DIAMETER 0.15        // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.35    // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.30    // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN

NewPing sonar[SONAR_NUM] = {
    // Sensor object array.
    NewPing(4, 3, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
};

Kinematics kinematics(Kinematics::DIFFERENTIAL_DRIVE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);
PID pidLeft(-PID_MAX, PID_MAX, K_P, K_I, K_D);
PID pidRight(-PID_MAX, PID_MAX, K_P, K_I, K_D);
Encoder encoder(EncoderCSLeft, EncoderCSRight, WHEEL_DIAMETER / 2, TICKS_PER_REVOLUTION);
Motor leftMotor(2);
Motor rightMotor(1);
Odometry odometry;

void resetCallback(const std_msgs::Empty &reset_msg);
ros::Subscriber<std_msgs::Empty> reset_sub("reset_wheel_odom", resetCallback);
void pidCallback(const geometry_msgs::Vector3 &vector_msg);
ros::Subscriber<geometry_msgs::Vector3> pid_sub("pid_update", pidCallback);
void cmdVelCallback(const geometry_msgs::Twist &cmd_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", cmdVelCallback);
ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/odom_wheel";
char odom[] = "/odom";
long lastCommandTime = 0;
long lastPingTime = 0;
Kinematics::rpm goalRPM;

geometry_msgs::Twist velTwist;
ros::Publisher velPub("raw_vel", &velTwist);

sensor_msgs::Range range;
ros::Publisher distPub("sonar", &range);

void cmdVelCallback(const geometry_msgs::Twist &cmd_msg)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored
  lastCommandTime = millis();
  goalRPM = kinematics.getRPM(cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.angular.z);
}

void pidCallback(const geometry_msgs::Vector3 &vector_msg)
{
  pidLeft = PID(-PID_MAX, PID_MAX, vector_msg.x, vector_msg.y, vector_msg.z);
  pidRight = PID(-PID_MAX, PID_MAX, vector_msg.x, vector_msg.y, vector_msg.z);
}

void resetCallback(const std_msgs::Empty &reset_msg)
{
  odometry.reset();
}

void setup()
{
  Serial.begin(115200);

  //Setup ROS
  nh.initNode();
  nh.subscribe(reset_sub);
  nh.subscribe(cmd_sub);
  nh.subscribe(pid_sub);
  nh.advertise(velPub);
  nh.advertise(distPub);
  broadcaster.init(nh);
  while (!nh.connected())
  {
    nh.spinOnce();
  }
  // goalRPM = kinematics.getRPM(0.2, 0, 0);
}

void loop()
{
  delay(10);

  if (millis() - lastCommandTime > 400)
  {
    goalRPM = kinematics.getRPM(0, 0, 0);
  }

  Encoder::RPM rpm = encoder.getRPM();
  leftMotor.adjust(pidLeft.compute(goalRPM.motor1, rpm.left));
  rightMotor.adjust(pidRight.compute(goalRPM.motor2, rpm.right));
  Kinematics::velocities vel = kinematics.getVelocities(rpm.left, rpm.right, 0, 0);
  Odometry::Position position = odometry.calculatePosition(vel.linear_x, vel.angular_z);

  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = position.x;
  t.transform.translation.y = position.y;
  t.transform.rotation = tf::createQuaternionFromYaw(position.theta);
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
  nh.spinOnce();

  velTwist.linear.x = vel.linear_x;
  velTwist.angular.z = vel.angular_z;
  velPub.publish(&velTwist);

  if (millis() - lastPingTime > 50)
  {
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    {
      // delay(30); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
      range.header.stamp = nh.now();
      range.min_range = 0.01;
      range.max_range = MAX_DISTANCE / 100;
      range.field_of_view = 0.261799;
      float cm = sonar[i].ping_cm();
      if (cm == 0)
      {
        range.range = INFINITY;
      }
      else
      {
        range.range = (float)cm / 100;
      }
      distPub.publish(&range);
    }
  }
}