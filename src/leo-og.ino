#include <SCServo.h>
#include <Ticker.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define WHEEL_RAD 0.0375 //define wheel radius
#define PUL_PER_ROUND 30000 //define number of output pulses per rotation round
#define COUNT_PERIOD 0.02 //periodic timer counting period
#define LEFT_MOTOR_ID 1
#define RIGHT_MOTOR_ID 2
#define VEHICLE_D 0.1667 //vehicle left right wheel distance
#define VEHICLE_L 0.1778 //vehicle front rear wheel distance
#define STEERING_POS_MAX 2500 //steering motor max position limit
#define STEERING_POS_MIN 1700 //steering motor min position limit
#define STEERING_POS_MID 2079 //steering motor middle position (for the steering motor position initialization)
#define STEERING_RAD2POS_SLOPE 786.6273353 //linear mapping slope for actual steering angle in radius to motor position convertion
#define TURNING_RADIOUS_MIN 0.35 //calibrated minimum turning radius (for max/min steering angles)
#define POS_MAX_CORRES_RAD 0.535 //actual steering angle in radius corresponding to the max position limit
#define POS_MIN_CORRES_RAD -0.482 //actual steering angle in radius corresponding to the min position limit
#define param 1


//define a struct for vehicle control (actual states VS target states)
typedef struct wheel_ctr_param {
  int32_t wheel_vel_L, wheel_vel_R;
  int32_t target_vel_L, target_vel_R;
  int16_t steering_position, target_position;
  int32_t errL, err_preL, err_pre_preL;
  int32_t errR, err_preR, err_pre_preR;
  int16_t pwm_outputL, pwm_outputR;
} wheel_ctr_param;

// Set struct and params for PID control
wheel_ctr_param wheel_ctr;
SMS_STS sms_sts;
float p_value = 6000, i_value = 50, d_value = 0;
float encoder2linear_v = 2.0 * PI * WHEEL_RAD / PUL_PER_ROUND / COUNT_PERIOD * 1000000;

//void twist_cmd_callback(const geometry_msgs::Twist& twist_cmd){
//  float turning_radius, steering_angle, v_L, v_R;
//  
//  if(twist_cmd.angular.z != 0) {
//    turning_radius = twist_cmd.linear.x / twist_cmd.angular.z;
//    if(abs(turning_radius < TURNING_RADIOUS_MIN))
//      turning_radius = (turning_radius / abs(turning_radius)) * TURNING_RADIOUS_MIN;
//    steering_angle = atan(VEHICLE_L / turning_radius);
//    v_L = twist_cmd.linear.x * (turning_radius - VEHICLE_D/2) / turning_radius;
//    v_R = twist_cmd.linear.x * (turning_radius + VEHICLE_D/2) / turning_radius;
//  }
//  else {
//    steering_angle = 0.0;
//    v_L = v_R = twist_cmd.linear.x;
//  }
//
//  wheel_ctr.target_vel_L = int32_t(v_L * 1000000);
//  wheel_ctr.target_vel_R = int32_t(v_R * 1000000);
//  wheel_ctr.target_position = int16_t(STEERING_POS_MIN + (steering_angle - POS_MIN_CORRES_RAD) * STEERING_RAD2POS_SLOPE);
//}

//callback funtion that sets the target left and right rear wheel linear speeds, and steering angle
//note: use steering_vel_cmd.linear.x as left rear motor speed in m/s !!!
//note: use steering_vel_cmd.linear.y as right rear motor speed in m/s !!!
//note: use steering_vel_cmd.angular.z as steering angle in rad !!!
void steering_vel_cmd_callback(const geometry_msgs::Twist& steering_vel_cmd){
    wheel_ctr.target_vel_L = int32_t(steering_vel_cmd.linear.x * 1000000);
    wheel_ctr.target_vel_R = int32_t(steering_vel_cmd.linear.y * 1000000);
    wheel_ctr.target_position = int16_t(STEERING_POS_MIN + (steering_vel_cmd.angular.z - POS_MIN_CORRES_RAD) * STEERING_RAD2POS_SLOPE); 
}

//define ROS publication & subscription for steering angle & left/right motor commands/feedbacks
ros::NodeHandle nh;
//geometry_msgs::Twist vehicle_current_twist;
geometry_msgs::Twist vehicle_current_steering_vel;
//ros::Subscriber<geometry_msgs::Twist> twist_cmd_sub("vehicle_target_twist", &twist_cmd_callback);
ros::Subscriber<geometry_msgs::Twist> steering_vel_cmd_sub("/vehicle_target_steering_vel", &steering_vel_cmd_callback);
//ros::Publisher vehicle_twist_pub("vehicle_current_twist", &vehicle_current_twist);
ros::Publisher vehicle_steering_vel_pub("/vehicle_current_steering_vel", &vehicle_current_steering_vel);

// Define motor input pins
const int motor_pin_left_a = 12;
const int motor_pin_left_b = 13;
const int motor_pin_right_a = 18;
const int motor_pin_right_b = 19;

// Define motor encoder pins
const int encoder_pin_left_a = 25;
const int encoder_pin_left_b = 26;
const int encoder_pin_right_a = 32;
const int encoder_pin_right_b = 33;

// Set PWM properties
const int freq = 1000;
const int motor_left_channel1 = 0;
const int motor_left_channel2 = 1;
const int motor_right_channel1 = 2;
const int motor_right_channel2 = 3;
const int resolution = 10;

// Motor readings variables & motor AB phase ISR functions
uint32_t motor_left_counter, motor_right_counter, motor_left_old, motor_right_old;
void IRAM_ATTR encoder_left_ISR() {
  int pinAState = digitalRead(encoder_pin_left_a);
  int pinBState = digitalRead(encoder_pin_left_b);

  if(pinAState == HIGH) {
    if(pinBState == LOW)
      motor_left_counter += 1;
    else
      motor_left_counter -= 1;
  }
  else {
    if(pinBState == HIGH)
      motor_left_counter += 1;
    else
      motor_left_counter -= 1;
  }
}

void IRAM_ATTR encoder_right_ISR() {
  int pinAState = digitalRead(encoder_pin_right_a);
  int pinBState = digitalRead(encoder_pin_right_b);

  if(pinAState == HIGH) {
    if(pinBState == LOW)
      motor_right_counter -= 1;
    else
      motor_right_counter += 1;
  }
  else {
    if(pinBState == HIGH)
      motor_right_counter -= 1;
    else
      motor_right_counter += 1;
  }
}

//motor control PWM value outputs; left and right motors have different polarities
void pwm_output_ctr(int16_t output_value, int8_t motor_id) {
    if(motor_id == LEFT_MOTOR_ID) {
      if(output_value >= 0) {
        ledcWrite(motor_left_channel1, output_value);
        ledcWrite(motor_left_channel2, 0);
      }
      else {
        ledcWrite(motor_left_channel2, -output_value);
        ledcWrite(motor_left_channel1, 0);      
      }        
    }
    else if(motor_id == RIGHT_MOTOR_ID) {
      if(output_value >= 0) {
        ledcWrite(motor_right_channel1, output_value);
        ledcWrite(motor_right_channel2, 0);
      }
      else {
        ledcWrite(motor_right_channel2, -output_value);
        ledcWrite(motor_right_channel1, 0);      
      }        
    }
}

//periodic PWM control & steering angle control callback function (every 20ms)
Ticker t;
void timerCallback() {
  noInterrupts();
  wheel_ctr.wheel_vel_L = (motor_left_counter - motor_left_old) * (int16_t)encoder2linear_v;
  wheel_ctr.wheel_vel_R = (motor_right_counter - motor_right_old) * (int16_t)encoder2linear_v;
  interrupts();

  int steering_pos_diff = 9999;
  sms_sts.WritePosEx(1, wheel_ctr.target_position, 0, 0);
  wheel_ctr.steering_position = sms_sts.ReadPos(1);
  if(!sms_sts.getErr())
    steering_pos_diff = abs(wheel_ctr.steering_position - wheel_ctr.target_position);    
//  delay(5);
    
  //wait for the steering motor to reach its target position and then drive the rear motors (could be improved with other techniques)
  if(steering_pos_diff <= 10) {
    //zero speed operation: let both rear motors brake
    if(wheel_ctr.target_vel_L == 0 && wheel_ctr.target_vel_R == 0) {
      ledcWrite(motor_left_channel1, 1000);
      ledcWrite(motor_left_channel2, 1000);
      ledcWrite(motor_right_channel1, 1000);
      ledcWrite(motor_right_channel2, 1000);    
    }
    //compute PWM output values for both left and right rear motors
    else {
      wheel_ctr.errL = wheel_ctr.target_vel_L - wheel_ctr.wheel_vel_L;
      wheel_ctr.errR = wheel_ctr.target_vel_R - wheel_ctr.wheel_vel_R;
      wheel_ctr.pwm_outputL += (int16_t)((p_value*(wheel_ctr.errL-wheel_ctr.err_preL) + i_value*wheel_ctr.errL + d_value*(wheel_ctr.errL - 2*wheel_ctr.err_preL + wheel_ctr.err_pre_preL))/1000000);
      wheel_ctr.pwm_outputR += (int16_t)((p_value*(wheel_ctr.errR-wheel_ctr.err_preR) + i_value*wheel_ctr.errR + d_value*(wheel_ctr.errR - 2*wheel_ctr.err_preR + wheel_ctr.err_pre_preR))/1000000);
      pwm_output_ctr(wheel_ctr.pwm_outputL, 1);
      pwm_output_ctr(wheel_ctr.pwm_outputR, 2);
      wheel_ctr.err_preL = wheel_ctr.errL; 
      wheel_ctr.err_preR = wheel_ctr.errR;
      wheel_ctr.err_pre_preL = wheel_ctr.err_preL; 
      wheel_ctr.err_pre_preR = wheel_ctr.err_preR;      
    }
  }
  else {
    ledcWrite(motor_left_channel1, 0);
    ledcWrite(motor_left_channel2, 0);
    ledcWrite(motor_right_channel1, 0);
    ledcWrite(motor_right_channel2, 0);
  }

  motor_left_old = motor_left_counter;
  motor_right_old = motor_right_counter;
}

void setup()
{
  //set up ROS publisher and subscriber for ROS_serial
  nh.initNode();
  nh.advertise(vehicle_steering_vel_pub);
//  nh.subscribe(twist_cmd_sub);
  nh.subscribe(steering_vel_cmd_sub);

  //Set up rear motor I/O pins & ISR functions
  pinMode(encoder_pin_left_a, INPUT);
  pinMode(encoder_pin_left_b, INPUT);
  pinMode(encoder_pin_right_a, INPUT);
  pinMode(encoder_pin_right_b, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_pin_left_a), encoder_left_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_pin_right_a), encoder_right_ISR, CHANGE);
  ledcSetup(motor_left_channel1, freq, resolution);
  ledcAttachPin(motor_pin_left_a, motor_left_channel1);
  ledcSetup(motor_left_channel2, freq, resolution);
  ledcAttachPin(motor_pin_left_b, motor_left_channel2);
  ledcSetup(motor_right_channel1, freq, resolution);
  ledcAttachPin(motor_pin_right_a, motor_right_channel1);
  ledcSetup(motor_right_channel2, freq, resolution);
  ledcAttachPin(motor_pin_right_b, motor_right_channel2);

  //set up steering motor serial communication interface
  Serial2.begin(1000000, SERIAL_8N1, 16, 17);
  sms_sts.pSerial = &Serial2;
  wheel_ctr.target_position = STEERING_POS_MID;

  //Initialize the timer
  t.attach_ms(20, timerCallback); 

  delay(1000);
}

uint8_t loop_counter = 0;
void loop()
{
  loop_counter += 1;

  //publish the motors' current states at a frequency around 22Hz
  //note: use vehicle_current_steering_vel.linear.x as left rear motor speed in m/s !!!
  //note: use vehicle_current_steering_vel.linear.y as right rear motor speed in m/s !!!
  //note: use vehicle_current_steering_vel.angular.z as steering angle in rad !!!
  if(loop_counter == 4) {
    vehicle_current_steering_vel.angular.z = (wheel_ctr.steering_position - STEERING_POS_MIN) / STEERING_RAD2POS_SLOPE + POS_MIN_CORRES_RAD;
    vehicle_current_steering_vel.linear.x = wheel_ctr.wheel_vel_L / 1000000.0;
    vehicle_current_steering_vel.linear.y = wheel_ctr.wheel_vel_R / 1000000.0;
    vehicle_steering_vel_pub.publish(&vehicle_current_steering_vel);

    nh.spinOnce();  
    loop_counter = 0;
  }
  delay(10);
}
