#include <SCServo.h>
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
#define STEERING_POS_MAX 3225 //steering motor max position limit
#define STEERING_POS_MIN 2265 //steering motor min position limit
#define STEERING_POS_MID 2745 //steering motor middle position (for the steering motor position initialization)
#define STEERING_RAD2POS_SLOPE 960.62 //linear mapping slope for actual steering angle in radius to motor position convertion
#define POS_MAX_CORRES_RAD 0.483 //actual steering angle in radius corresponding to the max position limit
#define POS_MIN_CORRES_RAD -0.5225 //actual steering angle in radius corresponding to the min position limit
#define VELOCITY_TO_PWM_SCALE 500 // Scaling factor to convert m/s to PWM. Tune this value as needed.
#define PWM_MAX 1023 // Max PWM value based on 10-bit resolution

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

volatile int16_t pwm_L = 0;
volatile int16_t pwm_R = 0;
volatile unsigned long last_cmd_time = 0;

const float STEERING_ANGLE_MIN_RAD = -0.5225;
const float STEERING_ANGLE_MAX_RAD = 0.483;

// Simplified struct for vehicle control
typedef struct wheel_ctr_param {
  int32_t wheel_vel_L, wheel_vel_R;
  int16_t steering_position, target_position;
} wheel_ctr_param;

// Set struct
wheel_ctr_param wheel_ctr;
SMS_STS sms_sts;

// Conversion factor for encoder feedback
float encoder2linear_v = 2.0 * PI * WHEEL_RAD / PUL_PER_ROUND / COUNT_PERIOD * 1000000;

// Motor control PWM value outputs; left and right motors have different polarities
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

// New unified callback function to handle incoming Twist commands
// Note: steering_vel_cmd.linear.x is left motor speed in m/s
// Note: steering_vel_cmd.linear.y is right motor speed in m/s
// Note: steering_vel_cmd.angular.z is steering angle in radians
void updated_twist_callback(const geometry_msgs::Twist& twist_cmd)
{
    // --- Steering Control ---

    last_cmd_time = millis();
    

    float steering_angle = twist_cmd.angular.y;

    // Clamp the steering angle to safeguard the linkage
    if(steering_angle < STEERING_ANGLE_MIN_RAD)
    {
      steering_angle = STEERING_ANGLE_MIN_RAD;
    }
    else if(steering_angle > STEERING_ANGLE_MAX_RAD)
    {
      steering_angle = STEERING_ANGLE_MAX_RAD;
    }
    
    // Convert steering angle in radians to motor position and command the servo
    wheel_ctr.target_position = int16_t(STEERING_POS_MIN + (steering_angle - POS_MIN_CORRES_RAD) * STEERING_RAD2POS_SLOPE);

    // --- Throttle Control ---
    // If target velocities are zero, brake the motors
      // Convert target linear velocities directly to PWM values
      pwm_L = (int16_t)(twist_cmd.linear.x * 1000);
      pwm_R = (int16_t)(twist_cmd.linear.y * 1000);

      // Clamp PWM values to the maximum allowed range
      if (pwm_L > PWM_MAX) pwm_L = 800;
      if (pwm_L < -PWM_MAX) pwm_L = -800;
      if (pwm_R > PWM_MAX) pwm_R = 800;
      if (pwm_R < -PWM_MAX) pwm_R = -800;
}

// Define ROS publication & subscription
ros::NodeHandle nh;
geometry_msgs::Twist vehicle_current_steering_vel;
ros::Subscriber<geometry_msgs::Twist> updated_twist_sub("/updated_twist_topic", &updated_twist_callback);
ros::Publisher vehicle_steering_vel_pub("/vehicle_current_steering_vel", &vehicle_current_steering_vel);



// Motor readings variables & motor AB phase ISR functions
uint32_t motor_left_counter, motor_right_counter, motor_left_old, motor_right_old;
void IRAM_ATTR encoder_left_ISR() {
  int pinAState = digitalRead(encoder_pin_left_a);
  int pinBState = digitalRead(encoder_pin_left_b);

  if(pinAState == HIGH) {
    if(pinBState == LOW) motor_left_counter += 1;
    else motor_left_counter -= 1;
  }
  else {
    if(pinBState == HIGH) motor_left_counter += 1;
    else motor_left_counter -= 1;
  }
}

void IRAM_ATTR encoder_right_ISR() {
  int pinAState = digitalRead(encoder_pin_right_a);
  int pinBState = digitalRead(encoder_pin_right_b);

  if(pinAState == HIGH) {
    if(pinBState == LOW) motor_right_counter -= 1;
    else motor_right_counter += 1;
  }
  else {
    if(pinBState == HIGH) motor_right_counter -= 1;
    else motor_right_counter += 1;
  }
}

void setup()
{
  // Set up ROS publisher and subscriber for ROS_serial
  nh.initNode();
  nh.advertise(vehicle_steering_vel_pub);
  nh.subscribe(updated_twist_sub);

  // Set up rear motor I/O pins & ISR functions
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

  // Set up steering motor serial communication interface
  Serial2.begin(1000000, SERIAL_8N1, 16, 17);
  sms_sts.pSerial = &Serial2;
  wheel_ctr.target_position = STEERING_POS_MID;
  
  // No timer initialization needed

  delay(1000);
}

uint8_t loop_counter = 0;
void loop()
{

 if (millis() - last_cmd_time > 2000) {
    pwm_L = 0;
    pwm_R = 0;
  }
 // Send PWM commands to the motors
  pwm_output_ctr(pwm_L, LEFT_MOTOR_ID);
  pwm_output_ctr(pwm_R, RIGHT_MOTOR_ID);
  sms_sts.WritePosEx(1, wheel_ctr.target_position, 0, 0);
  
  nh.spinOnce();
  delay(10);
}