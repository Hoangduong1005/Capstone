#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

/*--------------Declare PIN of each wheels---------------*/
#define encodPinA1      2          //Encoder front right                  
#define encodPinA2      28         //Encoder front right                    
#define encodPinB1      3          //Encoder rear right                  
#define encodPinB2      29         //Encoder rear right
#define encodPinC1      19         //Encoder front left                   
#define encodPinC2      30         //Encoder front left
#define encodPinD1      18         //Encoder rear left                   
#define encodPinD2      31         //Encoder rear left

int enA = 8, fwdA = 50, revA = 51;  //Direction and Speed pin front right wheel
int enB = 9, fwdB = 52, revB = 53;  //Direction and Speed pin front left wheel
int enC = 10, fwdC = 22, revC = 23; //Direction and Speed pin rear right wheel
int enD = 11, fwdD = 24, revD = 25; //Direction and Speed pin rear left wheel
/*------------------------END---------------------------*/

/*------------------------Decalre PID parameter for each wheels----------*/
double kp_frontright = 1, ki_frontright = 20, kd_frontright = 0;
double kp_frontleft = 1, ki_frontleft = 20, kd_frontleft = 0;
double kp_rearright = 1, ki_rearright = 20, kd_rearright = 0;
double kp_rearleft = 1, ki_rearleft = 20, kd_rearleft = 0;

double input_frontright = 0, output_frontright = 0, setpoint_frontright = 0;
double input_frontleft = 0, output_frontleft = 0, setpoint_frontleft = 0;
double input_rearright = 0, output_rearright = 0, setpoint_rearright = 0;
double input_rearleft = 0, output_rearleft = 0, setpoint_rearleft = 0;
/*---------------------------------END-----------------------------------*/

/*--------------------------PID object for each wheels-------------------*/
PID front_right(&input_frontright, &output_frontright, &setpoint_frontright, kp_frontright, ki_frontright, kd_frontright, DIRECT);
PID front_left(&input_frontleft, &output_frontleft, &setpoint_frontleft, kp_frontleft, ki_frontleft, kd_frontleft, DIRECT);
PID rear_right(&input_rearright, &output_rearright, &setpoint_rearright, kp_rearright, ki_rearright, kd_rearright, DIRECT); 
PID rear_left(&input_rearleft, &output_rearleft, &setpoint_rearleft, kp_rearleft, ki_rearleft, kd_rearleft, DIRECT);    
/*---------------------------------END-----------------------------------*/

/*-----------------Setting parameter to take sampling time---------------*/
unsigned long previous_time, current_time;
volatile long encoderA = 0, encoderB = 0, encoderC = 0, encoderD = 0;
volatile long last_encoderA = 0, last_encoderB = 0, last_encoderC = 0, last_encoderD = 0;
/*---------------------------------END-----------------------------------*/

/*-----------------Setting parameter from Software System---------------*/

double speed_req = 0;
double angular_speed_req = 0;

const double radius = 0.08;                   //Wheel radius, in m
const double wheelbase = 0.25;               //Wheelbase, in m
const double encoder_cpr = 2970;               //Encoder ticks or counts per rotation
const double speed_to_pwm_ratio = 0.00235;    //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.0882;          //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 
/*---------------------------------END-----------------------------------*/

ros::NodeHandle nh;

/*-----------------Function will called receive from host---------------*/
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
  setpoint_frontleft = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  setpoint_frontright = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}
/*---------------------------------END-----------------------------------*/

void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = input_frontleft;    //left wheel speed (in m/s)
  speed_msg.vector.y = input_frontright;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

void setup() {
  /*----------------Declare setting pinmode of the wheels----------------*/
  pinMode(encodPinA1, INPUT_PULLUP);                  
  pinMode(encodPinA2, INPUT_PULLUP);         
  pinMode(encodPinB1, INPUT_PULLUP);                  
  pinMode(encodPinB2, INPUT_PULLUP);
  pinMode(encodPinC1, INPUT_PULLUP);                  
  pinMode(encodPinC2, INPUT_PULLUP);
  pinMode(encodPinD1, INPUT_PULLUP);                  
  pinMode(encodPinD2, INPUT_PULLUP);        

  attachInterrupt(digitalPinToInterrupt(encodPinA1), encoder_A, RISING);
  attachInterrupt(digitalPinToInterrupt(encodPinB1), encoder_B, RISING);
  attachInterrupt(digitalPinToInterrupt(encodPinC1), encoder_C, RISING);
  attachInterrupt(digitalPinToInterrupt(encodPinD1), encoder_D, RISING);

  pinMode(enA, OUTPUT);  // Front right
  pinMode(enC, OUTPUT);  // Front left
  pinMode(enB, OUTPUT); // Rear right
  pinMode(enD, OUTPUT); // Rear left

  pinMode(fwdA, OUTPUT); // Front right - IN1 
  pinMode(revA, OUTPUT); // Front right - IN2
  pinMode(revC, OUTPUT); // Front left  - IN1
  pinMode(revC, OUTPUT); // Front left  - IN2

  pinMode(fwdB, OUTPUT); // Rear right - IN1 
  pinMode(revB, OUTPUT); // Rear right - IN2 
  pinMode(fwdD, OUTPUT); // Rear left - IN1 
  pinMode(fwdD, OUTPUT); // Rear left - IN2
  /*------------------------END---------------------------------------*/ 

  /*----------------Setting PID object 4 wheels-----------------------*/
  front_right.SetMode(AUTOMATIC);
  front_right.SetSampleTime(95);
  front_right.SetOutputLimits(-255, 255);

  front_left.SetMode(AUTOMATIC);
  front_left.SetSampleTime(95);
  front_left.SetOutputLimits(-255, 255);

  rear_right.SetMode(AUTOMATIC);
  rear_right.SetSampleTime(95);
  rear_right.SetOutputLimits(-255, 255);

  rear_left.SetMode(AUTOMATIC);
  rear_left.SetSampleTime(95);
  rear_left.SetOutputLimits(-255, 255);
  /*------------------------END---------------------------------------*/

  /*----------------Creating Node for ROS-----------------------*/
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
  /*------------------------END---------------------------------------*/
}

void loop() {
  nh.spinOnce();
  current_time = millis();
  int timeChange = (current_time - previous_time);
  if(timeChange >= 100){
    previous_time = current_time; 
    if(abs(encoderC) < 5){
      input_frontleft = 0;
    }
     else {
      input_frontleft=((encoderC/encoder_cpr)*2*PI)*(1000/100)*radius;           // calculate speed of left wheel
    }
  
    if (abs(encoderA) < 5){                                                  //Avoid taking in account small disturbances
      input_frontright = 0;
    }
    else {
    input_frontright=((encoderA/encoder_cpr)*2*PI)*(1000/100)*radius;          // calculate speed of right wheel
    }

    encoderC = 0;
    encoderA = 0;

    output_frontleft = constrain(output_frontleft, -255, 255);
    front_left.Compute();
    PWM_leftMotor = constrain(((setpoint_frontleft+sgn(setpoint_frontleft)*min_speed_cmd)/speed_to_pwm_ratio) + (output_frontleft/speed_to_pwm_ratio), -255, 255); //

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      pwmOut(enC, fwdC, revC, 0);
      pwmOut(enD, fwdD, revD, 0);
    }
    else if (setpoint_frontleft == 0){                        //Stopping
      pwmOut(enC, fwdC, revC, 0);
      pwmOut(enD, fwdD, revD, 0);
    }
    else {                          //Going forward
      pwmOut(enC, fwdC, revC, PWM_leftMotor);
      pwmOut(enD, fwdD, revD, PWM_leftMotor);
    }

    output_frontright = constrain(output_frontright, -255, 255);
    front_right.Compute();
    PWM_rightMotor = constrain(((setpoint_frontright+sgn(setpoint_frontright)*min_speed_cmd)/speed_to_pwm_ratio) + (output_frontright/speed_to_pwm_ratio), -255, 255); //

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      pwmOut(enA, fwdA, revA, 0);
      pwmOut(enA, fwdA, revA, 0);
    }
    else if (setpoint_frontleft == 0){                        //Stopping
      pwmOut(enA, fwdA, revA, 0);
      pwmOut(enA, fwdA, revA, 0);
    }
    else {                          //Going forward
      pwmOut(enA, fwdA, revA, PWM_rightMotor);
      pwmOut(enA, fwdA, revA, PWM_rightMotor);
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }
    
    publishSpeed(100);   //Publish odometry on ROS topic
  }
}


/*----------------------------------Interrupt function to count pusle of 4 wheels-----------------------------------------*/
void encoder_A()  {                                     // pulse and direction, direct port reading to save cycles front right wheels 
  if (digitalRead(encodPinA2) == HIGH)    encoderA++;             
  else                      encoderA--;             
}

void encoder_B()  {                                     // pulse and direction, direct port reading to save cycles rear right wheels 
  if (digitalRead(encodPinB2) == HIGH)    encoderB++;             
  else                      encoderB--;            
}

void encoder_C()  {                                     // pulse and direction, direct port reading to save cycles front left wheels
  if (digitalRead(encodPinC2) == HIGH)    encoderC++;            
  else                      encoderC--;             
}

void encoder_D()  {                                     // pulse and direction, direct port reading to save cycles rear left wheels
  if (digitalRead(encodPinD2) == HIGH)    encoderD++;             
  else                      encoderD--;             
}
/*-----------------------------------------------------END-----------------------------------------------------------------*/

void pwmOut(int en_pin, int fwd_pin, int back_pin, int out){
  if(out > 0){
    analogWrite(en_pin, out);
    digitalWrite(fwd_pin, HIGH);
    digitalWrite(back_pin, LOW);
  }
  else{
    analogWrite(en_pin, abs(out));
    digitalWrite(fwd_pin, LOW);
    digitalWrite(back_pin, HIGH);
  }
}
