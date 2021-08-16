#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <autobot_control/velocity_read.h>

#include <PID_v1.h>
#include <L298NX2.h>
#define PWM_Left            6
#define PWM_Right           7
#define InLeftA             22
#define InLeftB             24
#define InRightA            26
#define InRightB            28


#define ENC_COUNT_REV 330

#define inputCLKright 3
#define inputDTright 2

#define inputCLKleft 21
#define inputDTleft 20

volatile long encoder_total_right = 0, encoder_total_left = 0;

volatile long encoderValueright = 0;
volatile long encoderValueleft = 0;

long previousMillis_right = 0;
long currentMillis_right = 0;
long previousMillis_left = 0;
long currentMillis_left = 0;
long currentMillis = 0;
long previousMillis = 0;

double rpm_right = 0;
double rpm_left = 0;

double kp_right = 25 , ki_right = 1 , kd_right = 0;
double kp_left = 25 , ki_left = 1 , kd_left = 0 ;

double radpersec_right = 0;
double radpersec_left = 0;

double rad_right, rad_left;

double pwm_right = 0;
double pwm_left = 0;

double SetSpeed_right; //rad/s
double SetSpeed_left; //rad/s

int interval = 10;

int currentStateRight;
int currentStateLeft;

int previousStateRight;
int previousStateLeft;

ros::NodeHandle  nh;



L298NX2 motors(PWM_Left, InLeftB, InLeftA, PWM_Right, InRightB, InRightA);

//float left_motor = 0, right_motor = 0, left_motor_pwm =0, right_motor_pwm=0;

PID right_pid(&radpersec_right, &pwm_right, &SetSpeed_right, kp_right, ki_right, kd_right, DIRECT);
PID left_pid(&radpersec_left, &pwm_left, &SetSpeed_left, kp_left, ki_left, kd_left, DIRECT);



void servo_cb( const rospy_tutorials::Floats & cmd_msg) {
  //nh.loginfo("Command Received");
  if (cmd_msg.data[0] == 0 && cmd_msg.data[1] == 0)
  {
    motors.stop();
  }
  else if (cmd_msg.data[0] > 0 && cmd_msg.data[1] > 0)
  {
    motors.forward();
  }
  else if (cmd_msg.data[0] < 0 && cmd_msg.data[1] > 0)
  {
    motors.forwardA();
    motors.backwardB();
  }
  else if (cmd_msg.data[0] > 0 && cmd_msg.data[1] < 0)
  {
    motors.forwardB();
    motors.backwardA();
  }
  else if (cmd_msg.data[0] < 0 && cmd_msg.data[1] < 0)
  {
    motors.backward();
  }
  //  if (cmd_msg.data[0] > 7){
  //    cmd_msg.data[0] = 7;
  //  }
  //  else if(cmd_msg.data[1] > 7){
  //    cmd_msg.data[1] = 7;
  //  }


  SetSpeed_left = abs(cmd_msg.data[0]);
  SetSpeed_right = abs(cmd_msg.data[1]);




}

void callback(const autobot_control::velocity_read::Request & req,  autobot_control::velocity_read::Response & res)
{

  res.res[0] = radpersec_left;
  res.res[1] = radpersec_right;
  res.res[2] = rad_left;
  res.res[3] = rad_right;

  return;

}



ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_arduino", servo_cb);
ros::ServiceServer<autobot_control::velocity_read::Request, autobot_control::velocity_read::Response> server("/read_joint_state", &callback);


void setup() {

  nh.initNode();
  nh.subscribe(sub);
  nh.advertiseService(server);

  pinMode(inputCLKright, INPUT_PULLUP);
  pinMode(inputCLKleft, INPUT_PULLUP);
  pinMode(inputDTright, INPUT_PULLUP);
  pinMode(inputDTleft, INPUT_PULLUP);



  previousStateRight = digitalRead(inputCLKright);
  previousStateLeft = digitalRead(inputCLKleft);

  attachInterrupt(digitalPinToInterrupt(inputCLKright), updateEncoder_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(inputDTright), updateSpeed_right, RISING);

  attachInterrupt(digitalPinToInterrupt(inputCLKleft), updateEncoder_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(inputDTleft), updateSpeed_left, RISING);

  previousMillis_right = millis();
  previousMillis_left = millis();
  previousMillis = millis();
  motors.stop();
  rad_right = 0;
  rad_left = 0;

  left_pid.SetMode(AUTOMATIC);
  left_pid.SetSampleTime(1);
  left_pid.SetOutputLimits(0, 200);
  right_pid.SetMode(AUTOMATIC);
  right_pid.SetSampleTime(1);
  right_pid.SetOutputLimits(0, 200);
}

void loop() {

  // Simulate function running for a non-deterministic amount of time
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    rpm_left = (float)(encoderValueleft * 60 / ENC_COUNT_REV);
    radpersec_left = (float)(rpm_left * 0.1472);

    rpm_right = (float)(encoderValueright * 60 / ENC_COUNT_REV);
    radpersec_right = (float)(rpm_right * 0.1472);


    encoderValueright = 0;
    encoderValueleft = 0;
  }

  rad_right = ((360 / ENC_COUNT_REV) * encoder_total_right) * (3.14 / 180);
  rad_left = ((360 / ENC_COUNT_REV) * encoder_total_left) * (3.14 / 180);
  //  previousMillis = currentMillis;

  //  rpm_left = (float)(encoderValueleft * 60 / ENC_COUNT_REV);
  //  radpersec_left = (float)(rpm_left * 0.1472);
  //
  //  rpm_right = (float)(encoderValueright * 60 / ENC_COUNT_REV);
  //  radpersec_right = (float)(rpm_right * 0.1472);
  //
  //
  //  encoderValueright = 0;
  //  encoderValueleft = 0;



  //  currentMillis_left = millis();
  //  if (currentMillis_left - previousMillis_left > interval) {
  //    previousMillis_left = currentMillis_left;
  //    rpm_left = (float)(encoderValueleft * 60 / ENC_COUNT_REV);
  //    radpersec_left = (float)(rpm_left * 0.1472);
  //
  //    encoderValueleft = 0;
  //  }
  //  currentMillis_right = millis();
  //  if (currentMillis_right - previousMillis_right > interval) {
  //    previousMillis_right = currentMillis_right;
  //    rpm_right = (float)(encoderValueright * 60 / ENC_COUNT_REV);
  //    radpersec_right = (float)(rpm_right * 0.1472);
  //
  //    encoderValueright = 0;
  //  }


  right_pid.Compute();
  left_pid.Compute();
  motors.setSpeedB(pwm_left);
  motors.setSpeedA(pwm_right);



  nh.spinOnce();
}



void updateSpeed_right()
{
  encoderValueright++;

}

void updateSpeed_left()
{
  encoderValueleft++;

}

void updateEncoder_right()
{
  currentStateRight = digitalRead(inputCLKright);

  if (currentStateRight != previousStateRight && currentStateRight == 1) {
    // Increment value for each pulse from encoder
    if (digitalRead(inputDTright) != currentStateRight) {
      encoder_total_right--;
    }
    else {
      encoder_total_right++;
    }
  }
  previousStateRight = currentStateRight;
}

void updateEncoder_left()
{
  currentStateLeft = digitalRead(inputCLKleft);

  if (currentStateLeft != previousStateLeft && currentStateLeft == 1) {

    // Increment value for each pulse from encoder
    if (digitalRead(inputDTleft) != currentStateLeft) {
      encoder_total_left--;
    }
    else {
      encoder_total_left++;
    }
  }
  previousStateLeft = currentStateLeft;
}
