#include <PID_v1.h>


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
double kp_frontleft = 0.5, ki_frontleft = 15, kd_frontleft = 0;
double kp_rearright = 5, ki_rearright = 20, kd_rearright = 0;
double kp_rearleft = 1.5, ki_rearleft = 18, kd_rearleft = 0;

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
  front_right.SetSampleTime(1);
  front_right.SetOutputLimits(-255, 255);

  front_left.SetMode(AUTOMATIC);
  front_left.SetSampleTime(1);
  front_left.SetOutputLimits(-255, 255);

  rear_right.SetMode(AUTOMATIC);
  rear_right.SetSampleTime(1);
  rear_right.SetOutputLimits(-255, 255);

  rear_left.SetMode(AUTOMATIC);
  rear_left.SetSampleTime(1);
  rear_left.SetOutputLimits(-255, 255);
  /*------------------------END---------------------------------------*/
  Serial.begin(9600);
  setpoint_frontright = 160;
  setpoint_frontleft = 160;
  setpoint_rearright = 160;
  setpoint_rearleft = 160;
}

void loop() {
  current_time = millis();
  int timeChange = (current_time - previous_time);
  if(timeChange >= 20)
  {
    input_frontright = (360.0*1000*encoderA) /(2970.0*(current_time - previous_time));
    input_frontleft = input_frontright;
    input_rearleft = (360.0*1000*encoderD) /(2970.0*(current_time - previous_time));
    input_rearright = input_rearleft;
   
    previous_time = current_time;
    encoderA = 0;
    encoderB = 0;
    encoderC = 0;
    encoderD = 0;
  }
  front_right.Compute();
  pwmOut(enA, fwdA, revA, output_frontright);
  front_left.Compute();
  pwmOut(enC, fwdC, revC, output_frontleft);
  rear_right.Compute();
  pwmOut(enB, fwdB, revB, output_rearright);
  rear_left.Compute();
  pwmOut(enD, fwdD, revD, output_rearleft);
  Serial.print(input_frontright);
  Serial.print(" ");
  Serial.print(input_frontleft);
  Serial.print(" ");
  Serial.print(input_rearright);
  Serial.print(" ");
  Serial.println(input_rearleft);
  
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
