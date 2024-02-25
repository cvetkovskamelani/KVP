#include<Servo.h> 
float desired_angle;
int setpoint;
//funkcija za konverzija na vneseniot agol vo vrednost shto soodvetsvuva na potenciometarot
float convertinput(int input)
{
  float a;
  a = (0.0757*input*input)+(24.394*input)+2200;
  return a;
}

void inputAngle()
{
  setpoint=0;
  Serial.println("Enter a desired angle between -60 and 30 degrees");
  Serial.setTimeout(5000);
  setpoint = Serial.parseInt();
  desired_angle = convertinput(setpoint);
  
}
Servo esc;
float PID, pwm, error, previous_error, u, pid_p, pid_i, pid_d;
double kp = 0.04;//0.5
double ki=0.005;//0.01
double kd=0.1;//0.5
float sum = 1000; 


void setup() {
  
  esc.attach(8); 
 
  esc.writeMicroseconds(1000);
  Serial.begin(9600);
  delay(2000);
  inputAngle();
}
unsigned int time_ = millis();
void loop() {
  int potValue = analogRead(A0);
  Serial.print('\n');
  double pwm = map(potValue, 446, 788, 1000, 3000);//map(potValue, 585, 788, 1000, 3000);
  Serial.println(pwm);
  Serial.println(setpoint);
  //error = setpoint - pwm;
  error = desired_angle - pwm;
  // Calculate PID terms
  pid_p = kp * error;
  pid_i = sum + ki*error;
  pid_d = kd * (error - previous_error);
 if (u >= 1000 && u < 3000){  
  sum = sum + ki*error;
 }
  u = pid_p + pid_i + pid_d;
   
  // Ensure PWM values are within bounds
  if (u < 1000) {
    u = 1000;
  }
  if (u > 3000) {
    u = 3000;
  }
  
  esc.writeMicroseconds(u);
  previous_error = error;
  Serial.println(u);
  delay(150);
/*
  if (millis() - time_ >= 30000) {
    if (setpoint == 1700) {
      setpoint = 1900;
    } else {
      setpoint = 1300;
    }

    time_ = millis();
  }
*/
}

