#include<Servo.h> 
double setpoint =1700;  
Servo esc;
float PID, val, error, previous_error, u, pid_p, pid_i, pid_d;
double kp = 0.04;//0.5
double ki=0.005;//0.01
double kd=0.1;//0.5
float sum = 1000; 


void setup() {
  
  esc.attach(8); 

  // Initialize motors
  esc.writeMicroseconds(1000);
  Serial.begin(9600);
  delay(2000);
}
unsigned int time_ = millis();
void loop() {
  // Read the value from the feedback potentiometer
  int potValue = analogRead(A0);
  // Map the value withing 1000 and 3000 bounds
  double val = map(potValue, 585, 788, 1000, 3000);
  Serial.println(pwm);
  Serial.println(setpoint);
  // Calculate the error
  error = setpoint - val;
  // Calculate PID terms
  pid_p = kp * error;
  pid_i = sum + ki*error;
  pid_d = kd * (error - previous_error);
  // For safety if the controlling signal is withing bounds we ajdust the sum accordingly
 if (u >= 1000 && u < 3000){  
  sum = sum + ki*error;
 }
 // Calculate the controlling signal with the PID values
  u = pid_p + pid_i + pid_d;
   
  // Ensure contolling signal values are within bounds
  if (u < 1000) {
    u = 1000;
  }
  if (u > 3000) {
    u = 3000;
  }
  // Send the new controlling signal to the ESC
  esc.writeMicroseconds(u);
  previous_error = error;
  Serial.println(u);
  delay(150);

}
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
float convertinput(float input)
{
  float angle;
  //angle = 
  return angle;
}