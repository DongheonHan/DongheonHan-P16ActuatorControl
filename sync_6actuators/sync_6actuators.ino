#define pwm1 13 //actuator 1 PWM
#define pwm2 12 //actuator 2 PWM
#define pwm3 11 //actuator 3 PWM
#define pwm4 10 //actuator 4 PWM
#define pwm5 9  //actuator 5 PWM
#define pwm6 8  //actuator 6 PWM

#define en1_1 53  
#define en1_2 52  
#define en2_1 51  
#define en2_2 50 
#define en3_1 49  
#define en3_2 48
#define en4_1 47  
#define en4_2 46
#define en5_1 45  
#define en5_2 44
#define en6_1 43  
#define en6_2 42
// logic input to Motor Driver L298N
#define pos_in_1 A1     // Motor Driver 1_1
#define pos_in_2 A2     // Motor Driver 1_2
#define pos_in_3 A3     // Motor Driver 2_1
#define pos_in_4 A4     // Motor Driver 2_2
#define pos_in_5 A5     // Motor Driver 3_1
#define pos_in_6 A6     // Motor Driver 3_2

double errorlast1; double errorlast2; double errorlast3; double errorlast4; double errorlast5; double errorlast6; 
double error1;  double error2;  double error3;  double error4;  double error5;  double error6;

double K_p = 1.8; double K_i = 0.02; double K_d = 0.05; double dt; double des_pos = 0;
double output1;  double integral1 = 0;  double derivative1 = 0;  
double output2;  double integral2 = 0;  double derivative2 = 0;  
double output3;  double integral3 = 0;  double derivative3 = 0;  
double output4;  double integral4 = 0;  double derivative4 = 0;  
double output5;  double integral5 = 0;  double derivative5 = 0; 
double output6;  double integral6 = 0;  double derivative6 = 0;  

void setup() {
  Serial.begin(9600);
  pinMode(en1_1, OUTPUT); pinMode(en1_2, OUTPUT); pinMode(pwm1, OUTPUT); pinMode(pos_in_1, INPUT);  //actuator 1 
  pinMode(en2_1, OUTPUT); pinMode(en2_2, OUTPUT); pinMode(pwm2, OUTPUT); pinMode(pos_in_2, INPUT);  //actuator 2 
  pinMode(en3_1, OUTPUT); pinMode(en3_2, OUTPUT); pinMode(pwm3, OUTPUT); pinMode(pos_in_3, INPUT);  //actuator 3
  pinMode(en4_1, OUTPUT); pinMode(en4_2, OUTPUT); pinMode(pwm4, OUTPUT); pinMode(pos_in_4, INPUT);  //actuator 4
  pinMode(en5_1, OUTPUT); pinMode(en5_2, OUTPUT); pinMode(pwm5, OUTPUT); pinMode(pos_in_5, INPUT);  //actuator 5
  pinMode(en6_1, OUTPUT); pinMode(en6_2, OUTPUT); pinMode(pwm6, OUTPUT); pinMode(pos_in_6, INPUT);  //actuator 6
}
void loop() {
  move6actuatorsTo(0, 25);
  delay(500);
  move6actuatorsTo(900, 25);
  delay(500);
}
void move6actuatorsTo(double des_pos, double err_tolerance) { 
  digitalWrite(en1_1, LOW); digitalWrite(en1_2, LOW); analogWrite(pwm1, 0); 
  digitalWrite(en2_1, LOW); digitalWrite(en2_2, LOW); analogWrite(pwm2, 0);
  digitalWrite(en3_1, LOW); digitalWrite(en3_2, LOW); analogWrite(pwm3, 0);
  digitalWrite(en4_1, LOW); digitalWrite(en4_2, LOW); analogWrite(pwm4, 0);
  digitalWrite(en5_1, LOW); digitalWrite(en5_2, LOW); analogWrite(pwm5, 0); 
  digitalWrite(en6_1, LOW); digitalWrite(en6_2, LOW); analogWrite(pwm6, 0);
  boolean good = false;
  error1 = 0; error2 = 0; error3 = 0; error4 = 0; error5 = 0; error6 = 0;
  integral1 = 0; derivative1 = 0;
  integral2 = 0; derivative2 = 0;
  integral3 = 0; derivative3 = 0;
  integral4 = 0; derivative4 = 0;
  integral5 = 0; derivative5 = 0;
  integral6 = 0; derivative6 = 0;
  
  double prevtime = 0; double currtime = 0;
  
  while (!good) {
    //Serial.println(analogRead(pos_in));
    dt = 0.005;
    errorlast1 = error1;  errorlast1 = error2;  errorlast1 = error3;  
    errorlast1 = error4;  errorlast1 = error5;  errorlast1 = error6;
    
    error1 = des_pos - analogRead(pos_in_1);
    error2 = des_pos - analogRead(pos_in_2);
    error3 = des_pos - analogRead(pos_in_3);
    error4 = des_pos - analogRead(pos_in_4);
    error5 = des_pos - analogRead(pos_in_5);
    error6 = des_pos - analogRead(pos_in_6);
    
    if (abs(error1) <= err_tolerance && abs(error2) <= err_tolerance && abs(error3) <= err_tolerance && abs(error4) <= err_tolerance && abs(error5) <= err_tolerance  && abs(error6) <= err_tolerance) {
      output1 = 0; output2 = 0; output3 = 0; output4 = 0; output5 = 0; output6 = 0;
      error1 = 0; error2 = 0; error3 = 0; error4 = 0; error5 = 0; error6 = 0;
      good = true;
      analogWrite(pwm1, 0); // duty cycle = 0%
      analogWrite(pwm2, 0); // duty cycle = 0%
      analogWrite(pwm3, 0); // duty cycle = 0%
      analogWrite(pwm4, 0); // duty cycle = 0%
      analogWrite(pwm5, 0); // duty cycle = 0%
      analogWrite(pwm6, 0); // duty cycle = 0%
      
    } else {
      output1 = ((K_p * error1) + (K_i * integral1)) + (K_d * derivative1); //Serial.println(error);
      output2 = ((K_p * error2) + (K_i * integral2)) + (K_d * derivative2); //Serial.println(error);
      output3 = ((K_p * error3) + (K_i * integral3)) + (K_d * derivative3); //Serial.println(error);
      output4 = ((K_p * error4) + (K_i * integral4)) + (K_d * derivative4); //Serial.println(error);
      output5 = ((K_p * error5) + (K_i * integral5)) + (K_d * derivative5); //Serial.println(error);
      output6 = ((K_p * error6) + (K_i * integral6)) + (K_d * derivative6); //Serial.println(error);
    }
    
    if (output1 >= 225) {
      output1 = 255;
    }
    if (output1 <= -225) {
      output1 = -255;
    }
    if (output2 >= 225) {
      output2 = 255;
    }
    if (output2 <= -225) {
      output2 = -255;
    }
    if (output3 >= 225) {
      output3 = 255;
    }
    if (output3 <= -225) {
      output3 = -255;
    }
    if (output4 >= 225) {
      output4 = 255;
    }
    if (output4 <= -225) {
      output4 = -255;
    }
    if (output5 >= 225) {
      output5 = 255;
    }
    if (output5 <= -225) {
      output5 = -255;
    }
    if (output6 >= 225) {
      output6 = 255;
    }
    if (output6 <= -225) {
      output6 = -255;
    }
    
    integral1 += error1 * dt;
    integral2 += error2 * dt;
    integral3 += error3 * dt;
    integral4 += error4 * dt;
    integral5 += error5 * dt;
    integral6 += error6 * dt;
    
    derivative1 = (error1 - errorlast1) / dt;
    derivative2 = (error2 - errorlast2) / dt;
    derivative3 = (error3 - errorlast3) / dt;
    derivative4 = (error4 - errorlast4) / dt;
    derivative5 = (error5 - errorlast5) / dt;
    derivative6 = (error6 - errorlast6) / dt;
    if (output1 >= 0) {
      digitalWrite(en1_1, HIGH); digitalWrite(en1_2, LOW); analogWrite(pwm1, abs(output1)); 
    }
    if (output1 < 0) {
      digitalWrite(en1_1, LOW); digitalWrite(en1_2, HIGH);  analogWrite(pwm1, abs(output1));
    }
    if (output2 >= 0) {
      digitalWrite(en2_1, HIGH); digitalWrite(en2_2, LOW); analogWrite(pwm2, abs(output2));
    }
    if (output2 < 0) {
      digitalWrite(en2_1, LOW); digitalWrite(en2_2, HIGH);  analogWrite(pwm2, abs(output2));
    }
    if (output3 >= 0) {
      digitalWrite(en3_1, HIGH); digitalWrite(en3_2, LOW); analogWrite(pwm3, abs(output3));
    }
    if (output3 < 0) {
      digitalWrite(en3_1, LOW); digitalWrite(en3_2, HIGH);  analogWrite(pwm3, abs(output3));
    }
    if (output4 >= 0) {
      digitalWrite(en4_1, HIGH); digitalWrite(en4_2, LOW); analogWrite(pwm4, abs(output4));
    }
    if (output4 < 0) {
      digitalWrite(en4_1, LOW); digitalWrite(en4_2, HIGH);  analogWrite(pwm4, abs(output4));
    }
    if (output5 >= 0) {
      digitalWrite(en5_1, HIGH); digitalWrite(en5_2, LOW); analogWrite(pwm5, abs(output5));
    }
    if (output5 < 0) {
      digitalWrite(en5_1, LOW); digitalWrite(en5_2, HIGH);  analogWrite(pwm5, abs(output5));
    }
    if (output6 >= 0) {
      digitalWrite(en6_1, HIGH); digitalWrite(en6_2, LOW); analogWrite(pwm6, abs(output6));
    }
    if (output6 < 0) {
      digitalWrite(en6_1, LOW); digitalWrite(en6_2, HIGH);  analogWrite(pwm6, abs(output6));
    }
  }
}
