#define pwm 9.
#define en1 10
#define en2 11
#define pos_in A0 //input from actuator //constants
double errorlast; double error; double K_p = 1.5; double K_i = 0.4; double K_d = 0.17; double dt;
double output;
double integral = 0;
double derivative = 0;
double des_pos = 0;
unsigned long timeAt1 = 0; unsigned long timeAt2 = 1000; void setup() {
// initialize Pins Serial.begin(9600); pinMode(pwm, OUTPUT); pinMode(en1, OUTPUT); pinMode(en2, OUTPUT); pinMode(pos_in, INPUT);
}
void loop() {
// des_pos = 400;
// moveTo(des_pos);
Serial.print(analogRead(pos_in)); Serial.print(','); Serial.println(des_pos); unsigned long currentTime; currentTime = millis();
if (currentTime - timeAt2 >= 2000 && timeAt1 < timeAt2) { des_pos = 200;
moveTo(des_pos);
timeAt1 = millis();
} else {
moveTo(des_pos); }
currentTime = millis();
if (currentTime - timeAt1 >= 2000 && timeAt2 < timeAt1) {
des_pos = 900; moveTo(des_pos); timeAt2 = millis();
} else {
moveTo(des_pos); }
}
/*
* This function will move the actuator to the position desired position
* @param des_pos = the desired position ranging between (fully retracted - fully extended) * @return boolean indicating whether or not the position was reached.
*/
void moveTo(double des_pos){
digitalWrite(en1,LOW); digitalWrite(en2,LOW); analogWrite(pwm, 0); boolean good = false; error = 0;
integral = 0; derivative = 0; double prevtime = 0; double currtime = 0; while(!good) {
//Serial.println(analogRead(pos_in));
dt = 0.007;
errorlast = error;
error = des_pos - analogRead(pos_in); if (abs(error) <= 4) {
output = 0;
error = 0;
good = true;
analogWrite(pwm, 0); //Serial.println(analogRead(pos_in)); return;
} else {
output = ((K_p * error) + (K_i * integral)) + (K_d * derivative);

//Serial.println(error); }
if (output > 255) {
output = 255; }
if(output < -255) {
output = -255; }
integral += error*dt;
derivative = (error - errorlast) / dt; Serial.print(analogRead(pos_in)); Serial.print(','); Serial.println(des_pos);
if(output >= 0) {
digitalWrite(en1,HIGH); digitalWrite(en2,LOW); analogWrite(pwm, abs(output));
}
if(output < 0){
digitalWrite(en2,HIGH); digitalWrite(en1,LOW); analogWrite(pwm, abs(output));
} }
}
}
