#include <util/atomic.h>

#define direction_Pin  12
#define pwm_Pin  3
#define brake_Pin  9
#define encA_Pin  2

// globals
long prevT = 0;
int posPrev = 0;

// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;

float v1Filt = 0;
float v1Prev = 0;

float eintegral = 0;

void setup() {
  Serial.begin(115200);

  pinMode(encA_Pin,INPUT);
  pinMode(pwm_Pin,OUTPUT);
  pinMode(direction_Pin,OUTPUT);
  pinMode(brake_Pin,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encA_Pin),
                  readEncoder,RISING);
}

void loop() {
  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
  }
  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;
  
  // Convert count/s to RPM
  float v1 = velocity1/800.0*60.0;
  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  
  // Set a target
  float vt = 20;

  // Compute the control signal u
  float kp = 20;
  float ki = 10;
  float e = vt-v1Filt;
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  setMotor(dir,pwr,pwm_Pin,direction_Pin);
  
  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.println();
  delay(1);
}

void setMotor(int dir, int pwmVal, int pwm, int dirPin){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(dirPin,HIGH);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(dirPin,LOW);
  }
}

void readEncoder(){
  pos_i++;
}
