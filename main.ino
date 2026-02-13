#include <PID_v1.h> // PID library

#define interruptPin 21
#define enA 8
#define in1 3
#define in2 2

unsigned long timestamp = 0L; // Timestamp initializer 
double pulses = 0; // Pulse count initializer
double Setpoint, Input, Output; // Setpoint is wanted speed, Input is measured rpm from sensor, Output is PWM signal to DC motor
double prev_pulses = 0; // Saves pulsecount from previous rpm sample
double Kp=0.0012927, Ki=0.0051706, Kd=0; // PID parameters

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // Uses defined variables as arguments to PID controller library function

void count() { // Interrupt function routine for pulse counting
 pulses++;
}

void setup() {

  pinMode(interruptPin, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count, RISING); // Interrupt routine that counts every rising pulse from sensor
  
  
  digitalWrite(in1, LOW); // Set initial rotation direction
  digitalWrite(in2, HIGH);
  
  Serial.begin(9600);
  Setpoint = 500; // Defines wanted rpm
  myPID.SetMode(AUTOMATIC); // PID mode set to automatic so it automatically performs speed adjustments
  

}


void measureRPM(){ // Function that reads a sample of rpm every 0.5 second from sensor


  if (millis () - timestamp >= 500)  // triggered every half second
  {
    noInterrupts(); // No interrupts while taking speed sample
    timestamp += 500 ;  // setup for next time
    double now_pulses = pulses;  // sample the count
    double count = now_pulses - prev_pulses; // take difference from last sample
    prev_pulses = now_pulses ;  // update the previous value
    double rpm = count*120/1024; // convert pulses counted in a 0.5 s timeframe to rotations per minute
    Input = rpm; // Declare measured speed as input to PID controller
    Serial.print("RPM:");
    Serial.println(Input);
    interrupts(); // Turn interrupts on again after taking speed sample
    
  }
  }

void PWMout(){ // PWM function
  analogWrite(enA, Output); // Send PWM signal to L298N Enable pin
  }


void loop() { // Main loop function

  measureRPM(); // Measure rpm
  myPID.Compute(); // Compare speed to Setpoint and makes speed adjustment computation
  PWMout(); // Sends PWM voltage to DC motor that is proportional to the speed computation from PID controller
  
  }
