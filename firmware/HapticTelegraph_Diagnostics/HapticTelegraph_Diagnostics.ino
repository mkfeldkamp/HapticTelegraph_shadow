/*
  Starter code for Haptic Telegraph.  Use for diagnostics and initial build. 
  Prof. Tim Kowalewski 2024
  Copyleft, public domain. 
*/

// #define ENCODER_USE_INTERRUPTS
#include <Encoder.h>  // Teensy encoder library Examples-->Teensy-->Encoder
#include <HX711.h>  // https://github.com/bogde/HX711 bodge's library
#include <Servo.h>
Servo myservo;

///////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES  -- will be used by interrupts
volatile int motorPWMcmdA = 0;  // The actual low-level signed pwm command used by runMotorA
volatile int motorPWMcmdB = 0;  // e.g. motorPWMcmdA = desiredPWM + ditherPWN + deadBand ...
volatile int potA, potB;        // global variable for pot position reads
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Constants (in TitleCase;  variables are in camelCase)
// pins 7 & 8 reserver for TimerOne https://www.pjrc.com/teensy/td_libs_TimerOne.html
const char LedPin = 13;           // 13 is LED on teensy; will use it to indicate motors are live
const char MotorEnablePin = 12;   // Reads sleep pin on HX711; driven by Enable Switch; HIGH --> ON
const char MotorA1pin     =  4;   // Motor command pins (feeds into input of DRV8833 H bridges)
const char MotorA2pin     =  5;   // swaping 1 and 2 will reverse direction of motor
const char MotorB1pin     = 18;   //
const char MotorB2pin     =  6;   // 
const char LoadCellA_SCKpin = 1;  // HX711 Load cell pins;  Check carefully; one writes, one reads
const char LoadCellA_DTpin  = 0;  //  DT: Data outpt from load cell (to teensy, which reads it)
const char LoadCellB_SCKpin = 3;  //  SCK: Clock input to load cell (from teensy, which writes it)
const char LoadCellB_DTpin  = 2;  //

// Load cell calibration values ...
const float LoadCellA_scale = 420.1;  // 5kg  TODO: you need to calibrate this yourself.
const float LoadCellB_scale = 420.0;  // 5kg  scale*RawLoadCell = grams
HX711 loadCellA;
HX711 loadCellB;
// Motor feedback; N20 gear motors have digital encoders, servo's have analog potentiometers
// make sure encoder pins are interruptable
const char EncApin_aw = 8;   // Motor A; enoder channel a, white wire
const char EncApin_by = 9;   // Motor A; enoder channel b, white wire
const char EncBpin_aw = 10;  // Motor B; enoder channel a, white wire
const char EncBpin_by = 11;  // Motor B; enoder channel b, white wire
// const float EncCountsToDeg = 360.0f / (12.0f * 100.0f);  // Encoder counts per revol. (CPR) x Gear Ratio;
const float EncCountsToDeg = 360.0f / (12.0f * 298.0f);  // Encoder counts per revol. (CPR) x Gear Ratio;
Encoder encoderA(EncApin_aw, EncApin_by);
Encoder encoderB(EncBpin_aw, EncBpin_by);

// Potentiometer position feedback (modified servo)   TODO: you find these three vals for each servo
const char PotApin    = A1;                             // Servo A potentiometer
const char PotBpin    = A0;                             // Servo B potentiometer
const int PotMinA     = 43;                             // end stop of servo
const int PotMaxA     = 976;                            // end stop of servo
const float RngDegA   = 195;                            // endstop-to-endstop travel in degrees (use protractor)
const float Pot2DegA  = RngDegA / (PotMaxA - PotMinA);  // degree per pot val
const int PotMinB     = 44;                             // end stop of servo
const int PotMaxB     = 976;                            // end stop of servo
const float RngDegB   = 195;                            // travel between end stops in degrees
const float Pot2DegB  = RngDegB / (PotMaxB - PotMinB);  // degree per pot val


// Each servo has unique pots; compute servo-specific degrees here
float getServoPosA() {
  potA = analogRead(PotApin);
  return ((float)potA - PotMinA) * Pot2DegA - 90.0;  // make origin 0 so +/-90deg travel
}
float getServoPosB() {
  potB = analogRead(PotBpin);
  return ((float)potB - PotMinB) * Pot2DegB - 90.0;
}


// Motor items; PWM resolution and frequency; see https://www.pjrc.com/teensy/td_pulse.html
const int PwmResolution = 12;
const int MaxPWM = pow(2, PwmResolution) - 1;  // e.g. 8bit resolution has 255 max
const int PwmFrequency = 450;                  // Hz

// runs a motor and handles negative values (reverses H bridge, applies PWN
// takes values  -MaxPWM <= u <= MaxPWM  where MaxPWM is max 100% pwm write of analogWrite()
inline void runMotorA(int u) {
  // for negative u, run H-bridge in reverse; otherwise just pass command through
  motorPWMcmdA = u;
  if (motorPWMcmdA < 0) {
    // if ( motorPWMcmdA < -MaxPWM) {
    //   Serial.println("% Warning: Clipping motor output to " + String(-MaxPWM));
    //   motorPWMcmdA = MaxPWM;
    // }
    analogWrite(MotorA1pin,  0            );
    analogWrite(MotorA2pin, -motorPWMcmdA );
  } else {
    // if ( motorPWMcmdA> MaxPWM) {
    //   Serial.println("% Warning: Clipping motor output to " + String(MaxPWM));
    //   motorPWMcmdA = MaxPWM;
    // }
    analogWrite(MotorA1pin,  motorPWMcmdA );
    analogWrite(MotorA2pin,  0            );
  } 
}

inline void runMotorB(int u) {  
  motorPWMcmdB = u;
  // for negative u, run H-bridge in reverse; otherwise just pass command through
  if ( motorPWMcmdB < 0) {
    analogWrite(MotorB1pin,  0            );
    analogWrite(MotorB2pin, -motorPWMcmdB );
  } else {
    analogWrite(MotorB1pin,  motorPWMcmdB );
    analogWrite(MotorB2pin,  0            );
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//   SETUP  Code in this function only runs once; initialize stuff, default values
///////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  // put your setup code here, to run once:
  pinMode(LedPin, OUTPUT);
  pinMode(MotorEnablePin, INPUT);
  pinMode(MotorA1pin, OUTPUT);
  pinMode(MotorA2pin, OUTPUT);    
  pinMode(MotorB1pin, OUTPUT);  
  pinMode(MotorB2pin, OUTPUT);
  analogWriteFrequency(MotorA1pin, PwmFrequency);
  analogWriteFrequency(MotorA2pin, PwmFrequency);
  analogWriteFrequency(MotorB1pin, PwmFrequency);
  analogWriteFrequency(MotorA2pin, PwmFrequency);

  // turn motors off, etc.
  digitalWrite(MotorA1pin, LOW);
  digitalWrite(MotorA2pin, LOW);
  digitalWrite(MotorB1pin, LOW);
  digitalWrite(MotorB2pin, LOW);
  digitalWrite(LedPin, digitalRead(MotorEnablePin));  // LED on means motors enabled
  analogWriteResolution(PwmResolution);            // analogWrite value 0 to 4095, or 4096 for high

  // load cell setup ...
  loadCellA.begin(LoadCellA_DTpin, LoadCellA_SCKpin);
  loadCellB.begin(LoadCellB_DTpin, LoadCellB_SCKpin);
  loadCellA.set_scale(LoadCellA_scale);  // TODO you need to calibrate this yourself.
  loadCellB.set_scale(LoadCellB_scale);  // TODO you need to calibrate this yourself.

  // reset the scale to zero = 0  (assume startup load is desired unloaded 0 value)
  loadCellA.tare();
  loadCellB.tare();

  // set encoder value at current position; fyi encoder resets each power cycle
  encoderA.write(0);
  encoderB.write(0);

  // run serial at 115200 or greater; will need to interact over serial with decent latency
  Serial.begin(115200);
  Serial.println("Haptic Telegraph is running...");
  myservo.attach(22);
}



///////////////////////////////////////////////////////////////////////////////////////////////////////
//   LOOP   This function will repeat until power down;  Put your main code here
///////////////////////////////////////////////////////////////////////////////////////////////////////
int cmdPWM = 0, delta = 1;
float forceA, forceB, posA, posB;
unsigned long t0;  // timestamp
void loop() {

  // LED to indicate H Bridge status; led on -> enabled
  digitalWrite(LedPin, digitalRead(MotorEnablePin));
  t0 = millis();
  
  forceA = loadCellA.get_units();
  forceB = loadCellB.get_units();
  posA = getServoPosA();
  posB = getServoPosB();
  //int a = analogRead(A0) + analogRead( A1 ) + analogRead( A2  )+ analogRead( A3 )+ analogRead( A4 )+ analogRead( A5 ) + analogRead( A6 );
  //Serial.print(a);

  // // Print all values for serial plotter:
  //  // Print all sensor values to serial
  // Serial.print("ForceA:");
  // Serial.print(loadCellA.get_units());   Serial.print(", ");

  // //Serial.print("EncA:");
  // //Serial.print(encoderA.read() * EncCountsToDeg);  Serial.print(",");

  // Serial.print("PotApin:");
  // Serial.print(getServoPosA());  Serial.print(", ");

  // Serial.print(" ForceB:");
  // Serial.print(loadCellB.get_units());   Serial.print(", ");

  // Serial.print("EncB:");
  // Serial.print(encoderB.read() * EncCountsToDeg);  Serial.print(", ");

  // //Serial.print("PotBpin:");
  // //Serial.print(getServoPosA());  Serial.print(",");

  // Serial.print("cmdPWM:");
  // Serial.print(cmdPWM); Serial.print(",");
  // Serial.print(" of:"+String(MaxPWM)); Serial.print(", ");
  // // Serial.println("");


  // Print all sensor values to serial
  Serial.print(loadCellA.get_units());
  Serial.print("  ");
  Serial.print(loadCellB.get_units());
  Serial.print("    ");
  Serial.print(encoderA.read() * EncCountsToDeg);
  Serial.print("  ");
  Serial.print(encoderB.read() * EncCountsToDeg);
  Serial.print("    ");
  Serial.print(getServoPosA());
  Serial.print("  ");
  Serial.print(getServoPosB());
  Serial.print("    ");
  Serial.print(cmdA);
  Serial.print("  ");
  Serial.print(cmdB);
  Serial.print("    ");
  Serial.print(digitalRead(12));
  Serial.print("    ");
  Serial.println("// loadcellA B   encA B   potA B   cmdA B   EnableStatus");

  // drive motors forwards and backwards by small PWM ramp (20% of MaxPWM = 0.2*MaxPWM )
  //runMotorA( MaxPWM * 0.15 );
  //runMotorB( MaxPWM * 0.15 );

  //int v = MaxPWM * 0.2f;
  //runMotorA( cmd );  runMotorB( cmd );
  // if (cmd >  v) delta = -1;
  // if (cmd < -v) delta = 1;
  // cmd += delta;

  //delay(1);

  // // Force control; 100g
  // cmdPWM = -(loadCellA.get_units()-100) * 20;

  // // Position control 100 deg
  // cmdPWM = -(100-getServoPosA()) * 80;

  // runMotorA(cmdPWM);

  
  // // Force control; 100g
  //cmdPWM = -(100-loadCellB.get_units()) * 2;

  // // Position control 100 deg
  // // cmdPWM = (1-encoderA.read() * EncCountsToDeg) * 30;

  // runMotorB(cmdPWM);

  // Servo stuff
  int cmdS = map(encoderB.read() * EncCountsToDeg +15, -45, 45, 1000, 2000);
  myservo.writeMicroseconds( cmdS );
  Serial.print("PosA "); Serial.print( posA ) ; Serial.print("  ForceB ");  Serial.print(forceB);
  runMotorA( 0); //-forceA*10 );

  // teleoperation
  float cmdA, cmdB, errA=0, errB=0;
  cmdA = forceB*10 ;
  errA = cmdA*10;
  
  errB = forceA;
  cmdB = -5 * errB;
  // runMotorA(cmdA);
  runMotorB( cmdB );
  // cmdPWM = cmdA;
  Serial.print(" cmdB: ");
  Serial.print(errB);
  Serial.print(",");
  Serial.print(" deltaT[MS]: ");
  Serial.print(millis() - t0);
  Serial.println("");
}
