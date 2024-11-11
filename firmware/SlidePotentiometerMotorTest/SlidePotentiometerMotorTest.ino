/*
  Starter code for Haptic Telegraphg:\My Drive\_Teaching\ME_8284\admin(staff)\HapticTelegraph\software\HapticTelegraph_SystemID\HapticTelegraph_SystemID.ino.  Use for diagnostics and initial build. 
  Prof. Tim Kowalewski 2024
  Copyleft, public domain. 
*/

#include <HX711.h>  // https://github.com/bogde/HX711 bodge's library
// #define ENCODER_USE_INTERRUPTS
#include <Encoder.h>  // Teensy encoder library Examples-->Teensy-->Encoder
#include <Servo.h>
Servo myservo;

// Constants (in TitleCase;  variables are in camelCase)
const char LedPin = 13;         // 13 is LED on teensy; will use it to indicate motors are live
// const char MotorEnable = 12;    // Reads sleep pin on HX711; driven by Enable Switch; HIGH --> ON
const char MotorA1 = 2;         // Motor command pins (feeds into input of DRV8833 H bridges)
const char MotorA2 = 3;         // swaping 1 and 2 will reverse direction of motor
const char MotorB1 = 4;         //
const char MotorB2 = 5;         //
const char LoadCellA_DT  = 35;  //  DT: Data outpt from load cell (to teensy, which reads it)
const char LoadCellA_SCK = 36;  // HX711 Load cell pins;  Check carefully; one writes, one reads
const char LoadCellB_DT  = 37;  //
const char LoadCellB_SCK = 38;  //  SCK: Clock input to load cell (from teensy, which writes it)

// Load cell calibration values ...
const float LoadCellA_scale = 420.1;  // 5kg  TODO: you need to calibrate this yourself.
const float LoadCellB_scale = 420.0;  // 5kg  TODO: you need to calibrate this yourself.
HX711 loadCellA;
HX711 loadCellB;
// Motor feedback; N20 gear motors have digital encoders, servo's have analog potentiometers
// make sure encoder pins are interruptable
const char EncA_aw = 8;   // Motor A; encoder channel a,  white wire
const char EncA_by = 9;   // Motor A; encoder channel b, yellow wire
const char EncB_aw = 10;  // Motor B; encoder channel a,  white wire
const char EncB_by = 11;  // Motor B; encoder channel b, yellow wire
// const float EncCountsToDeg = 360.0f / (12.0f * 100.0f);  // Encoder counts per revol. (CPR) x Gear Ratio;
const float EncCountsToDeg = 360.0f / (12.0f * 298.0f);  // Encoder counts per revol. (CPR) x Gear Ratio;
Encoder encoderA(EncA_aw, EncA_by);
Encoder encoderB(EncB_aw, EncB_by);

// Potentiometer position feedback (modified servo)   TODO: you find these three vals for each servo
const char PotA = A13;                                  // Servo A potentiometer
const char PotB = A14;                                  // Servo B potentiometer
const char PotLog= A12;
const int PotMinA = 1023;                                // end stop of servo
const int PotMaxA = 1;                               // end stop of servo
const float RngDistA = 100;                            // travel between end stops in mm (use protractor or datasheet)
const float PotToDistA = RngDistA / (PotMaxA - PotMinA);  // degree per pot val
const int PotMinB = 1023;                                // end stop of servo
const int PotMaxB = 0;                               // end stop of servo
const float RngDistB = 100;                             // travel between end stops in degrees
const float PotToDistB = RngDistB / (PotMaxB - PotMinB);  // degree per pot val
int potA, potB;                                        // global variable for pot position reads

// Each servo has unique pots; compute servo-specific degrees here
float getServoPosA() {
  potA = analogRead(PotA);
  return ((float)potA - PotMinA) * PotToDistA - 0.0;  // make origin 0 so +/-90deg travel
}
float getServoPosB() {
  potB = analogRead(PotB);
  return ((float)potB - PotMinB) * PotToDistB - 0.0;
}

// Global Variables
int cmdPWM = 0, delta = 1;
unsigned long t0, td;  // timestamp and time delta
float posA, posB, forceA, forceB, posAprev, posBprev; // previous measurement, previous previous prev2
float posDeltaA, velA, posDeltaB, velB;
float e = 0, ed = 0, e_1 = 0;

// Motor items; PWM resolution and frequency; see https://www.pjrc.com/teensy/td_pulse.html
const int   PwmResolution = 12;
const int   MaxPWM = pow(2, PwmResolution) - 1;  // e.g. 8bit resolution has 255 max
const float PwmFrequency = 488.28;//488.28 ;                  // Hz deafault for Teensy 3.5 488.28 Hz
// runs a motor and handles negative values (reverses H bridge, applies PWN
// takes values  -MaxPWM <= u <= MaxPWM  where MaxPWM is max 100% pwm write of analogWrite()
void runMotorA(int u) {
  // // for negative u, run H-bridge in reverse; otherwise just pass command through
  // if (u < 0) {

  //   if (u < -MaxPWM) {
  //     Serial.println("% Warning: Clipping motor output to " + String(-MaxPWM));
  //     u = MaxPWM;
  //   }
  //   analogWrite(MotorA1, 0);
  //   analogWrite(MotorA2, abs(u));
  // } else {
  //   if (u > MaxPWM) {
  //     Serial.println("% Warning: Clipping motor output to " + String(MaxPWM));
  //     u = MaxPWM;
  //   }
  //   analogWrite(MotorA1, u);
  //   analogWrite(MotorA2, 0);
  // }
  const int PWMstartAneg = -800*0, PWMstartApos = 0*800;
  // for negative commands, clip at max and min
  if (u < 0) {
    if (u < -MaxPWM) {
      Serial.println("% Warning: Clipping motor output to " + String(-MaxPWM));
      u = MaxPWM;
    } else if (abs(posDeltaA)<0.2 &&  (u > PWMstartAneg)) {u = -PWMstartAneg;} //if not moving and asked for small effort, give break-free effort
    
    analogWrite(MotorA2, abs(u));
    analogWrite(MotorA1, 0);
    
  } else {  // 0 < u  (positive), do the same as above but swapped
    if (MaxPWM < u) {
      Serial.println("% Warning: Clipping motor output to " + String(MaxPWM));
      u = MaxPWM;
    } else if ( abs(posDeltaA)<0.2 && ( u < PWMstartApos ) ) {u = PWMstartApos;}
    
    analogWrite(MotorA1, abs(u));
    analogWrite(MotorA2, 0);
    
  }
}
void runMotorB(int u) {

  // Simple Version:
  // // for negative u, run H-bridge in reverse; otherwise just pass command through
  // if (u < 0) {
  //   analogWrite(MotorB1, 0);
  //   analogWrite(MotorB2, abs(u));
  // } else {
  //   analogWrite(MotorB1, u);
  //   analogWrite(MotorB2, 0);
  // }

  // More complex:  don't allow |u| to exceed MaxPWM, if near zero, map it to a PWM val that at least starts the motor running
  // to help overcome stiction.  TODO: modulate this with speed, if moving, don't need to do this
  const int PWMstartBneg = -30, PWMstartBpos = 40;
  // for negative commands, clip at max and min
  if (u < 0) {
    if (u < -MaxPWM) {
      Serial.println("% Warning: Clipping motor output to " + String(-MaxPWM));
      u = MaxPWM;
    } else if (PWMstartBneg < u) u = -PWMstartBneg;
    else {
      analogWrite(MotorB1, 0);
      analogWrite(MotorB2, abs(u));
    }
  } else {  // 0 < u  (positive), do the same as above but swapped
    if (MaxPWM < u) {
      Serial.println("% Warning: Clipping motor output to " + String(MaxPWM));
      u = MaxPWM;
    } else if (u < PWMstartBpos) u = PWMstartBpos;
    else {
      analogWrite(MotorB1, abs(u));
      analogWrite(MotorB2, 0);
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//   SETUP  Code in this function only runs once; initialize stuff, default values
///////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  // put your setup code here, to run once:
  pinMode(LedPin, OUTPUT);
  // pinMode(MotorEnable, INPUT);
  pinMode(MotorA1, OUTPUT);
  analogWriteFrequency(MotorA1, PwmFrequency);
  pinMode(MotorA2, OUTPUT);
  analogWriteFrequency(MotorA2, PwmFrequency);
  pinMode(MotorB1, OUTPUT);
  analogWriteFrequency(MotorB1, PwmFrequency);
  pinMode(MotorB2, OUTPUT);
  analogWriteFrequency(MotorA2, PwmFrequency);

  // turn motors off, etc.
  digitalWrite(MotorA1, LOW);
  digitalWrite(MotorA2, LOW);
  digitalWrite(MotorB1, LOW);
  digitalWrite(MotorB2, LOW);
  // digitalWrite(LedPin, digitalRead(MotorEnable));  // LED on means motors enabled
  analogWriteResolution(PwmResolution);            // analogWrite value 0 to 4095, or 4096 for high

  // load cell setup ...
  loadCellA.begin(LoadCellA_DT, LoadCellA_SCK);
  loadCellB.begin(LoadCellB_DT, LoadCellB_SCK);
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
  myservo.attach(38);
  myservo.write(90);
}



///////////////////////////////////////////////////////////////////////////////////////////////////////
//   LOOP   This function will repeat until power down;  Put your main code here
///////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  // LED to indicate H Bridge status; led on -> enabled
  // digitalWrite(LedPin, digitalRead(MotorEnable));
  t0 = millis();

  // // Print all values for serial plotter:
  // // Print all sensor values to serial
  forceA =  loadCellA.get_units();
  posAprev = posA;
  posBprev = posB;
  posA = getServoPosA();
  posB = getServoPosB();
  posDeltaA = posA - posAprev;
  posDeltaB = posB - posBprev;


  // Serial.print("ForceA:");
  // Serial.print(loadCellA.get_units());   Serial.print(", ");

  // //Serial.print("EncA:");
  // //Serial.print(encoderA.read() * EncCountsToDeg);  Serial.print(",");

  // Serial.print("PotA:");
  // Serial.print(getServoPosA());  Serial.print(", ");

  
  // Serial.print("| ForceB:");
  // Serial.print(loadCellB.get_units());   Serial.print(", ");

  // // Serial.print("EncB:");
  // // Serial.print(encoderB.read() * EncCountsToDeg);  Serial.print(", ");

  // Serial.print("PotB:");
  // Serial.print(getServoPosA());  Serial.print(",");

  // Serial.print("cmdPWM:");
  // Serial.print(cmdPWM); Serial.print(",");
  // Serial.print(" of:"+String(MaxPWM)); Serial.print(", ");
  // // Serial.println("");


  // Print all sensor values to serial
  Serial.print(forceA);
  Serial.print("  ");
  Serial.print( analogRead( A18 )); delay(1);
  Serial.print("     ");   
  //Serial.print(loadCellB.get_units());
  //Serial.print("    ");
  // Serial.print(encoderA.read() * EncCountsToDeg);
  // Serial.print("  ");
  // Serial.print(encoderB.read() * EncCountsToDeg);
  // Serial.print("    ");
  Serial.print( posA );
  Serial.print("  ");
  Serial.print( posB );
  Serial.print("  ");
  Serial.print( posDeltaA ); //getServoPosB());
  
  Serial.print("    ");
  Serial.print(cmdPWM);
  Serial.print(" of ");
  Serial.print(MaxPWM);
  Serial.print("    ");  
  Serial.print("// loadcellA B   potA B Adelta   cmdPWM of MaxPWM   ");

  //drive motors forwards and backwards by small PWM ramp (20% of MaxPWM = 0.2*MaxPWM )
  // float t = millis()/1000.0;
  // float f = 1;
  // cmdPWM = MaxPWM * sin(2*PI*f * t);
  //cmdPWM = MaxPWM * (analogRead(PotB)*2 -1023)/1023.0;
  // runMotorA( cmdPWM );
  //runMotorB( cmdPWM );

  //int v = MaxPWM * 0.2f;
  //runMotorA( cmd );  runMotorB( cmd );
  // if (cmd >  v) delta = -1;
  // if (cmd < -v) delta = 1;
  // cmd += delta;

  //delay(1);

  // Force regulator; target force 0g
  //cmdPWM = -forceA * 20;  
  // e_1 = e;
  // e = -forceA;
  // ed = e - e_1;
  // cmdPWM = e*100 + (ed)*0;
  //  Serial.print("   e:"); Serial.print (e);Serial.print("   e1:"); Serial.print (e_1); Serial.print("   ed:"); Serial.print (ed); 
  
  // Force control; 100g
  // cmdPWM = (loadCellA.get_units()-100) * 2;

  // Position control 100 deg
  e_1 = e;
  e = posB-posA;
  ed = e - e_1;
  cmdPWM = e*500 + (ed)*200 ;
  Serial.print("   e:"); Serial.print (e);Serial.print("   e1:"); Serial.print (e_1); Serial.print("   ed:"); Serial.print (ed); 
  runMotorA(cmdPWM);
  myservo.write( 180 );//map(cmdPWM, -MaxPWM, MaxPWM, 1000, 2000) );


  // // Force control; 100g
  // cmdPWM = -(100-loadCellB.get_units()) * 2;

  // // Position control 100 deg
  // // cmdPWM = (1-encoderA.read() * EncCountsToDeg) * 30;

  // runMotorB(cmdPWM);


  // teleoperation
  // int cmdA, cmdB, errA, errB;
  // cmdA = -loadCellB.get_units() / 100 * MaxPWM;
  // errB = (getServoPosA() + encoderB.read() * EncCountsToDeg);
  // cmdB = -40 * errB;
  // runMotorA(cmdA);
  // runMotorB(cmdB);
  // cmdPWM = cmdA;
  // Serial.print(" errB: ");
  // Serial.print(errB);
  Serial.print(" ,");
  Serial.print(" deltaT[MS]: ");
  Serial.print(millis() - t0);
  Serial.println("");
}
