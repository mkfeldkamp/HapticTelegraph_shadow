// Modify this file for your specific pins, calibration values, hardware setup, and motor drive
// command functions.  
// #include this file above your haptic telegraph code.  You can use the same pinouts/config
// in different programs and only need to update pins/config in (this) one single file.  

///////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES  -- will be used by interrupts
volatile int motorPWMcmdA = 0;  // The actual low-level signed pwm command used by runMotorA
volatile int motorPWMcmdB = 0;  // e.g. motorPWMcmdA = desiredPWM + ditherPWN + deadBand ...
volatile int potA, potB;        // global variable for pot position reads
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Constants (in TitleCase;  variables are in camelCase)
// pins 7 & 8 reserver for TimerOne https://www.pjrc.com/teensy/td_libs_TimerOne.html
const char LedPin           = 13;  // 13 is LED on teensy; indicate power on
const char MotorEnablePin   = 12;  // (optional) H Bridge sleep pin; reads Enable Switch; HIGH-->ON
const char MotorA1pin       =  4;  // Motor command pins (feeds into input of DRV8833 H bridges)
const char MotorA2pin       =  5;  // swaping 1 and 2 will reverse direction of motor
const char MotorB1pin       = 18;  //
const char MotorB2pin       =  6;  // 
const char LoadCellA_SCKpin =  1;  // HX711 Load cell pins;  Check carefully; one writes, one reads
const char LoadCellA_DTpin  =  0;  // Different manufacturers may USE A DIFFERENT ORDER
const char LoadCellB_SCKpin =  2;  // SCK: Clock input to load cell (from teensy, which writes it)
const char LoadCellB_DTpin  =  3;  // DT: Data outpt from load cell (to teensy, which reads it)

// Load cell calibration values ...   need to calibrate to get true grams or kg
const float LoadCellA_scale = 420.1;  // TODO: you need to calibrate this yourself. 5kg ~ 420.0
const float LoadCellB_scale = 420.0;  // RawLoadCell/scale =[g/au]*[au] => [grams]; 1kg ~ XXX.X  
float loadCellA_offset = 528416.0;    //  set by tare s.t. readChannelRaw() - offset = 0.0g
float loadCellB_offset = 55000.0;
Adafruit_HX711 loadCellAdaA(LoadCellA_DTpin, LoadCellA_SCKpin);
Adafruit_HX711 loadCellAdaB(LoadCellB_DTpin, LoadCellB_SCKpin);

// Rreturn load in [grams];  call it when loadCellxx.isBusy() is false, otherwise
// it will block and take up to 1/80Hz sec.  
inline float getLoadA(){  
  return( -( loadCellAdaA.readChannelRaw(CHAN_A_GAIN_128) - loadCellA_offset )  /  LoadCellA_scale );
}
inline float getLoadB(){  
  return(  ( loadCellAdaB.readChannelRaw(CHAN_A_GAIN_128) - loadCellB_offset )  /  LoadCellB_scale );
}

// Motor feedback; N20 gear motors have digital encoders, modified servo's have analog potentiometers
// make sure encoder pins are interruptable (see teensy documentation)
const char EncApin_aw = 8;   // Motor A; enoder channel a, white wire
const char EncApin_by = 9;   // Motor A; enoder channel b, white wire
const char EncBpin_aw = 10;  // Motor B; enoder channel a, white wire
const char EncBpin_by = 11;  // Motor B; enoder channel b, white wire
// const float EncCountsToDeg = 360.0f / (12.0f * 100.0f);  // Encoder counts per revol. (CPR) x Gear Ratio;
const float EncCountsToDeg = 360.0f / (12.0f * 210.0f);  // Encoder counts per revol. (CPR) x Gear Ratio;
Encoder encoderA(EncApin_aw, EncApin_by);
Encoder encoderB(EncBpin_aw, EncBpin_by);


///////////////////////////////////////////////////////////////////////////////////////////////////////
// SERVO STUFF:  Servo can be used in two modes:
//
//  UNMODIFIED (like Servo.write( 180 )) as in Arduino-->examples-->Servo->sweep
//
//  MODIFIED:  you hack servo, splice into its potentiometer (pot) and maybe cut out it's drive 
//             circuitry all together and drive the servo's DC motor directly with the H bridge. 
//  ** you can comment out the code below ( especially in loop() )  for the mode you are NOT using **

// UNMODIFIED:   no pot or direct H-brdige drive  
const int ServoDrivePinA   = A1;   // same pin as anolog PotApin of modified  servo A1==15 (pin 15)
const int ServoDrivePinB   = A0;   //  "    "   "    "   PotBpin  "     "      "    A0==14 (pin 14)
const int ServoInitialPosA = 90;   // on startup, servo will be driven to this value (degrees)
const int ServoInitialPosB = 90;
Servo servoA, servoB;


// MODIFIED servo:  both potentiometer access via analogRead() and possibly driving with H bridge
//                  if driving with H brdige, same code as for N20's  below
// Potentiometer position feedback (modified servo)   TODO: you find these three vals for each servo
const char  PotApin   = A1;                             // Servo A potentiometer 
const char  PotBpin   = A0;                             // Servo B potentiometer
const int   PotMinA   = 43;                             // end stop of servo
const int   PotMaxA   = 976;                            // end stop of servo
const float RngDegA   = 195;                            // endstop-to-endstop travel in degrees (use protractor)
const float Pot2DegA  = RngDegA / (PotMaxA - PotMinA);  // degree per pot val
const int   PotMinB   = 44;                             // end stop of servo
const int   PotMaxB   = 976;                            // end stop of servo
const float RngDegB   = 195;                            // travel between end stops in degrees
const float Pot2DegB  = RngDegB / (PotMaxB - PotMinB);  // degree per pot val
// Each (modified) servo reads in unique pot values; compute servo-specific degrees here
float getModServoPosA() {
  potA = analogRead(PotApin);
  return ((float)potA - PotMinA) * Pot2DegA - 90.0;  // make origin 0 for +/-90deg travel
}
float getModServoPosB() {
  potB = analogRead(PotBpin);
  return ((float)potB - PotMinB) * Pot2DegB - 90.0;
}

// DC Motor items  (both N20 and modified servo with direct drive of servo's DC motor); 
// PWM resolution and frequency; see https://www.pjrc.com/teensy/td_pulse.html
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

