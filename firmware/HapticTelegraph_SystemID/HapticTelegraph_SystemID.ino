// This reads encoder values using a hardware timer interrupt and ring buffer
// A timer interrupt (timer1 16-bit timer) is used
// dumps data to serial port.  Test signals can be sent to motors; 
// PWM command, timestamps, and all sensors are dumped to serial for plotting/analysis

#include <ADC.h>    // Analog to Digital Conversion library (avoids blocking time of native analogRead())
#include <HX711.h>  // https://github.com/bogde/HX711 bodge's library

//  ****  ENCODER SECTION    *****      //
#define ENC_TICKS_PER_REV 12.0f  //8192 CUI; N20 motors have 12 CPR HALL encoder      // encoder ticks per revolution
#include <Encoder.h>
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   Avoid using pins with LEDs attached
//Encoder myEnc(10, 11);                                              // must be 2,3 on arduino UNO, any digital pin on Teensy
// const float ticksToDegrees = 360.0 / (ENC_TICKS_PER_REV * 298.0f);    // 298:1 gear ratio
const float ticksToDegrees = 360.0 / (ENC_TICKS_PER_REV * 100.0f);    // 100:1 gear ratio


///////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES  -- will be used by interrupts
volatile int motorPWMcmdA = 0;  // The actual low-level signed pwm command used by runMotorA
volatile int motorPWMcmdB = 0;  // e.g. motorPWMcmdA = desiredPWM + ditherPWN + deadBand ...
volatile int potA, potB;        // global variable for pot position reads (raw values)
volatile int encA, encB;        // raw values 
volatile int forceA, forceB;    // converted force values in grams-force.
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
inline float getServoPosA() {
  potA = analogRead(PotApin);
  return ((float)potA - PotMinA) * Pot2DegA - 90.0;  // make origin 0 so +/-90deg travel
}
inline float getServoPosB() {
  potB = analogRead(PotBpin);
  return ((float)potB - PotMinB) * Pot2DegB - 90.0;
}


// Motor items; PWM resolution and frequency; see https://www.pjrc.com/teensy/td_pulse.html
const int PwmResolution = 12;
const int MaxPWM = pow(2, PwmResolution) - 1;  // e.g. 8bit resolution has 255 max
const int PwmFrequency = 00;                  // Hz
const int PwmDitherMax = MaxPWM*20/100.0f ;         // e.g. dither with 20% of MaxPWM

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
  //motorPWMcmdB = u;
  // for negative u, run H-bridge in reverse; otherwise just pass command through
  if ( u < 0) {
    analogWrite(MotorB1pin,  0            );
    analogWrite(MotorB2pin, -u );
  } else {
    analogWrite(MotorB1pin,  u );
    analogWrite(MotorB2pin,  0            );
  }
}



// ****  HARDWARE TIMER SECTION  ****    //
// Use Timer1 (16-bit timer that does not mess with millis(), micros() or arduino scheduler which uses
// timer0 (8bit)
// https://www.pjrc.com/teensy/td_libs_TimerOne.html
#include <TimerOne.h>
#define LED_PIN 13
// Sample period of timer interupt service routine (ISR) in microseconds
//#define   TIMER_PERIOD 100000 // e.g. set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz)
#define TIMER_PERIOD 1000  // e.g. 1000us = 1ms =>  1 KHz update rate.


// ****  SERVO SECTION       **** ////
#include <Servo.h>
#define SERVO_PIN 23
Servo myservo;  // unmodified servo 

// // ****** MOTOR PWM / H-Bridge Section   *** //
// #define MOT_A 18
// #define MOT_B 6
// volatile int u;  // motor command global variable.
// void motorCommand(int uPWM) {

//   if (u < 0) {
//     analogWrite(MOT_A, -uPWM);
//     analogWrite(MOT_B, 0);
//   } else {
//     analogWrite(MOT_A, 0);
//     analogWrite(MOT_B, uPWM);
//   }
//   return;
// }

//   ****  RING BUFFER SECTION ****     ///
// A ring buffer.  Allows you to write data VERY quickly inside an interrupt.  Can read it later in chunks
// to write to ring buffer, write myDataSample to: iWrite = (currentIndex+1) % BUF_LENGTH location along BUF_LENGTH;
//                          THEN, WHEN DONE WRITING: set lastWritten = iWrite
// to read from ring bufffer, read myDataSample from: iRead = (currentIndex+1) % BUF_LENGTH location;
//                          THEN, WHEN DONE READING: set lastread = iRead
#define BUF_LENGTH 10000  // number of entries in ring buffer before overflow (positive integer, needs to be long enough to not overrun)
#define BUF_DIMENSION 1   // dimmension of buffer  (e.g. number of encoders and therefore dataSamples per time sample)
struct myDataSample {
  unsigned long int t;    // time of sample (micro sec)
  int   motorPWMcmdA;     // actual motor command (no dither or compensation) at this time sample
  int   motorPWMcmdB;     // ...
  float forceA;           // load cell force (with unit conversion) 
  float forceB;           // load cell force (with unit conversion)     
  int   encA;             // encoder reading at this time sample
  int   encB;             // ...
  int   potA, potB;       // potentiometers
  bool  bufferOverflow;   // 0 - all OK;  true - buffer overflowed and data corrupted.
};

// Instantiate the ringBuffer itself.
myDataSample ringBuffer[BUF_LENGTH][BUF_DIMENSION];
bool ringBufferOverflow;
volatile int lastWritten;  // using signed ints so that ( lastRead - 1 ) will behave nicely near zero, makes modulo arithmetic more intuitive.
volatile int lastRead;
volatile int iWrite, iRead;  // index values for bookeeping, a separate i for writing and reading.


/// --------------------------
/// Custom ISR Timer Routine  (ISR: Interrupt Service Routine, the function that gets called each time TIMER_PERIOD expires.
/// --------------------------
volatile unsigned long int tg;  // global time [micros]
volatile bool ditherToggle = 1; 
void inline timerIsr() {

  // Toggle LED
  //digitalWrite( LED_PIN, digitalRead( LED_PIN ) ^ 1 );         // can attach an o-scope to measure jitter on this pin directly

  // read sensors to global vars and get time
  volatile static unsigned long int t0 = micros();  // static vars only get initialized the first time function is called, persiste thereafter
  tg = micros() - t0;
  encA = encoderA.read();       encB = encoderB.read();       // should be very fast
  potA = analogRead( PotApin ); potB = analogRead( PotBpin ); // TO DO: use ADC library to make this fast. 
  // forceA =  loadCellA.get_units();  !! Cannot do this in ISR, long duration, blocking call. 


  // Get most recent data, write it to ring buffer.
  // to write to ring buffer, write myDataSample to: iWrite = (currentIndex+1) % BUF_LENGTH location along BUF_LENGTH;
  //                          THEN, WHEN DONE WRITING: set lastWritten = iWrite
  iWrite = (BUF_LENGTH + lastWritten + 1) % BUF_LENGTH;  // shift index to write to next entry

  if (iWrite == lastRead)       // This is bad; working code needs to ensure it never happens,
    ringBufferOverflow = true;  // if it does, data is overwritten; increase buffer length or reading frequency to avoid

  ringBuffer[iWrite][0].t = tg;                               // current time in microsec
  ringBuffer[iWrite][0].motorPWMcmdA = motorPWMcmdA;          // current motor commands (plant input u, no dither or open loop comp.)
  ringBuffer[iWrite][0].motorPWMcmdB = motorPWMcmdB;          // current motor commands (plant input u, no dither ...)
  ringBuffer[iWrite][0].forceA =  forceA;                     // get global forceA, won't update as fast and timestamp will stray
  ringBuffer[iWrite][0].forceB =  forceB;                     // get global forceA, won't update as fast and timestamp will stray
  ringBuffer[iWrite][0].encA = encoderA.read();               // current encoder position
  ringBuffer[iWrite][0].encB = encoderB.read();               // current encoder position
  ringBuffer[iWrite][0].potA = potA;                          // current pot val 
  ringBuffer[iWrite][0].potB = potB;                          // current pot val 
  ringBuffer[iWrite][0].bufferOverflow = ringBufferOverflow;  // tag if/when the buffer overflows
  lastWritten = iWrite;                                       // officially done writing the "next entry"

  // dither
  if ( ditherToggle )
    motorPWMcmdB += PwmDitherMax;
  else 
    motorPWMcmdB -= PwmDitherMax; 
  runMotorB( motorPWMcmdB );
  ditherToggle = !ditherToggle;   
}





/////////////////////////////////////////////////////////////////////////
//  **** SETUP -- only runs once; Set up buffer and timer here   ****  //
/////////////////////////////////////////////////////////////////////////

#define BANNER "% t[ms]  command[pwmA]  encPosA[ticks]  potA[raw]  command[pwmB]  encPosB[ticks]  potB[raw]   bufferOverflow \t iRead iToRead i "
unsigned long int enc, enc_1, enc_2;  // the current, previous, and previous-previous encoder readings
unsigned int j = 0, increasingDelay;
long oldPosition = -999;

float dT, t;

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

  while (!Serial /*&& (millis() < 5000)*/) {}  // wait up to 5 seconds for Terminal
  Serial.begin(115200);
  Serial.println(BANNER);
  Serial.print("% PWM Freq: "); Serial.print(PwmFrequency) ;
  Serial.print(" TimerPeriod[us]: ") ;Serial.println(TIMER_PERIOD);
  Serial.flush();

  // while (!Serial.available()) { /* do nothing until a serial event is received like a key press */
  // }
  // Serial.readString(); // consume whatever line is on serial so loop will start with empty input buffer

  // Initialize Ring Buffer indexes so that next write and read will occur at 0 (both must be equal)
  lastRead = BUF_LENGTH - 1;
  lastWritten = BUF_LENGTH - 1;

  // Set the timer and attach it.  Once that happens buffer will start getting filled with entries each TIMER_PERIOD
  // That means you must periodically read (and therefore empty) the buffer fast enough so it does not overflow.
  Timer1.initialize(TIMER_PERIOD);   //
  Timer1.attachInterrupt(timerIsr);  // attach the service routine here
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  the LOOP()
//     * must empty the buffer fast enough to prevent overruns.
//     * empties the buffuer to serial, computing differences and derivatives as it does so
//     * To test when bufferoverrun occurs, artificially increases a delay() each loop.
//
const float F0 = 2 * PI * 0.2;     // Initial freq. in rad/s
const float F1 = 2 * PI * 2;       // final freq. in rad/s
const float T = 20 * 1;            // time in ms to run a test signal like chirp
const float C = log(F1 / F0) / T;  // compute this constant once for faster processing in chirpExp
// chirp =  sin( t .*  F0 .* exp( t.*C ) /1000 );


int squareWave(int uL, int uH, int TL, int TH) {
  static unsigned char state = 0;  // holds state of state machine
  int out = 0;
  if (state == 0) {
    out = uL;
  } else if (state == 1) {
    out = uH;
  } else {
    state = 0;
  }

  return out;
}

void loop() {

  // first take force readings.  This will be slow and block until done.  Then do timestamps, etc. 
  // onlt take both readings if you have to.  One or zero if your task does not need force(s)
  // forceA = loadCellA.get_units(); 
  forceB = loadCellB.get_units(); 
  
  static unsigned long int n = 0 /* num periods */, repsSoFar = 0; /* how many square wave period epochs of n have happened so far */
  //t = (micros() - t0) / 1000000.0;  // t in sec (now)
  //u = MaxPWM/2.5 * (1 - cos(t*F0) ) / 2.0;  // constant sine wave
  //u = 90 * ( 1 - cos(t*F0 * exp( t*C )) );  // chirp wave from F0 to F1
  //if( t > 5) return;
  //u =  int(t*100.0) % 300;//int(t) % PWM_MAX ;

  // SQUARE WAVE
  static int u, uL = MaxPWM * 100 / 100, uH = MaxPWM * 0 / 100;  // amplitude high and low of square wave
  static float T0 = 1000, T1 = 1000 + T0;                        //duration [ms]:  (end of) low and high  part of square wave
  static int state = 0;
  static unsigned long int tt0 = millis(), tt;  // a second timer just for wave generator
  tt = millis() - tt0;
  // Serial.print(" tt, T0 , tt>T0: ");
  // Serial.print(tt); Serial.print(" ");
  // Serial.print(T0); Serial.print(" ");
  // Serial.print(tt > T0); Serial.println(" ");
  if (state == 0) {
    u = uL;
    if (tt > T0) {
      state = 1;
    }
  } else if (state == 1) {
    u = uH;
    if (tt > T1) {
      state = 0;
      u = uL;          // smooth transition to avoid time back-jumps in plotting
      tt0 = millis();  // reset time for next run
      n++;             //Serial.print(" Done with State 1 ");
    }
  }  //else { state = 0; }

  u = 30; // can write a constant value here.
  motorPWMcmdB = 0;
  runMotorA(u);
  //runMotorB(  );

  //myservo.write( u );
  //myservo.writeMicroseconds(  1500 + u );

  // ////////////////////////////////////////////////////////////////////////////////////////////////////
  // RING BUFFER CODE
  // Empty entire ringBuffer to Serial...
  // to read from ring bufffer, read myDataSample from: iRead = (currentIndex + 1) % BUF_LENGTH location;
  //                          THEN, WHEN DONE READING: set lastread = iRead
  // First, if the buffer has overflowed, freak out.  This should never happen
  if (ringBufferOverflow)
    Serial.println("ERROR:  Ring buffer overflowed!  ... cannot trust data integrity going forward, increase buffer length?");

  // How many unread entries are there to read right now?
  int numToRead = (BUF_LENGTH + lastWritten - lastRead) % BUF_LENGTH;

  // read and process each entry one at a time.  incrementing lastRead as you do so...
  for (int i = 0; i < numToRead; i++) {

    iRead = (BUF_LENGTH + lastRead + 1) % BUF_LENGTH;  // start reading next entry (one entry at a time per each i iteration)

    // do math and unit conversion or other calculuations like floating point HERE, not in any Interrupt Service Routine (ISR) code.
    // ringBuffer[iRead][0].posEncA = ringBuffer[iRead][0].enc * ticksToDegrees;
    // ringBuffer[iRead][0].posEncB = ringBuffer[iRead][0].enc * ticksToDegrees;
    // ringBuffer[iRead][0].posPotA = getServoPosA();
    // ringBuffer[iRead][0].posPotB = getServoPosB();
    //ringBuffer[iRead][0].vel = ringBuffer[iRead][0].pos - ringBuffer[(BUF_LENGTH + iRead - 1) % BUF_LENGTH][0].pos;
    //ringBuffer[iRead][0].acc = ringBuffer[iRead][0].vel - ringBuffer[(BUF_LENGTH + iRead - 1) % BUF_LENGTH][0].vel;

    // compute deltaT, how much time elapsed since the previous sample, in seconds.  Should be same as SAMPLE_PERIOD/1000.0
    //dT = ringBuffer[(BUF_LENGTH + iRead ) % BUF_LENGTH][0].t     -     ringBuffer[(BUF_LENGTH + iRead - 1) % BUF_LENGTH][0].t ; // diff in us
    //dT = dT / 1000000.0 ;  // diff in sec.
    // scale .pos differences by this amount to return .vel and .acc in /sec.
    //ringBuffer[iRead][0].vel = ringBuffer[iRead][0].vel / dT ;
    //ringBuffer[iRead][0].acc = ringBuffer[iRead][0].acc / dT ;

    // output to serial as ASCII
    Serial.print(ringBuffer[iRead][0].t / 1000.0, 2);
    Serial.print("    ");   
    // Motor A ... 
    Serial.print(ringBuffer[iRead][0].motorPWMcmdA);
    Serial.print("  ");
    Serial.print(ringBuffer[iRead][0].potA);
    Serial.print("  ");
    Serial.print(ringBuffer[iRead][0].encA);
    Serial.print("    ");    
    Serial.print(ringBuffer[iRead][0].forceA);
    Serial.print("    ");    
    // Motor B ... 
    Serial.print(ringBuffer[iRead][0].motorPWMcmdB);
    Serial.print("  ");
    Serial.print(ringBuffer[iRead][0].potB);
    Serial.print("  ");
    Serial.print(ringBuffer[iRead][0].encB);
    Serial.print(" ");
    Serial.print(ringBuffer[iRead][0].forceB);
    
    // Serial.print(ringBuffer[iRead][0].pos, 2);
    //Serial.print( " " );
    //Serial.print( ringBuffer[iRead][0].vel , 2);
    //Serial.print( " " );
    //Serial.print( ringBuffer[iRead][0].acc , 2);
    //Serial.print( " \t % dT =  " ); Serial.print( dT  );
    Serial.print(" ");
    //Serial.print( iRead );
    Serial.print(" ");
    Serial.print(tt);
    Serial.println("");

    lastRead = iRead;  // officially done reading the iRead entry
    delayMicroseconds(1);
  }
  //Serial.flush();

  if (n > 0) {
    n = 0;
    u = 0;
    runMotorA(u);
    runMotorB(u);
    repsSoFar++;
    uL = uL * 90 / 100, uH = uH * 90 / 100;  // amplitude high and low of square wave; lower for next round by xx percent
    if (repsSoFar >= 35) {
      Serial.flush();
      while (!Serial.available()) { /* do nothing until a serial event is received like a key press */
      }
      Serial.readString();
      tt0 = millis();
    }
  }
}
