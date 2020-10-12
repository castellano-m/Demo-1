/* Name: Madison Heeg and Andrew Rouze
 * Date: October 12, 2020
 *  
 * Title: Demo 1 - Localization  
 *  - Motor1:   PMW1 = pin 9; SIGN1 = pin 7
 *              Channel A (yellow) = pin 2 (ISR)
 *              Channel B (white) = pin 13
 *              Vcc (blue) = 5V on Arduino
 *              
 *  - Motor 2:  PMW2 = pin 10; SIGN2 = pin 8 
 *              Channel A (color) = pin 3(ISR) 
 *              Channel B (color) = pin 13
 *              Vcc (color) = 5 V on Arduino? 
 *   
 * Functionality
 *  - ???
 */

#include <Encoder.h>  // Downloaded from https://www.pjrc.com/teensy/td_libs_Encoder.html 

/**************************************** PIN DEFINITIONS ****************************************/ 
/* RIGHT MOTOR */
#define R_PWM           9            // PWM 
#define R_SIGN          7            // direction
#define A_RW            2            // channel A, ISR (yellow)  
#define B_RW            12           // channel B (white)

/* LEFT MOTOR */
#define L_PWM           10           // PWM
#define L_SIGN          8            // direction
#define A_LW            3            // channel A, ISR (yellow)
#define B_LW            13           // channel B (white)

/**************************************** GLOBAL CONSTANTS ***************************************/ 
#define batteryVoltage  8            // [V]      voltage available from battery
#define N               3200         // [counts] # of turns per one revolution, Pololu motor datasheet
#define micro           1000000      // [s]      conversion from micros to seconds  

/************************************* VARIABLE DECLARATIONS *************************************/ 
const double  fullRotation = 6.22;    // [rad]  approx angular position after full rotation
const int     sampleTime = 50;        // [ms]   sampling time 

/* RIGHT MOTOR */
long          R_countPrev = -999;       // [counts]   prev encoder counts
long          R_countNow = 0;           // [counts]   current encoder counts
double        R_angPosPrev = 0.0;       // [rad]      prev angular position
double        R_angPosNow = 0.0;        // [rad]      current angular position
double        R_angVelNow = 0.0;        // [rad/s]    angular velocity
double        R_angVelPrev = 0.0;       // [rad/s]    angular velocity
double        R_linVel = 0.0;           // [m/s]      linear velocity
unsigned long R_timePrev = 0;           // [s]        prev time
unsigned long R_timeNow = 0;            // [s]        current time
unsigned long R_deltaT = 0;             // [s]        difference of current - prev time

bool          R_dir = 1;                // controls the direction of rotation
double        R_motorVotlage = 0.0;     // [V]  control voltage from PID

/* LEFT MOTOR^ */
long          L_countPrev = -999;       // [counts]   prev encoder counts
long          L_countNow = 0;           // [counts]   current encoder counts
double        L_angPosPrev = 0.0;       // [rad]      prev angular position
double        L_angPosNow = 0.0;        // [rad]      current angular position
double        L_angVelNow = 0.0;        // [rad/s]    angular velocity
double        L_angVelPrev = 0.0;       // [rad/s]    angular velocity
double        L_linVel = 0.0;           // [m/s]      linear velocity
unsigned long L_timePrev = 0;           // [s]        prev time
unsigned long L_timeNow = 0;            // [s]        current time
unsigned long L_deltaT = 0;             // [s]        difference of current - prev time

bool          L_dir = 1;                // controls the direction of rotation
double        L_motorVoltage = 0.0;     // [V]  control voltage from PID

/* ROBOT */
/* phi = angle w/ respect to x-axis */
const double  tooQuick = 0.05;                                            // [ms]   range if enter ISR too quickly
bool          enterISR = false;                                           // flag to detect if motor variables changed
double        x_prev = 0.0; double y_prev = 0.0; double phi_prev = 0.0;   // [?]    prev positions
double        x_now = 0.0; double y_now = 0.0; double phi_now = 0.0;      // [?]    current positions
unsigned long timePrev = 0;                                               // [s]    time exit main loop
unsigned long timeNow = 0;                                                // [s]    time enter main loop
unsigned long deltaT = 0;                                                 // [s]    time diff b/t exit and enter   

/* CONTROLS */
/* put control, gain values, etc. here */

/************************************* VARIABLE TO MANIPULATE *************************************/ 
const double  radius = 1.0;               // [m]  radius of wheels
const double  baseline = 0.1;             // [m]  width of robot

/************************************* DESIRED TASK VARIABLES ************************************/
 

/********************************************* SETUP *********************************************/ 
Encoder rightWheel(A_RW, B_RW); Encoder leftWheel(A_LW, B_LW);  // init encoder library for wheels

void setup() {

  pinMode(R_PWM, OUTPUT); pinMode(L_PWM, OUTPUT);                 // setting PWM as output
  pinMode(R_SIGN, OUTPUT); pinMode(L_SIGN, OUTPUT);               // setting signs as output

  attachInterrupt(digitalPinToInterrupt(A_RW), RW_ISR, CHANGE);   // ISR monitor changes to right wheel
  attachInterrupt(digitalPinToInterrupt(A_LW), LW_ISR, CHANGE);   // ISR monitor changes to left wheel
  
  Serial.begin(115200);                                           // initialize serial monitor
  
}

void loop() {

  /* if characteristics of wheel changed, update x, y, phi */
  if(enterISR) {
      timeNow = micros()/(unsigned long)micro;        // capture time enter loop
      deltaT = timeNow - timePrev;                    // calculate time since last enter

      /* calculate position and angle */
      x_now = x_prev + ((double)deltaT*double(cos(phi_prev))*(R_linVel + L_linVel))/(2.0);
      y_now = y_prev + ((double)deltaT*double(sin(phi_prev))*(R_linVel + L_linVel))/(2.0);
      phi_now = phi_prev + (double)deltaT*(radius/baseline)*(R_linVel-L_linVel);

      /* Print statements */
      /* -- here -- */
      
      /* set previous values to current values */
      x_prev = x_now;
      y_prev = y_now;
      phi_prev = phi_now;
      enterISR = false;

      timePrev = micros()/(double)micro;
  }

}

/* changes to the right wheel */
void RW_ISR() {
  enterISR = true;                                // set flag to monitor enter ISR true
  
  R_timeNow = micros()/(unsigned long)micro;      // capture the time enter ISR
  R_deltaT = R_timeNow - R_timePrev;              // calculate time since last ISR

  R_countNow = rightWheel.read();                 // read in the current encoder counts
  if (R_countNow != R_countPrev) {                // if there is a change to the encoder counts
    
      R_angPosNow = (2.0*PI*(double)R_countNow)/(double)N;                // calculate angular position of wheel
  
      if (abs(R_angPosNow) > fullRotation) {                              // if the wheel has gone a full rotation
          rightWheel.write(0); R_countNow = 0; R_angPosNow = 0.0;             // reset characteristics to zero
          R_angVelNow = R_angVelPrev;                                         // current angVel set to previous angVel
      } else {
          R_angVelNow = (R_angPosNow - R_angPosPrev)/((double)R_deltaT);  // else, calculate angVel based on angPos
      }
        
      R_linVel = radius*R_angVelNow;              // calculate linear velocity

      /* set prev values to current values */
      R_countPrev = R_countNow;              
      R_angPosPrev = R_angPosNow;             
      R_angVelPrev = R_angVelNow; 
      R_timePrev = micros()/(unsigned long)micro; // capture time leaving ISR
  }
}

/* changes to the left wheel */
void LW_ISR() {
  enterISR = true;                                // set flag to monitor enter ISR true
  
  L_timeNow = micros()/(unsigned long)micro;      // capture the time enter ISR
  L_deltaT = L_timeNow - L_timePrev;              // calculate time since last ISR

  L_countNow = leftWheel.read();                 // read in the current encoder counts
  if (L_countNow != L_countPrev) {                // if there is a change to the encoder counts
    
      L_angPosNow = (2.0*PI*(double)L_countNow)/(double)N;                // calculate angular position of wheel
  
      if (abs(L_angPosNow) > fullRotation) {                              // if the wheel has gone a full rotation
          leftWheel.write(0); L_countNow = 0; L_angPosNow = 0.0;             // reset characteristics to zero
          L_angVelNow = L_angVelPrev;                                         // current angVel set to previous angVel
      } else {
          L_angVelNow = (L_angPosNow - L_angPosPrev)/((double)L_deltaT);  // else, calculate angVel based on angPos
      }
        
      L_linVel = radius*L_angVelNow;              // calculate linear velocity

      /* set prev values to current values */
      L_countPrev = L_countNow;              
      L_angPosPrev = L_angPosNow;             
      L_angVelPrev = L_angVelNow; 
      L_timePrev = micros()/(unsigned long)micro; // capture time leaving ISR
  }

}
