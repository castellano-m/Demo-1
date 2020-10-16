/* Name: Madison Heeg and Andrew Rouze
 * Date: October 12, 2020
 *  
 * Title: Demo 1 - Localization  
 *  - Motor RW:   R_PWM = pin 9 (black); SIGN1 = pin 7 (red) *jumper pins
 *                Channel A (yellow) = pin 2 (ISR)
 *                Channel B (white) = pin 12
 *                Vcc (blue) = 5V on Arduino
 *              
 *  - Motor LW:   L_PWM = pin 10 (black); SIGN2 = pin 8 (red)
 *                Channel A (yellow) = pin 3(ISR) 
 *                Channel B (white) = pin 13
 *                Vcc (Blue) = pin 4 (set high)    
 *                
 * Functionality
 *  - calcualte angular velocities of each wheel
 */

/**************************************** PIN DEFINITIONS ****************************************/ 
/* RIGHT MOTOR */
#define R_PWM           9            // PWM (black)
#define R_SIGN          7            // direction (red)
#define A_RW            2            // channel A, ISR (yellow)  
#define B_RW            6            // channel B (white)

/* LEFT MOTOR */
#define L_PWM           10           // PWM (black)
#define L_SIGN          8            // direction (red)
#define A_LW            3            // channel A, ISR (orange)
#define B_LW            11           // channel B (white)
#define Vcc2            5            // secondary 5V supply for encoder

/* MOTOR SHIELD */
#define Enable          4            // HIGH - motor to receive current
#define STATUS          12           // Uknown pin on motor shield, DO NOT USE

/**************************************** GLOBAL CONSTANTS ***************************************/ 
#define batteryVoltage  8            // [V]      voltage available from battery
#define N               3200         // [counts] # of turns per one revolution, Pololu motor datasheet
#define micro           1000000      // [s]      conversion from micros to seconds  

/************************************* VARIABLE DECLARATIONS *************************************/ 
const double  fullRotation = 6.22;      // [rad]  approx angular position after full rotation
const int     sampleTime = 50;          // [ms]   sampling time 

/* RIGHT MOTOR */
bool          updateRW = false;         // flag to detect if right wheel variables changed
static int    R_AChannelNow;            // [1 or 0]   channel A of RW encoder 
static int    R_BChannelNow;            // [1 or 0]   channel B or RW encoder
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
bool          updateLW = false;         // flag to detect if left wheel variables changed
static int    L_AChannelNow;            // [1 or 0]   channel A of RW encoder 
static int    L_BChannelNow;            // [1 or 0]   channel B or RW encoder
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
const unsigned long  tooQuick =0.1;                                       // [ms]   50ms range if enter ISR too quickly
bool          enterISR = true;                                            // flag to detect if motor variables changed
double        x_prev = 0.0; double y_prev = 0.0; double phi_prev = 0.0;   // [m]    prev positions
double        x_now = 0.0; double y_now = 0.0; double phi_now = 0.0;      // [m]    current positions
unsigned long timePrev = 0;                                               // [s]    time exit main loop
unsigned long timeNow = 0;                                                // [s]    time enter main loop
unsigned long deltaT = 0;                                                 // [s]    time diff b/t exit and enter   

/* CONTROLS */
/* put control, gain values, etc. here */
double R_error      = 0;                  // [counts]
double R_errorInteg = 0;                  // [rad*s]
double R_kp         = 1;                  // [V/rad]
double R_ki         = 1;                  // [V/rad*s]

double L_error      = 0;                  // [counts]
double L_errorInteg = 0;                  // [rad*s]
double L_kp         = 1;                  // [V/rad]
double L_ki         = 1;                  // [V/rad*s]

/************************************* VARIABLE TO MANIPULATE *************************************/ 
const double  radius = 0.075;               // [m]  radius of wheels
const double  baseline = .245;              // [m]  width of robot

/************************************* DESIRED TASK VARIABLES ************************************/
const double  desireXPos = 0;               // [m]  final x position
const double  desireYPos = 0;               // [m]  final y position
double        R_desireAngPos = 0;           // [rad]  goal angular position for right wheel
double        L_desireAngPos = 0;           // [rad]  goal angular position for left wheel
int           R_fullRots = 0;               // [count]  number of right wheel full rotations
int           L_fullRots = 0;               // [count]  number of left wheel full rotations
double        pathLength = 0;               // [m]  length of direct path to (desireXPos, desireYPos)

/************************************** FUNCTION PROTOTYPES **************************************/ 
void calculatePosition();             // update x, y, phi of robot
void updateRightWheel();              // update angPos, angVel, and linVel of right wheel
void updateLeftWheel();               // update angPos, angVel, and linVel of left wheel

/********************************************* SETUP *********************************************/
void setup() {
  pinMode(Enable, OUTPUT); digitalWrite(Enable, HIGH);            // ensure motors get current
  pinMode(Vcc2, OUTPUT); digitalWrite(Vcc2, HIGH);                // secondary Vcc = 5V 
  pinMode(R_PWM, OUTPUT); pinMode(L_PWM, OUTPUT);                 // setting PWM as output
  pinMode(R_SIGN, OUTPUT); pinMode(L_SIGN, OUTPUT);               // setting signs as output
  
  pinMode(A_RW, INPUT_PULLUP); pinMode(B_RW, INPUT_PULLUP);       // setting channel A and B of right wheel to be pullup resistors
  pinMode(A_LW, INPUT_PULLUP); pinMode(B_LW, INPUT_PULLUP);       // setting channel A and B of left wheel to be pull up resistors

  attachInterrupt(digitalPinToInterrupt(A_RW), updateRW_countsISR, CHANGE);   // ISR monitor changes to right wheel
  attachInterrupt(digitalPinToInterrupt(A_LW), updateLW_countsISR, CHANGE);   // ISR monitor changes to left wheel
  
  Serial.begin(115200);                                           // initialize serial monitor

  pathLength = sqrt(pow(desireXPos,2) + pow(desireYPos,2));             // calculate path length
  R_fullRots = (int)(pathLength/(2*PI*radius));                         // calculate the total number of rotations required by right wheel
  // typecasting to int will make this round up, not sure how to make it always round down
  L_fullRots = (int)(pathLength/(2*PI*radius));                         // calculate the total number of rotations required by left wheel
  R_desireAngPos = ((pathLength/(2*PI*radius)) - R_fullRots)/radius;    // calculate right wheel final angular position
  L_desireAngPos = ((pathLength/(2*PI*radius)) - L_fullRots)/radius;    // calculate left wheel final angular position
  
}

void loop() {
  
  if(enterISR) {          /* update x, y, phi of robot */
    calculatePosition(); 
  }
  
  if(updateRW) {          /* update angPos, angVel, and linVel of right wheel */
    //Serial.print("R count: "); Serial.println(R_countNow);
    updateRightWheel();
  }
  
  if(updateLW) {          /* update angPos, angVel, and linVel of left wheel */
    updateLeftWheel();
    //Serial.print("L count: "); Serial.println(L_countNow);
  }

  // Calculate analogWrite inputs for Left and Write motors using PI control
   

}

/* update right encoder count values */
void updateRW_countsISR() {
    enterISR = true;                      // set flag to indicate enter ISR 
    updateRW = true;                      // set flag to indicate RW characteritics to be updated
    R_countPrev = R_countNow;             // set previous count value to  the current count value that has not been updated 

    R_AChannelNow = digitalRead(A_RW);    // read in Channel A 
    R_BChannelNow = digitalRead(B_RW);    // read in Channel B

    if (R_AChannelNow == R_BChannelNow) R_countNow +=2;
    else R_countNow -=2;
}

/* update left encoder count values */
void updateLW_countsISR() {
    enterISR = true;                      // set flag to indicate enter ISR 
    updateLW = true;                      // set flag to indicate LW characteritics to be updated
    L_countPrev = L_countNow;             // set previous count value to  the current count value that has not been updated 

    L_AChannelNow = digitalRead(A_LW);    // read in Channel A 
    L_BChannelNow = digitalRead(B_LW);    // read in Channel B

    if (L_AChannelNow == L_BChannelNow) L_countNow -=2;
    else L_countNow +=2;    

}

/* update right wheel velocities and positions */
void updateRightWheel() {
    R_timeNow = micros()/(unsigned long)micro;      // capture the time enter ISR
    R_deltaT = R_timeNow - R_timePrev;              // calculate time since last ISR

    if (R_deltaT > tooQuick){
          if(R_countNow != R_countPrev) {
                R_angPosNow = (2.0*PI*(double)R_countNow)/(double)N;
                
                if (abs(R_countNow) > N) {                              // if the wheel has gone a full rotation
                    R_countNow = 0; R_angPosNow = 0.0;                      // reset characteristics to zero
                    R_angVelNow = R_angVelPrev;                             // current angVel set to previous angVel
                } else {
                  R_angVelNow = (R_angPosNow - R_angPosPrev)/((double)R_deltaT);  // else, calculate angVel based on angPos
                }
                
                R_linVel = radius*R_angVelNow;              // calculate linear velocity

                updateRW = false;
                R_angPosPrev = R_angPosNow;             
                R_angVelPrev = R_angVelNow; 
                R_timePrev = micros()/(unsigned long)micro; // capture time leaving ISR
          }
    }
}


/* update left wheel velocities and positions */
void updateLeftWheel() {
    L_timeNow = micros()/(unsigned long)micro;      // capture the time enter ISR
    L_deltaT = L_timeNow - L_timePrev;              // calculate time since last ISR

    if (L_deltaT > tooQuick){
          if(L_countNow != L_countPrev) {
                L_angPosNow = (2.0*PI*(double)L_countNow)/(double)N;
                
                if (abs(L_countNow) > N) {                             // if the wheel has gone a full rotation
                    L_countNow = 0; L_angPosNow = 0.0;                       // reset characteristics to zero
                    L_angVelNow = L_angVelPrev;                             // current angVel set to previous angVel
                } else {
                  L_angVelNow = (L_angPosNow - L_angPosPrev)/((double)L_deltaT);  // else, calculate angVel based on angPos
                }
                
                L_linVel = radius*L_angVelNow;              // calculate linear velocity

                updateLW = false;
                L_angPosPrev = L_angPosNow;             
                L_angVelPrev = L_angVelNow; 
                L_timePrev = micros()/(unsigned long)micro; // capture time leaving ISR
          }
    }
}

/* Calculate new position and angle of robot*/
void calculatePosition() {
  timeNow = micros()/(unsigned long)micro;        // capture time enter loop
  deltaT = timeNow - timePrev;                    // calculate time since last enter
  
  /* calculate position and angle */
  x_now = x_prev + ((double)deltaT*double(cos(phi_prev))*(R_linVel + L_linVel))/(2.0);
  y_now = y_prev + ((double)deltaT*double(sin(phi_prev))*(R_linVel + L_linVel))/(2.0);
  phi_now = phi_prev + (double)deltaT*(radius/baseline)*(R_linVel-L_linVel);

  /* Print statements */
  Serial.print(x_now); Serial.print("\t"); Serial.print(y_now); Serial.print("\t"); Serial.println(phi_now); 
           
  /* set previous values to current values */
  x_prev = x_now;
  y_prev = y_now;
  phi_prev = phi_now;
  enterISR = false;

  timePrev = micros()/(double)micro;   
}
