/* Name: Madison Heeg and Andrew Rouze
 * Date: October 12, 2020
 *  
 * Title: Demo 1 - Localization  
 *  - Motor RW:   Channel A (yellow) = pin 2 (ISR)
 *                Channel B (white) = pin 12
 *                Vcc (blue) = 5V on Arduino
 *              
 *  - Motor LW:   Channel A (yellow) = pin 3(ISR) 
 *                Channel B (white) = pin 13
 *                Vcc (Blue) = pin 4 (set high)    
 *                
 * Functionality
 *  - calcualte angular velocities of each wheel
 */

/**************************************** PIN DEFINITIONS ****************************************/ 
/* RIGHT MOTOR - Motor B */
#define R_PWM           10           // PWM (black)
#define R_SIGN          8            // direction (red)
#define A_RW            2            // channel A, ISR (yellow)  
#define B_RW            6            // channel B (white)

/* LEFT MOTOR - Motor A */
#define L_PWM           9           // PWM (black)
#define L_SIGN          7            // direction (red)
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
static int    R_AChannelNow;            // [1 or 0]   channel A of RW encoder 
static int    R_BChannelNow;            // [1 or 0]   channel B or RW encoder
long          R_countNow = 0;           // [counts]   current encoder counts
long          R_countPrev = 0; 
double        R_angPosPrev = 0.0;       // [rad]      prev angular position
double        R_angPosNow = 0.0;        // [rad]      current angular position
double        R_angVelNow = 0.0;        // [rad/s]    angular velocity
double        R_angVelPrev = 0.0;       // [rad/s]    angular velocity
double        R_linVel = 0.0;           // [in/s]     linear velocity
unsigned long R_timePrev = 0;           // [us]       prev time
unsigned long R_timeNow = 0;            // [us]       current time
unsigned long R_deltaT = 0;             // [us]       difference of current - prev time

/* LEFT MOTOR^ */
static int    L_AChannelNow;            // [1 or 0]   channel A of RW encoder 
static int    L_BChannelNow;            // [1 or 0]   channel B or RW encoder
long          L_countNow = 0;           // [counts]   current encoder counts
long          L_countPrev = 0;
double        L_angPosPrev = 0.0;       // [rad]      prev angular position
double        L_angPosNow = 0.0;        // [rad]      current angular position
double        L_angVelNow = 0.0;        // [rad/s]    angular velocity
double        L_angVelPrev = 0.0;       // [rad/s]    angular velocity
double        L_linVel = 0.0;           // [in/s]      linear velocity
unsigned long L_timePrev = 0;           // [us]       prev time
unsigned long L_timeNow = 0;            // [us]       current time
unsigned long L_deltaT = 0;             // [us]       difference of current - prev time


/* ROBOT */
/* phi = angle w/ respect to x-axis */
/*  R L 
 *  0 0 Forward
 *  0 1 CCW
 *  1 0 CW
 *  1 1 B
 *  
 *  CCW = negative rotational Velocity
 *  
*/
//const unsigned long  tooQuick = 1;                                          // [us]   us range if enter ISR too quickly
double        x_prev = 0.0; double y_prev = 0.0; double phi_prev = 0.0;       // [in]   starting positions
double        x_now = 0.0; double y_now = 0.0; double phi_now = 0.0;          // [in]   current positions
unsigned long timePrev = 0;                                                   // [s]    time exit main loop
unsigned long timeNow = 0;                                                    // [s]    time enter main loop
unsigned long deltaT = 0;                                                     // [s]    time diff b/t exit and enter

double        J_linVel = 0.0;                                                 // [in/s]     linear velocity                 
double        J_rotVel = 0.0;

//double x_posCalib = 4.16;                                                    // [in] calibration for front of camera to be 0 origin
//double y_posCalib = 4.16;                                                    // [in] calibration for front of camera to be 0 origin 

bool          R_dir = 0;                // controls the direction of rotation
bool          L_dir = 1;                // controls the direction of rotation
double        R_motorVotlage = 0.0;     // [V]  control voltage from PID
double        L_motorVoltage = 0.0;     // [V]  control voltage from PID

/* CONTROLS */
/* put control, gain values, etc. here */

/************************************* VARIABLE TO MANIPULATE *************************************/ 
const double  radius = 2.952;               // [in]  radius of wheels
const double  baseline = 10.827;            // [in]  width of robot

/************************************* DESIRED TASK VARIABLES ************************************/
double loopTime = 0;

/************************************** FUNCTION PROTOTYPES **************************************/ 
void calculatePositionandVel();       // update x, y, phi of robot
void updateRightWheel();              // update angPos, angVel, and linVel of right wheel
void updateLeftWheel();               // update angPos, angVel, and linVel of left wheel
void updateWheels();                  // update angPos, angVel, and linVel of both wheels

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

  //Serial.print("x [in]"); Serial.print("\t"); Serial.print("y [in]"); Serial.print("\t"); Serial.println("phi [in]");
  //Serial.print("L_AngPos [rad] "); Serial.print("\t"); Serial.print("L_AngPrev [rad]"); Serial.print("\t"); Serial.print("L_deltaT [us]"); Serial.print("\t"); Serial.print("R_AngPrev [rad]"); Serial.print("\t");Serial.print("R_angPos [rad]"); Serial.print("\t"); Serial.print("R_deltaT [us]"); Serial.print("\t"); 
  //Serial.print("time [ms]");Serial.print("\t"); Serial.print("L_angVel[rad/s]"); Serial.print("\t"); Serial.println("R_angVel [rad/s]");
  Serial.print("time [ms]");Serial.print("\t"); Serial.print("J_linVel[in/s]"); Serial.print("\t"); Serial.println("J_rotVel[rad/s]");  

  //int timeStep = millis();
  //while(millis() < (timeStep + 1000));                            // wait 1 second before applying voltage step
  digitalWrite(R_SIGN, R_dir);  digitalWrite(L_SIGN, L_dir);      // assign direction to motors
  analogWrite(R_PWM, 127);      analogWrite(L_PWM, 127);          // write 50% duty cycle to each motor
  
}

/********************************************* LOOP **********************************************/
void loop() {
   
  digitalWrite(R_SIGN, R_dir);  digitalWrite(L_SIGN, L_dir);      // assign direction to motors
  
  if(millis() > 2800){
    analogWrite(R_PWM, 0);                    // let motor run for 1 s
    analogWrite(L_PWM, 0);                    // let motor run for 1 s
  } else {
    analogWrite(R_PWM, 127);      analogWrite(L_PWM, 127);          // write 50% duty cycle to each motor
  }
       
    calculatePositionandVel();        /* update x, y, phi of robot */
    //Serial.print("RW: "); Serial.print(R_countNow); Serial.print("\t"); Serial.print("LW: "); Serial.print(L_countNow); Serial.print("\t");
    //Serial.print(x_now); Serial.print("\t"); Serial.print(y_now); Serial.print("\t"); Serial.println(phi_now); 

  if((millis()%50 == 0) && (millis() <= 2800)){   /* sample velocity of both left and right wheels every 50 ms while time <= 2.8 s */
    //Serial.print(L_angPosNow); Serial.print("\t"); Serial.print("\t"); Serial.print(L_angPosPrev); Serial.print("\t"); Serial.print("\t"); Serial.print(L_deltaT); Serial.print("\t"); Serial.print("\t");Serial.print(R_angPosNow); Serial.print("\t");Serial.print("\t"); Serial.print(R_angPosPrev); Serial.print("\t"); Serial.print("\t"); Serial.print(R_deltaT); Serial.print("\t"); Serial.print("\t"); 
    //Serial.print((double)millis()/(double)1000);  Serial.print("\t"); Serial.print("\t"); Serial.print(L_angVelNow); Serial.print("\t"); Serial.print("\t"); Serial.println(R_angVelNow);
    Serial.print((double)millis()/(double)1000);  Serial.print("\t"); Serial.print("\t"); Serial.print(J_linVel); Serial.print("\t"); Serial.print("\t"); Serial.println(J_rotVel);
  }

}

/********************************************* FUNCTIONS *****************************************/

/* update right encoder count values */
void updateRW_countsISR() {
    R_timePrev = R_timeNow; 
    R_timeNow = micros();                 // capture the time enter ISR
    R_deltaT = R_timeNow - R_timePrev;    // calculate time since last ISR
    
    R_countPrev = R_countNow;             // set previous count value to  the current count value that has not been updated 

    R_AChannelNow = digitalRead(A_RW);    // read in Channel A 
    R_BChannelNow = digitalRead(B_RW);    // read in Channel B

    if (R_AChannelNow == R_BChannelNow) R_countNow +=2;
    else R_countNow -=2;

    updateRightWheel();                   // update right wheel velocities and positions
    
    //R_timePrev = micros();
}

/* update left encoder count values */
void updateLW_countsISR() {
    L_timePrev = L_timeNow; 
    L_timeNow = micros();                           // capture the time enter ISR
    L_deltaT = L_timeNow - L_timePrev;              // calculate time since last ISR
  
    L_countPrev = L_countNow;             // set previous count value to  the current count value that has not been updated 

    L_AChannelNow = digitalRead(A_LW);    // read in Channel A 
    L_BChannelNow = digitalRead(B_LW);    // read in Channel B

    if (L_AChannelNow == L_BChannelNow) L_countNow -=2;
    else L_countNow +=2; 

    updateLeftWheel();                    // update right wheel velocities and positions 
    
    //L_timePrev = micros();   

}

/* update right wheel velocities and positions */
void updateRightWheel() {
    //R_timeNow = micros();                           // capture the time enter ISR
    //R_deltaT = R_timeNow - R_timePrev;              // calculate time since last ISR
    
    //if (R_deltaT > tooQuick){
          if(R_countNow != R_countPrev) {
                R_angPosNow = (2.0*PI*(double)R_countNow)/(double)N;
                
                if (abs(R_angPosNow) > fullRotation) {                              // if the wheel has gone a full rotation
                    R_countNow = 0; R_angPosNow = 0.0;                      // reset characteristics to zero
                    R_angVelNow = R_angVelPrev;                             // current angVel set to previous angVel
                } else {
                  R_angVelNow = ((double)R_angPosNow - (double)R_angPosPrev)/((double)R_deltaT/(double)micro);  // else, calculate angVel based on angPos
                }              
                
                R_linVel = (double)radius*(double)R_angVelNow;              // calculate linear velocity

                //Serial.print(micros()); Serial.print("\t"); Serial.println(R_linVel);  

                R_angPosPrev = R_angPosNow;             
                R_angVelPrev = R_angVelNow; 
                //R_timePrev = micros(); // capture time leaving ISR
          }
    //}
}


/* update left wheel velocities and positions */
void updateLeftWheel() {
    //L_timeNow = micros();                           // capture the time enter ISR
    //L_deltaT = L_timeNow - L_timePrev;              // calculate time since last ISR

    //if (L_deltaT > tooQuick){
          if(L_countNow != L_countPrev) {
                L_angPosNow = (2.0*PI*(double)L_countNow)/(double)N;
                
                if (abs(L_angPosNow) > fullRotation) {                            // if the wheel has gone a full rotation
                    L_countNow = 0; L_angPosNow = 0.0;                            // reset characteristics to zero
                    L_angVelNow = L_angVelPrev;                                   // current angVel set to previous angVel
                } else {
                  L_angVelNow = ((double)L_angPosNow - (double)L_angPosPrev)/((double)L_deltaT/(double)micro);  // else, calculate angVel based on angPos
                }

                //Serial.print("R_deltaAngPos: "); Serial.println(R_angPosNow - R_angPosPrev);
                 //Serial.print("L: "); Serial.println((double)L_angPosNow - (double)L_angPosPrev);
                
                L_linVel = (double)radius*(double)L_angVelNow;              // calculate linear velocity

                L_angPosPrev = L_angPosNow;             
                L_angVelPrev = L_angVelNow; 
                //L_timePrev = micros(); // captre time leaving ISR
          }
    //}
}

/* Calculate new position and angle of robot*/
void calculatePositionandVel() {
  timePrev = timeNow;
  timeNow = micros();                             // capture time enter loop
  deltaT = timeNow - timePrev;                    // calculate time since last enter
  
  /* calculate position and angle */
  x_now = x_prev + (((double)deltaT/(double)micro)*double(cos(phi_prev))*(R_linVel + L_linVel))/(2.0);
  y_now = y_prev + (((double)deltaT/(double)micro)*double(sin(phi_prev))*(R_linVel + L_linVel))/(2.0);
  phi_now = phi_prev + ((double)deltaT/(double)micro)*(radius/baseline)*(R_linVel-L_linVel);


  /*robot linear and rotational velocities */
  J_linVel = ((double)R_linVel + (double)L_linVel)/(double)2.0;
  J_rotVel = ((double)L_linVel - (double)R_linVel)/(double)baseline; 
           
  /* set previous values to current values */
  x_prev = x_now;
  y_prev = y_now;
  phi_prev = phi_now;
  //timePrev = micros();   
  
}

/*void updateWheels() {

    if ((L_deltaT > tooQuick) && (R_deltaT > tooQuick)){
          if((L_countNow != L_countPrev) && (R_countNow != R_countPrev)) {
                L_angPosNow = (2.0*PI*(double)L_countNow)/(double)N;
                R_angPosNow = (2.0*PI*(double)R_countNow)/(double)N;
                
                if (abs(L_angPosNow) > fullRotation) {                            // if the wheel has gone a full rotation
                    L_countNow = 0; L_angPosNow = 0.0;                            // reset characteristics to zero
                    L_angVelNow = L_angVelPrev;                                   // current angVel set to previous angVel
                } else {
                  L_angVelNow = ((double)L_angPosNow - (double)L_angPosPrev)/((double)L_deltaT/(double)micro);  // else, calculate angVel based on angPos
                }

                if (abs(R_angPosNow) > fullRotation) {
                  R_countNow = 0; R_angPosNow = 0.0; 
                  R_angVelNow = R_angVelPrev;
                } else {
                  R_angVelNow = ((double)R_angPosNow - (double)R_angPosPrev)/((double)R_deltaT/(double)micro);
                }
                
                L_linVel = radius*L_angVelNow;              // calculate linear velocity
                R_linVel = radius*L_angVelNow;

                L_angPosPrev = L_angPosNow; 
                R_angPosPrev = R_angPosNow;            
                L_angVelPrev = L_angVelNow;
                R_angVelPrev = R_angVelNow; 
                
          }
    }
}*/
