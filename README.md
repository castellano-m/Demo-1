# Demo-1 - Master Branch
Robot must perform three different tasks.
Each task is assigned to a team member(s) based on their roles in the team (Raspberry Pi or Arduino)
* **Task 1**: Detect and recognize a beacon, and report the angle in degrees between the camera axis (i.e. an arrow pointing out of the camera perpendicular to the camera face) and the beacon. The angle is signed, with a positive angle when the beacon is to the left of the camera axis.
* **Task 2**: Move forward in a straight line and stop after a specified distance in feet (between 1 and 10 feet).
* **Task 3**: Rotate the robot by a specified angle in degrees, and move forward 1 foot.

A short description of the files in this master branch are as followed: 
* **CLContrlDemo1.ino**: This program was used perform Tasks 2 and 3. ISRs for each wheel were used to one, update the encoder counts of the motor, and two, calculate the angular position, angular velocity, and linear velocity of each wheel. In the main loop, the x-y-phi position is calculated and the controls for foward and rotational motion is implemented. Based on the tasks variables - linear distance to move foward or rotational angle to move -  the variable is converted to the associated angular position that each wheel needs to go to. The feedback element of the controls is the individual angular position of each wheel currently. 
Due to time constraints and hardware issues, the team recognizes that much work is needed to make the controls for the robot more robust for the final demo. The team hopes to fine tune how / when the angular position, angular velocity,  linear velocity of each wheel and how x-y-phi is calculated, and re-design the controls to be based off the x-y-phi of the robot as a whole, instead of the individual angular positioning of each wheel. When testing the robot to move certain distance, it was observed that different gain values were needed based on the specified distance (hence the large if-else statement in the setup() loop). The team understands that this design is not optimal to implement moving forward, and the team will eliminate this moving forward.

* **CLControlDemo1v2.ino**: After Monday's demonstration, the Arduino team worked with the TA to understand the issues inherent in the *ControlDemo1.ino code.* The team determined that the calculations of angular velocity and linear velocity in each wheel's ISR was taking so long, such that encoder changes occuring in the other ISR were not being detected. The team then decided to only update encoder counts in each ISR and the angular veloicty and linear velocity were updated in the main loop() at a specified time interval. This code, **CLControlDemo1v2.ino** was tested using only proportional gain Friday afternoon Oct 30 2020 13:30 MDT. This results of this test, were more successful than the demo on Monday. This change in implementation makes the team more confident and prepared going into Demo 2. 

* **Camera_LCD_Code**: Prints the angle and distance values calculated from **MarkerDetect+AngleCalc.py** to the LCD screen

* **MarkerDetect+AngleCalc.py**: Detects any aruco markers that appear within field of view of the camera. Calculated the distance and angle of center of the aruco marker from the center of the camera. 

* **stepExperiment.ino**: This program was used to perform the step reponse experiments of our robot. The experimental values from this code was used to design the controls of our system.
