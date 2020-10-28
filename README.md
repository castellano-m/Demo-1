# Demo-1 - Master Branch
Robot must perform three different tasks.
Each task is assigned to a team member(s) based on their roles in the team (Raspberry Pi or Arduino)
* 1. Detect and recognize a beacon, and report the angle in degrees between the camera axis (i.e. an arrow pointing out of the camera perpendicular to the camera face) and the beacon. The angle is signed, with a positive angle when the beacon is to the left of the camera axis.
* 2. Move forward in a straight line and stop after a specified distance in feet (between 1 and 10 feet).
* 3. Rotate the robot by a specified angle in degrees, and move forward 1 foot.

A short description of the files in this master branch are as followed: 
* **CLContrlDemo1.ino**: This program was used perform Tasks 2 and 3. ISRs for each wheel were used to one, update the encoder counts of the motor, and two, calculate the angular position, angular velocity, and linear velocity of each wheel. In the main loop, the x,y,phi position is calculated and the controls for foward and rotational motion is implemented. Based on the tasks variables - linear distance to move foward or rotational angle to move forward -  the variable is converted to the associated angular position that each wheel needs to go to. The feedback element of the controls is the individual angular position of each wheel currently. 
Due to time constraints and hardware issues, the team recognizes that much work is needed to make the controls for the robot more robust for the final demo. The team hopes to fine tune how / when the angular position, angular velocity,  linear velocity of each wheel and how x-y-phi is calculated; re-design the controls to be based off the x-y-phi of the robot as a whole, instead of the individual angular positioning of each wheel. 
