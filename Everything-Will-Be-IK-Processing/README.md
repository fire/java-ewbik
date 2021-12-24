# Everything-Will-Be-IK-Processing This project serves as both a Processing extension of the robust java Inverse Kinematics library [Everything Will Be IK](https://github.com/EGjoni/Everything-WIll-Be-IK), and an official reference for anyone looking to port or extend EWBIK for their own purposes. See here for a demo: https://youtu.be/y\_o34kOx\_FA  
  
**Features:**

*   Orientation AND position based targets (6-DOF).
*   Highly stable.
*   Multiple end-effector support
*   Intermediary effector support.
*   Dampening (stiffness control).
*   Highly versatile 3-DOF constraints with arbitrarily shaped orientation regions.

  
**Installation Instructions:**  

1.  Download a .zip from this repository.
2.  Locate your processing sketchbook folder. (You can find this from within the Processing IDE by clicking File -> Preferences).
3.  Navigate to that sketchbook directory.
4.  Extract the ewbIK folder into the 'libraries' folder within your sketchbook.

The final directory layout should look something like ``` ..sketchbook/ ┣ libraries/ ┣ ewbIK/ ┣ doc/ ┣ examples/ ┣ library/ ┣ src/ ```

 **DISCLAIMER: This code was intended primarily for graphics applications and has not been thoroughly tested for use in robotics. Until this disclaimer disappears (or you have independently verified it is suitable for your purposes) please do not use this code to command any servos that can put out enough torque to cause damage to people, property, or other components in your build.**