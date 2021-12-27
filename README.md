# EWBIK is a robust Inverse Kinematics library.  
  
See here for a demo: https://youtu.be/y_o34kOx_FA It relies on a novel (I'm still writing an explainer) highly stable generalization of CCD.  
  
**Features:**

*   Position AND orientation targets (6-DOF).
*   Highly stable.
*   Multiple end-effector support
*   Intermediary effector support.
*   Dampening (stiffness control).
*   Target weight/priority (per target, per degree of freedom).
*   Highly versatile 3-DOF constraints with arbitrarily shaped orientation regions.
*   "Soft" constraint support, allowing joints to meet the target in the least uncomfortable way.
  
The code is quite fast and suitable for realtime use in most graphics applications. A fully constrained humanoid torso effectored at the hips, hands and head (simultaneously trying to reach all four corresponding targets in position and orientation) will solve in well under a millisecond (roughly 0.2 milliseconds on a 8 year old mid-level consumer grade CPU). But further optimizations are likely still possible with data-structure tweaks.  
  
Please let me know if you find bugs you can't fix. Please commit back changes for any bugs you do fix.  
  
**DISCLAIMER: This code was intended primarily for graphics applications and has not been thoroughly tested for use in robotics. Until this disclaimer disappears (or you have independently verified it is suitable for your purposes) please do not use this code to command any servos that can put out enough torque to cause damage to people, property, or other components in your build.**

## How to use on Microsoft Windows 11

1. Install openjdk. `scoop bucket add java`
1. Install openjdk. `scoop install openjdk gradle`
1. Add the openjdk bin folder to path.
1. `gradle installDist`
1. `.\build\install\java-ewbik\bin\java-ewbik.bat`
