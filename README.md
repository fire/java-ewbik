# EWBIK is a robust Inverse Kinematics library.

See here for a demo: https://youtu.be/y_o34kOx_FA. It relies on a novel, highly stable generalization of CCD.

**DISCLAIMER: This code was intended primarily for graphics applications and has not been thoroughly tested for use in robotics. Until this disclaimer disappears (or you have independently verified it is suitable for your purposes), please do not use this code to command any servos that can put out enough torque to cause damage to people, property, or other components in your build.**

**Features:**

- Position AND orientation targets (6-DOF).
- Highly stable.
- Multiple end-effector support
- Intermediary effector support.
- Dampening (stiffness control).
- Target weight/priority (per target, degree of freedom).
- Highly versatile 3-DOF constraints with arbitrarily shaped orientation regions.
- "Soft" constraint support, allowing joints to meet the target least uncomfortably.

The code is fast and suitable for real-time use in most graphics applications. For example, a humanoid can be fully constrained and effector-ed at the hips, hands and head. This humanoid (simultaneously trying to reach all four corresponding targets in position and orientation) will solve in under a millisecond (roughly 0.2 milliseconds on an 8-year-old mid-level consumer-grade CPU).

Further optimizations are likely still possible with InverseKinematics-structure tweaks.

Any improvements are welcome as a Github pull request.

Post a Github issue if you find bugs you can't fix.

## How to use EWBIK on Microsoft Windows 11

1. Install OpenJDK. `scoop bucket add java`
1. Install openjdk. `scoop install openjdk gradle`
1. Add the OpenJDK bin folder to the path.
1. `gradle installDist`
1. `.\build\install\java-ewbik\bin\java-ewbik.bat`

## See also

https://github.com/godot-extended-libraries/ewbik

## Special thanks

Special thanks to Humble Tim https://github.com/humbletim for assistance and troubleshooting.
