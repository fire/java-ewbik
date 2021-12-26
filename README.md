# Everything-WIll-Be-IK 'Everything Will Be IK' is a robust Inverse Kinematics library.  
  
See here for a demo: https://youtu.be/y\_o34kOx\_FA It relies on a novel (I'm still writing an explainer) highly stable generalization of CCD.  
  
**Features:**

*   Position AND orientation targets (6-DOF).
*   Highly stable.
*   Multiple end-effector support
*   Intermediary effector support.
*   Dampening (stiffness control).
*   Target weight/priority (per target, per degree of freedom).
*   Highly versatile 3-DOF constraints with arbitrarily shaped orientation regions.
*   "Soft" constraint support, allowing joints to meet the target in the least uncomfortable way.

  
The code is quite fast and suitable for realtime use in most graphics applications. A fully constrained humanoid torso effectored at the hips, hands and head (simultaneously trying to reach all four corresponding targets in position and orientation) will solve in well under a millisecond (roughly 0.2 milliseconds on an 8 year old mid-level consumer grade CPU). But further optimizations are likely still possible with data-structure tweaks.  
  
Please let me know if you find bugs you can't fix. Please commit back changes for any bugs you do fix.  
  
**DISCLAIMER: This code was intended primarily for graphics applications and has not been thoroughly tested for use in robotics. Until this disclaimer disappears (or you have independently verified it is suitable for your purposes) please do not use this code to command any servos that can put out enough torque to cause damage to people, property, or other components in your build.**

## How to use on Win10

1. Install openjdk. `scoop bucket add java`
1. Install openjdk. `scoop install openjdk gradle`
1. Add the openjdk bin folder to path.
1. `gradle installDist`
1. `.\build\install\ewbik-workspace\bin\ewbik-workspace.bat`

## GDScript
extends Skeleton3D

func _ready():	
	var file = File.new()
	file.open("user://save_armature.dat", File.WRITE)
	for bone_i in get_bone_count():
		var bone_name = get_bone_name(bone_i)
		if bone_name.is_empty():
			continue
		var bone_parent_i = get_bone_parent(bone_i)
		var bone_armature_pose = get_bone_global_pose(bone_i)
		var height = 0
		var parent_name = ""
		if bone_parent_i != -1:
			parent_name = get_bone_name(bone_parent_i)
			var parent_armature_pose = get_bone_global_pose(bone_parent_i)
			var origin = (parent_armature_pose.affine_inverse() * bone_armature_pose).origin
			height = parent_armature_pose.origin.y - bone_armature_pose.origin.y
		file.store_line("addBone(\"%s\", \"%s\", %s);"% [get_bone_name(bone_i), parent_name, height])
		var orientation = bone_armature_pose.basis
		var quaternion = orientation.get_rotation_quaternion()
		var origin = bone_armature_pose.origin
		file.store_line("addPin(\"%s\", %s, %s, %s, %s, %s, %s, %s);" % [bone_name, origin.x, origin.y, origin.z, quaternion.x, quaternion.y ,quaternion.z, quaternion.w])
	file.close()