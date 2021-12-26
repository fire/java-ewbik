EWBIK-Workspace

# How to use on Win10

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