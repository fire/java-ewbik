EWBIK-Workspace

# How to use on Win10

1. Install openjdk. `scoop bucket add java`
1. Install openjdk. `scoop install openjdk gradle`
1. Add the openjdk bin folder to path.
1. `gradle installDist`
1. `.\build\install\ewbik-workspace\bin\ewbik-workspace.bat`

## Script
    # Be in edit mode
    ob = bpy.context.object
    if ob.type == 'ARMATURE':
    armature = ob.data
    for bone in armature.edit_bones:
        parent_name = ""
        if bone.parent:        
        parent_name = bone.parent.name
        r = bone.matrix.to_quaternion()
        r_string = f"{r.x}, {r.y}, {r.z}, {r.w}"
        l = bone.matrix.translation
        l_string = f"{l.x}, {l.y}, {l.z}"
        print(f"addBone(\"{bone.name}\", \"{parent_name}\", {bone.length}, {r_string}, {l_string});")
