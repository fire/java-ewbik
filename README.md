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
            heading = bone.y_axis
            heading_string = f"{heading.x}, {heading.y}, {heading.z}"
            roll_heading = bone.x_axis
            roll_string = f"{roll.x}, {roll.y}, {roll.z}"
            print(f"addBone(\"{bone.name}\", \"{parent_name}\", {heading_string}, {roll_string}, {bone.length});")
