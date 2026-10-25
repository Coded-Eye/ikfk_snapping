import bpy
from mathutils import Vector


class FKIK_LimbSettings(bpy.types.PropertyGroup):
    fk_bone_1: bpy.props.StringProperty(
        name="FK Bone 1",
        description="FK upper arm/thigh",
        default=""
    )
    fk_bone_2: bpy.props.StringProperty(
        name="FK Bone 2",
        description="FK forearm/calf",
        default=""
    )
    fk_bone_3: bpy.props.StringProperty(
        name="FK Bone 3",
        description="FK hand/foot",
        default=""
    )

    deform_bone_1: bpy.props.StringProperty(
        name="Deform Bone 1",
        description="deform upper arm/thigh",
        default=""
    )
    deform_bone_2: bpy.props.StringProperty(
        name="Deform Bone 2",
        description="deform forearm/calf",
        default=""
    )
    deform_bone_3: bpy.props.StringProperty(
        name="Deform Bone 3",
        description="deform hand/foot",
        default=""
    )

    ik_target: bpy.props.StringProperty(
        name="IK Target",
        description="IK target hand/foot",
        default=""
    )
    ik_pole: bpy.props.StringProperty(
        name="IK Pole",
        description="IK pole elbow/knee",
        default=""
    )


class FKIK_ArmatureSettings(bpy.types.PropertyGroup):
    left_arm: bpy.props.PointerProperty(type=FKIK_LimbSettings)
    right_arm: bpy.props.PointerProperty(type=FKIK_LimbSettings)
    left_leg: bpy.props.PointerProperty(type=FKIK_LimbSettings)
    right_leg: bpy.props.PointerProperty(type=FKIK_LimbSettings)

def get_pose_bone(armature, bone_name):
    if not armature or not bone_name:
        return None
    return armature.pose.bones.get(bone_name)


def snap_fk_to_ik(armature, limb_settings):
    if not armature:
        return False, "No armature found"

    # Get bones - matching original variable names
    deform_upper = get_pose_bone(armature, limb_settings.deform_bone_1)
    deform_forearm = get_pose_bone(armature, limb_settings.deform_bone_2)
    deform_hand = get_pose_bone(armature, limb_settings.deform_bone_3)

    fk_upper = get_pose_bone(armature, limb_settings.fk_bone_1)
    fk_forearm = get_pose_bone(armature, limb_settings.fk_bone_2)
    fk_hand = get_pose_bone(armature, limb_settings.fk_bone_3)

    ik_target = get_pose_bone(armature, limb_settings.ik_target)

    if not all([deform_upper, deform_forearm, fk_upper, fk_forearm, fk_hand, ik_target]):
        return False, "One or more bones not found"

    fk_upper.matrix = deform_upper.matrix
    bpy.context.view_layer.update()

    fk_forearm.matrix = deform_forearm.matrix
    bpy.context.view_layer.update()

    fk_hand.matrix = ik_target.matrix
    bpy.context.view_layer.update()

    return True, "FK snapped to IK"


def snap_ik_to_fk(armature, limb_settings):
    if not armature:
        return False, "No armature found"

    # Get required bones
    fk_bone_1 = get_pose_bone(armature, limb_settings.fk_bone_1)
    fk_bone_2 = get_pose_bone(armature, limb_settings.fk_bone_2)
    fk_bone_3 = get_pose_bone(armature, limb_settings.fk_bone_3)
    ik_target = get_pose_bone(armature, limb_settings.ik_target)
    ik_pole = get_pose_bone(armature, limb_settings.ik_pole)

    if not fk_bone_3:
        return False, "FK end bone not found"
    if not ik_target:
        return False, "IK target bone not found"

    # Snap IK target to FK hand/foot
    ik_target.matrix = fk_bone_3.matrix.copy()
    bpy.context.view_layer.update()

    # Calculate and set pole vector position if pole bone exists
    if ik_pole and fk_bone_1 and fk_bone_2:
        # Get joint positions in world space
        upper_head = armature.matrix_world @ fk_bone_1.head
        lower_head = armature.matrix_world @ fk_bone_2.head
        end_head = armature.matrix_world @ fk_bone_3.head

        limb_axis = (end_head - upper_head).normalized()

        to_mid = lower_head - upper_head

        # Project onto limb axis to find the perpendicular component
        proj_length = to_mid.dot(limb_axis)
        proj = limb_axis * proj_length

        pole_direction = (to_mid - proj).normalized()

        pole_distance = (upper_head - lower_head).length
        pole_position = lower_head + pole_direction * pole_distance

        pole_local = armature.matrix_world.inverted() @ pole_position

        if ik_pole.parent:
            parent_matrix = ik_pole.parent.matrix
            pole_local = parent_matrix.inverted() @ armature.pose.bones[limb_settings.ik_pole].bone.matrix_local @ Vector((0, 0, 0))
            ik_pole.matrix.translation = pole_position

        ik_pole.location = ik_pole.bone.matrix_local.inverted() @ pole_local

        bpy.context.view_layer.update()

    return True, "IK snapped to FK"


class FKIK_OT_SnapBase:
    @classmethod
    def poll(cls, context):
        return (context.active_object and
                context.active_object.type == 'ARMATURE' and
                context.mode == 'POSE')

    def get_limb_name(self):
        return self.limb

    def get_direction(self):
        return self.direction

    def execute(self, context):
        armature = context.active_object

        # Ensure the property exists
        if not hasattr(armature, 'fkik_settings'):
            self.report({'ERROR'}, "FK/IK settings not found. Try re-enabling the addon.")
            return {'CANCELLED'}

        settings = armature.fkik_settings
        limb_name = self.get_limb_name()

        limb_settings = getattr(settings, limb_name, None)
        if not limb_settings:
            self.report({'ERROR'}, f"Limb settings not found: {limb_name}")
            return {'CANCELLED'}

        direction = self.get_direction()
        if direction == "fk_to_ik":
            success, message = snap_fk_to_ik(armature, limb_settings)
        else:
            success, message = snap_ik_to_fk(armature, limb_settings)

        if success:
            self.report({'INFO'}, message)
            return {'FINISHED'}
        else:
            self.report({'ERROR'}, message)
            return {'CANCELLED'}


class FKIK_OT_SnapFKtoIK_LeftArm(FKIK_OT_SnapBase, bpy.types.Operator):
    bl_idname = "pose.fkik_snap_fk_to_ik_left_arm"
    bl_label = "FK → IK (Left Arm)"
    bl_description = "Snap FK controls to match IK pose for left arm"
    bl_options = {'REGISTER', 'UNDO'}

    def get_limb_name(self):
        return "left_arm"

    def get_direction(self):
        return "fk_to_ik"


class FKIK_OT_SnapIKtoFK_LeftArm(FKIK_OT_SnapBase, bpy.types.Operator):
    bl_idname = "pose.fkik_snap_ik_to_fk_left_arm"
    bl_label = "IK → FK (Left Arm)"
    bl_description = "Snap IK controls to match FK pose for left arm"
    bl_options = {'REGISTER', 'UNDO'}

    def get_limb_name(self):
        return "left_arm"

    def get_direction(self):
        return "ik_to_fk"


class FKIK_OT_SnapFKtoIK_RightArm(FKIK_OT_SnapBase, bpy.types.Operator):
    bl_idname = "pose.fkik_snap_fk_to_ik_right_arm"
    bl_label = "FK → IK (Right Arm)"
    bl_description = "Snap FK controls to match IK pose for right arm"
    bl_options = {'REGISTER', 'UNDO'}

    def get_limb_name(self):
        return "right_arm"

    def get_direction(self):
        return "fk_to_ik"


class FKIK_OT_SnapIKtoFK_RightArm(FKIK_OT_SnapBase, bpy.types.Operator):
    bl_idname = "pose.fkik_snap_ik_to_fk_right_arm"
    bl_label = "IK → FK (Right Arm)"
    bl_description = "Snap IK controls to match FK pose for right arm"
    bl_options = {'REGISTER', 'UNDO'}

    def get_limb_name(self):
        return "right_arm"

    def get_direction(self):
        return "ik_to_fk"


class FKIK_OT_SnapFKtoIK_LeftLeg(FKIK_OT_SnapBase, bpy.types.Operator):
    bl_idname = "pose.fkik_snap_fk_to_ik_left_leg"
    bl_label = "FK → IK (Left Leg)"
    bl_description = "Snap FK controls to match IK pose for left leg"
    bl_options = {'REGISTER', 'UNDO'}

    def get_limb_name(self):
        return "left_leg"

    def get_direction(self):
        return "fk_to_ik"


class FKIK_OT_SnapIKtoFK_LeftLeg(FKIK_OT_SnapBase, bpy.types.Operator):
    bl_idname = "pose.fkik_snap_ik_to_fk_left_leg"
    bl_label = "IK → FK (Left Leg)"
    bl_description = "Snap IK controls to match FK pose for left leg"
    bl_options = {'REGISTER', 'UNDO'}

    def get_limb_name(self):
        return "left_leg"

    def get_direction(self):
        return "ik_to_fk"


class FKIK_OT_SnapFKtoIK_RightLeg(FKIK_OT_SnapBase, bpy.types.Operator):
    bl_idname = "pose.fkik_snap_fk_to_ik_right_leg"
    bl_label = "FK → IK (Right Leg)"
    bl_description = "Snap FK controls to match IK pose for right leg"
    bl_options = {'REGISTER', 'UNDO'}

    def get_limb_name(self):
        return "right_leg"

    def get_direction(self):
        return "fk_to_ik"


class FKIK_OT_SnapIKtoFK_RightLeg(FKIK_OT_SnapBase, bpy.types.Operator):
    bl_idname = "pose.fkik_snap_ik_to_fk_right_leg"
    bl_label = "IK → FK (Right Leg)"
    bl_description = "Snap IK controls to match FK pose for right leg"
    bl_options = {'REGISTER', 'UNDO'}

    def get_limb_name(self):
        return "right_leg"

    def get_direction(self):
        return "ik_to_fk"

# need to have at least set either the left or right bone
class FKIK_OP_Symmetrize(bpy.types.Operator):
    bl_label = "Symmetrize"
    bl_idname = "pose.fkik_symmetrize"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Rig Tools'
    bl_parent_id = "VIEW3D_PT_fkik_main"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (context.active_object and
                context.active_object.type == 'ARMATURE' and
                (context.mode == 'POSE' or context.mode == "EDIT_ARMATURE"))


    def bone_exists(self, armature, bone_name):
        return bone_name in armature.data.bones

    def sym_bone(self, armature, left_limb, right_limb, bone_name):
        left_val = getattr(left_limb, bone_name)
        right_val = getattr(right_limb, bone_name)

        # check if bone str it's not empty
        if left_val != "":
            # check if bone exists and set it if it does
            left_val = left_val.replace(".L", ".R")
            if self.bone_exists(armature, left_val):
                setattr(right_limb, bone_name, left_val)
        elif right_val != "":
            right_val = right_val.replace(".R", ".L")
            if self.bone_exists(armature, right_val):
                setattr(left_limb, bone_name, right_val)


    def execute(self, context):  # pyright: ignore[reportIncompatibleMethodOverride]
        armature = context.active_object
        settings = armature.fkik_settings

        bones = list(FKIK_LimbSettings.__annotations__.keys())
        for bone in bones:
            self.sym_bone(armature, settings.left_arm, settings.right_arm, bone)
            self.sym_bone(armature, settings.left_leg, settings.right_leg, bone)

        self.report({'INFO'}, "mirrored")

        return {'FINISHED'}


class FKIK_PT_MainPanel(bpy.types.Panel):
    bl_label = "FK/IK Snapping"
    bl_idname = "VIEW3D_PT_fkik_main"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Rig Tools'

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def draw(self, context):
        layout = self.layout

        layout.operator("pose.fkik_symmetrize", text="symmetrize")

        if context.mode != 'POSE':
            layout.label(text="Enter Pose Mode to use", icon='INFO')
            return


class FKIK_PT_LeftArmPanel(bpy.types.Panel):
    bl_label = "Left Arm"
    bl_idname = "VIEW3D_PT_fkik_left_arm"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Rig Tools'
    bl_parent_id = "VIEW3D_PT_fkik_main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        armature = context.active_object
        settings = armature.fkik_settings.left_arm

        # Snap buttons
        row = layout.row(align=True)
        row.operator("pose.fkik_snap_fk_to_ik_left_arm", text="FK → IK")
        row.operator("pose.fkik_snap_ik_to_fk_left_arm", text="IK → FK")


class FKIK_PT_LeftArmSettings(bpy.types.Panel):
    bl_label = "Bone Settings"
    bl_idname = "VIEW3D_PT_fkik_left_arm_settings"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Rig Tools'
    bl_parent_id = "VIEW3D_PT_fkik_left_arm"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        armature = context.active_object
        settings = armature.fkik_settings.left_arm

        layout.label(text="FK Bones:")
        layout.prop_search(settings, "fk_bone_1", armature.pose, "bones", text="Upper")
        layout.prop_search(settings, "fk_bone_2", armature.pose, "bones", text="Lower")
        layout.prop_search(settings, "fk_bone_3", armature.pose, "bones", text="End")

        layout.separator()
        layout.label(text="Deform Bones:")
        layout.prop_search(settings, "deform_bone_1", armature.pose, "bones", text="Upper")
        layout.prop_search(settings, "deform_bone_2", armature.pose, "bones", text="Lower")
        layout.prop_search(settings, "deform_bone_3", armature.pose, "bones", text="End")

        layout.separator()
        layout.label(text="IK Controls:")
        layout.prop_search(settings, "ik_target", armature.pose, "bones", text="Target")
        layout.prop_search(settings, "ik_pole", armature.pose, "bones", text="Pole")


class FKIK_PT_RightArmPanel(bpy.types.Panel):
    bl_label = "Right Arm"
    bl_idname = "VIEW3D_PT_fkik_right_arm"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Rig Tools'
    bl_parent_id = "VIEW3D_PT_fkik_main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        row = layout.row(align=True)
        row.operator("pose.fkik_snap_fk_to_ik_right_arm", text="FK → IK")
        row.operator("pose.fkik_snap_ik_to_fk_right_arm", text="IK → FK")


class FKIK_PT_RightArmSettings(bpy.types.Panel):
    bl_label = "Bone Settings"
    bl_idname = "VIEW3D_PT_fkik_right_arm_settings"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Rig Tools'
    bl_parent_id = "VIEW3D_PT_fkik_right_arm"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        armature = context.active_object
        settings = armature.fkik_settings.right_arm

        layout.label(text="FK Bones:")
        layout.prop_search(settings, "fk_bone_1", armature.pose, "bones", text="Upper")
        layout.prop_search(settings, "fk_bone_2", armature.pose, "bones", text="Lower")
        layout.prop_search(settings, "fk_bone_3", armature.pose, "bones", text="End")

        layout.separator()
        layout.label(text="Deform Bones:")
        layout.prop_search(settings, "deform_bone_1", armature.pose, "bones", text="Upper")
        layout.prop_search(settings, "deform_bone_2", armature.pose, "bones", text="Lower")
        layout.prop_search(settings, "deform_bone_3", armature.pose, "bones", text="End")

        layout.separator()
        layout.label(text="IK Controls:")
        layout.prop_search(settings, "ik_target", armature.pose, "bones", text="Target")
        layout.prop_search(settings, "ik_pole", armature.pose, "bones", text="Pole")


class FKIK_PT_LeftLegPanel(bpy.types.Panel):
    bl_label = "Left Leg"
    bl_idname = "VIEW3D_PT_fkik_left_leg"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Rig Tools'
    bl_parent_id = "VIEW3D_PT_fkik_main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        row = layout.row(align=True)
        row.operator("pose.fkik_snap_fk_to_ik_left_leg", text="FK → IK")
        row.operator("pose.fkik_snap_ik_to_fk_left_leg", text="IK → FK")


class FKIK_PT_LeftLegSettings(bpy.types.Panel):
    bl_label = "Bone Settings"
    bl_idname = "VIEW3D_PT_fkik_left_leg_settings"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Rig Tools'
    bl_parent_id = "VIEW3D_PT_fkik_left_leg"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        armature = context.active_object
        settings = armature.fkik_settings.left_leg

        layout.label(text="FK Bones:")
        layout.prop_search(settings, "fk_bone_1", armature.pose, "bones", text="Upper")
        layout.prop_search(settings, "fk_bone_2", armature.pose, "bones", text="Lower")
        layout.prop_search(settings, "fk_bone_3", armature.pose, "bones", text="End")

        layout.separator()
        layout.label(text="Deform Bones:")
        layout.prop_search(settings, "deform_bone_1", armature.pose, "bones", text="Upper")
        layout.prop_search(settings, "deform_bone_2", armature.pose, "bones", text="Lower")
        layout.prop_search(settings, "deform_bone_3", armature.pose, "bones", text="End")

        layout.separator()
        layout.label(text="IK Controls:")
        layout.prop_search(settings, "ik_target", armature.pose, "bones", text="Target")
        layout.prop_search(settings, "ik_pole", armature.pose, "bones", text="Pole")


class FKIK_PT_RightLegPanel(bpy.types.Panel):
    bl_label = "Right Leg"
    bl_idname = "VIEW3D_PT_fkik_right_leg"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Rig Tools'
    bl_parent_id = "VIEW3D_PT_fkik_main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        row = layout.row(align=True)
        row.operator("pose.fkik_snap_fk_to_ik_right_leg", text="FK → IK")
        row.operator("pose.fkik_snap_ik_to_fk_right_leg", text="IK → FK")


class FKIK_PT_RightLegSettings(bpy.types.Panel):
    bl_label = "Bone Settings"
    bl_idname = "VIEW3D_PT_fkik_right_leg_settings"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Rig Tools'
    bl_parent_id = "VIEW3D_PT_fkik_right_leg"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        armature = context.active_object
        settings = armature.fkik_settings.right_leg

        layout.label(text="FK Bones:")
        layout.prop_search(settings, "fk_bone_1", armature.pose, "bones", text="Upper")
        layout.prop_search(settings, "fk_bone_2", armature.pose, "bones", text="Lower")
        layout.prop_search(settings, "fk_bone_3", armature.pose, "bones", text="End")

        layout.separator()
        layout.label(text="Deform Bones:")
        layout.prop_search(settings, "deform_bone_1", armature.pose, "bones", text="Upper")
        layout.prop_search(settings, "deform_bone_2", armature.pose, "bones", text="Lower")
        layout.prop_search(settings, "deform_bone_3", armature.pose, "bones", text="End")

        layout.separator()
        layout.label(text="IK Controls:")
        layout.prop_search(settings, "ik_target", armature.pose, "bones", text="Target")
        layout.prop_search(settings, "ik_pole", armature.pose, "bones", text="Pole")





classes = (
    FKIK_LimbSettings,
    FKIK_ArmatureSettings,
    FKIK_OT_SnapFKtoIK_LeftArm,
    FKIK_OT_SnapIKtoFK_LeftArm,
    FKIK_OT_SnapFKtoIK_RightArm,
    FKIK_OT_SnapIKtoFK_RightArm,
    FKIK_OT_SnapFKtoIK_LeftLeg,
    FKIK_OT_SnapIKtoFK_LeftLeg,
    FKIK_OT_SnapFKtoIK_RightLeg,
    FKIK_OT_SnapIKtoFK_RightLeg,
    FKIK_OP_Symmetrize,
    FKIK_PT_MainPanel,
    FKIK_PT_LeftArmPanel,
    FKIK_PT_LeftArmSettings,
    FKIK_PT_RightArmPanel,
    FKIK_PT_RightArmSettings,
    FKIK_PT_LeftLegPanel,
    FKIK_PT_LeftLegSettings,
    FKIK_PT_RightLegPanel,
    FKIK_PT_RightLegSettings,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Object.fkik_settings = bpy.props.PointerProperty(type=FKIK_ArmatureSettings)


def unregister():
    del bpy.types.Object.fkik_settings

    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
