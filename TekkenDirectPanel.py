import bpy
from . TekkenDirect import TKDirect      

"""
class TekkenDirect:
    attach_player = False
    def setPreview(self, a):
        pass
    def setTracking(self, a):
        pass
    def stop(self):
        pass
"""

TK = TKDirect()

# --- Callback --- #

def onLivePreviewChange(self, context):
    if self.tekken_live_preview_check:
        self.tekken_live_tracking_check = False
    TK.setPreview(self.tekken_live_preview_check)
    
def onLiveTrackingChange(self, context):
    if self.tekken_live_tracking_check:
        self.tekken_live_preview_check = False
    TK.setTracking(self.tekken_live_tracking_check)

def onCameraPreviewChange(self, context):
    if self.tekken_camera_preview_check:
        self.tekken_camera_track_check = False
    TK.setCameraPreview(self.tekken_camera_preview_check)
    
def onCameraTrackingChange(self, context):
    if self.tekken_camera_track_check:
        self.tekken_camera_preview_check = False
    TK.setCameraTracking(self.tekken_camera_track_check)
    
def onAttachPlayerChange(self, context):
    TK.attach_player = self.tekken_attach_other_player_check
    
# --- Checkboxes --- #

checkboxes = [
    {
        "var_name": "tekken_live_preview_check",
        "name": "Live preview (Blender -> Game)",
        "default": False,
        "callback": onLivePreviewChange
    },
    
    {
        "var_name": "tekken_live_tracking_check",
        "name": "Live tracking (Game -> Blender)",
        "default": False,
        "callback": onLiveTrackingChange
    },
    
    {
        "var_name": "tekken_attach_other_player_check",
        "name": "Attach other player",
        "default": False,
        "callback": onAttachPlayerChange
    },
    
    {
        "var_name": "tekken_distance_limit",
        "name": "Disable distance limit",
        "default": False,
        "callback": lambda s, c: TK.setDistanceLimit(s.tekken_distance_limit)
    },
    
    {
        "var_name": "tekken_camera_preview_check",
        "name": "Preview camera",
        "default": False,
        "callback": onCameraPreviewChange
    },
    {
        "var_name": "tekken_camera_track_check",
        "name": "Track camera",
        "default": False,
        "callback": onCameraTrackingChange
    },
    
]

# --- Functions --- #7

def init_variables():
    scene = bpy.context.scene
    
    for variableName in defaultVariables:
        scene[variableName] = defaultVariables[variableName]
    
    
# --- Btns --- #

# PLACEHOLDER
class PLACEHOLDER_YES(bpy.types.Operator):
    bl_idname = "tekken.yes"
    bl_label = "Yes"
    
    def execute(self, context):
        return {'FINISHED'}
    
# PLACEHOLDER
class PLACEHOLDER_NO(bpy.types.Operator):
    bl_idname = "tekken.no"
    bl_label = "No"
    
    def execute(self, context):
        return {'FINISHED'}

# 1p Allow head movement
class AllowHeadMovementBtn(bpy.types.Operator):
    bl_idname = "tekken.allow_head_movement"
    bl_label = "1P - Allow head movement"
    
    def execute(self, context):
        TK.allowHeadMovement(0)
        return {'FINISHED'}
        
# 2p Allow head movement
class AllowHeadMovementBtn2p(bpy.types.Operator):
    bl_idname = "tekken.allow_head_movement_2p"
    bl_label = "2P - Allow head movement"
    
    def execute(self, context):
        TK.allowHeadMovement(1)
        return {'FINISHED'}

# Sets 1p's active skeleton
class SetActiveSkeletonBtn(bpy.types.Operator):
    bl_idname = "tekken.set_skeleton"
    bl_label = "1P - Set active skeleton"
    
    def execute(self, context):
        if context.object.type == 'ARMATURE':
            TK.setActiveSkeleton(0, context.active_object.name)
            context.active_object.show_name = True
            self.report({'INFO'}, "New armature selected")
        else:
            self.report({'INFO'}, "(1p) Not a valid armature")
            TK.armature_name = None
            
        return {'FINISHED'}

# Sets 2P's active skeleton
class Set2pActiveSkeletonBtn(bpy.types.Operator):
    bl_idname = "tekken.set_skeleton_2p"
    bl_label = "2P - Set active skeleton"
    
    def execute(self, context):
        if context.object.type == 'ARMATURE':
            TK.setActiveSkeleton(1, context.active_object.name)
            context.active_object.show_name = True
            self.report({'INFO'}, "New 2P armature selected")
        else:
            self.report({'INFO'}, "(2p) Not a valid armature")
            TK.armature_name_2p = None
        return {'FINISHED'}

# Sets active camera
class SetActiveCamera(bpy.types.Operator):
    bl_idname = "tekken.set_camera"
    bl_label = "Set active camera"
    
    def execute(self, context):
        if context.object.type == 'CAMERA':
            context.scene.objects[context.active_object.name].data.lens_unit = 'FOV'
            context.scene.objects[context.active_object.name].data.angle = 1
            
            TK.camera_name = context.active_object.name
            self.report({'INFO'}, "New camera selected")
        else:
            self.report({'ERROR'}, "No valid camera selected")
        return {'FINISHED'}

# --- Main panel --- #

class TekkenPanel(bpy.types.Panel):
    
    bl_idname = "VIEW3D_PT_tekken"
    bl_label = 'Tekken Direct'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    
    def draw(self, context):
        l = self.layout
        
        # Live tracking #
        
        l.prop(context.scene, "tekken_live_tracking_check")
        l.prop(context.scene, "tekken_live_preview_check")
        
        l.label(text='____________________________________')
        
        # Live preview #
        
        
        l.operator(SetActiveSkeletonBtn.bl_idname)
        l.operator(AllowHeadMovementBtn.bl_idname)
        
        l.prop(context.scene, "p1_collision")
        l.prop(context.scene, "tekken_attach_other_player_check")
        l.prop(context.scene, "tekken_distance_limit")
        
        #2p
        l.operator(Set2pActiveSkeletonBtn.bl_idname)
        l.operator(AllowHeadMovementBtn2p.bl_idname)
        
        #Camera: unused because needs more math
        l.label(text='____________________________________')
        l.prop(context.scene, "tekken_camera_preview_check")
        l.prop(context.scene, "tekken_camera_track_check")
        l.operator(SetActiveCamera.bl_idname)
        
        
# --- Registering --- #

classes = [
    PLACEHOLDER_YES,
    PLACEHOLDER_NO,
    SetActiveSkeletonBtn,
    Set2pActiveSkeletonBtn,
    AllowHeadMovementBtn,
    AllowHeadMovementBtn2p,
    SetActiveCamera,
    TekkenPanel
]

def register():
    for propInfo in checkboxes:
        setattr(bpy.types.Scene, propInfo["var_name"], bpy.props.BoolProperty(
            name = propInfo["name"],
            default = propInfo["default"],
            update = propInfo["callback"],
        ))
        
    bpy.types.Scene.p1_collision = bpy.props.IntProperty(
        name = "Players Collision",
        default = 0,
        min = 0,
        max = 10,
        update = lambda s, c: TK.setPlayerCollision(s.p1_collision)
    )
        
    for c in classes:
        bpy.utils.register_class(c)
        #print("Registered class " + str(c))
    
    print("Registered")
    
def unregister():
    TK.stop()
    
    for c in classes:
        bpy.utils.unregister_class(c)
            
    print("Unregistered")

if __name__ == '__main__':
    init_variables()
    register()