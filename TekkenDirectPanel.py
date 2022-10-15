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

def onStop():
    bpy.context.scene["tekken_live_preview"] = False
    bpy.context.scene["tekken_live_hand_preview"] = False
    bpy.context.scene["tekken_live_face_preview"] = False
    bpy.context.scene["tekken_live_face_2p_preview"] = False
    bpy.context.scene["tekken_live_tracking"] = False
    bpy.context.scene["tekken_camera_preview"] = False
    bpy.context.scene["tekken_camera_track"] = False
    
TK = TKDirect(onStop)

# --- Callback --- #

def onLivePreviewChange(self, context):
    if self.tekken_live_preview:
        self.tekken_live_tracking = False
    TK.setPreview(self.tekken_live_preview)
    
def onLiveTrackingChange(self, context):
    if self.tekken_live_tracking:
        self.tekken_live_preview = False
    TK.setTracking(self.tekken_live_tracking)

def onCameraPreviewChange(self, context):
    if self.tekken_camera_preview and self.tekken_camera_track:
        self.tekken_camera_track = False
    TK.setCameraPreview(self.tekken_camera_preview)
    
def onCameraTrackingChange(self, context):
    if self.tekken_camera_track and self.tekken_camera_preview:
        self.tekken_camera_preview = False
    TK.setCameraTracking(self.tekken_camera_track)
    
def onAttachPlayerChange(self, context):
    TK.attach_player = self.tekken_attach_other_player
    
# --- Checkboxes --- #

checkboxes = [
    {
        "var_name": "tekken_live_preview",
        "name": "Live preview",
        "default": False,
        "callback": onLivePreviewChange
    },
    
    {
        "var_name": "tekken_live_hand_preview",
        "name": "Live hand",
        "default": False,
        "callback": lambda s, c: TK.onHandLivePreviewChange(s.tekken_live_hand_preview) 
    },
    
    {
        "var_name": "tekken_live_face_preview",
        "name": "(1P) Live face",
        "default": False,
        "callback": lambda s, c: TK.onFaceLivePreviewChange(s.tekken_live_face_preview) 
    },
    
    
    {
        "var_name": "tekken_live_face_2p_preview",
        "name": "(2P) Live face",
        "default": False,
        "callback": lambda s, c: TK.on2PFaceLivePreviewChange(s.tekken_live_face_2p_preview) 
    },
    
    {
        "var_name": "tekken_live_tracking",
        "name": "Live tracking (Game -> Blender)",
        "default": False,
        "callback": onLiveTrackingChange
    },
    
    {
        "var_name": "tekken_attach_other_player",
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
        "var_name": "tekken_player_collision",
        "name": "Disable player collision",
        "default": False,
        "callback": lambda s, c: TK.setPlayerCollision(s.tekken_player_collision)
    },
    {
        "var_name": "tekken_wall_collision",
        "name": "Disable wall collision",
        "default": False,
        "callback": lambda s, c: TK.setWallCollision(s.tekken_wall_collision)
    },
    
    {
        "var_name": "tekken_camera_preview",
        "name": "Preview camera",
        "default": False,
        "callback": onCameraPreviewChange
    },
    {
        "var_name": "tekken_camera_track",
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

"""
class debugBtn(bpy.types.Operator):
    bl_idname = "tekken.debug_btn"
    bl_label = "1P - Set face anim 0"
    
    def execute(self, context):
        TK.writePlayerFaceAnim(0)
        return {'FINISHED'}
"""

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
            if context.active_object.name == TK.armature_name_2p:
                self.report({'INFO'}, "(1p) Already in use by 2p")
            else:
                TK.setActiveSkeleton(0, context.active_object.name)
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
            if context.active_object.name == TK.armature_name:
                self.report({'INFO'}, "(1p) Already in use by 1p")
            else:
                TK.setActiveSkeleton(1, context.active_object.name)
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
            cam = context.scene.objects[context.active_object.name]
            cam.data.lens_unit = 'FOV'
            cam.data.angle = 1
            cam.rotation_mode = "YZX"
            
            TK.camera_name = context.active_object.name
            self.report({'INFO'}, "New camera selected")
        else:
            self.report({'ERROR'}, "No valid camera selected")
        return {'FINISHED'}

# --- Main panel --- #

class TekkenPanel(bpy.types.Panel):
    bl_category = "Tekken"
    bl_idname = "VIEW3D_PT_tekken"
    bl_label = 'Tekken Direct'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    
    def draw(self, context):
        l = self.layout
        
        # Live tracking #
        
        l.label(text='- Tracking (Game - Blender) -')
        l.prop(context.scene, "tekken_live_tracking")
        
        l.label(text='- Previews (Blender -> Game) -')
        l.prop(context.scene, "tekken_live_preview")
        l.prop(context.scene, "tekken_live_hand_preview")
        l.prop(context.scene, "tekken_live_face_preview")
        l.prop(context.scene, "tekken_live_face_2p_preview")
        
        l.label(text='____________________________________')
        
        # Live preview #
        #l.operator(debugBtn.bl_idname) #to remove
        
        l.operator(SetActiveSkeletonBtn.bl_idname)
        l.operator(AllowHeadMovementBtn.bl_idname)
        
        l.prop(context.scene, "stage_collision")
        l.prop(context.scene, "tekken_attach_other_player")
        l.prop(context.scene, "tekken_distance_limit")
        l.prop(context.scene, "tekken_player_collision")
        l.prop(context.scene, "tekken_wall_collision")
        
        #2p
        l.operator(Set2pActiveSkeletonBtn.bl_idname)
        l.operator(AllowHeadMovementBtn2p.bl_idname)
        
        #Camera: unused because needs more math
        l.label(text='____________________________________')
        l.prop(context.scene, "tekken_camera_preview")
        l.prop(context.scene, "tekken_camera_track")
        l.operator(SetActiveCamera.bl_idname)
        
        
        l.prop(context.scene, "bone_target_test")
        l.prop(context.scene, "val1_test")
        l.prop(context.scene, "val2_test")
        l.prop(context.scene, "val3_test")
        
        
# --- Registering --- #

classes = [
    #debugBtn, #to remove
    
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
        
    bpy.types.Scene.stage_collision = bpy.props.IntProperty(
        name = "Stage collision",
        default = 0,
        min = 0,
        max = 10,
        update = lambda s, c: TK.setPlayersStageCollision(s.stage_collision)
    )
    
    bpy.types.Scene.bone_target_test = bpy.props.IntProperty(
        name = "Bone target",
        default = 8,
        min = 0,
        max = 24,
        update = lambda s, c: TK.setBoneTarget(s.bone_target_test)
    )
    bpy.types.Scene.val1_test = bpy.props.FloatProperty(
        name = "Mat 1",
        default = 0,
        min = -7,
        max = 7,
        update = lambda s, c: TK.setVal(s.val1_test, 1)
    )
    bpy.types.Scene.val2_test = bpy.props.FloatProperty(
        name = "Mat 2",
        default = 0,
        min = -7,
        max = 7,
        update = lambda s, c: TK.setVal(s.val1_test, 2)
    )
    bpy.types.Scene.val3_test = bpy.props.FloatProperty(
        name = "Mat 3",
        default = 0,
        min = -7,
        max = 7,
        update = lambda s, c: TK.setVal(s.val1_test, 3)
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