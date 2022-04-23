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

# --- #

defaultVariables = {
    "tekken_armature": None,
    "tekken_armature_2p": None,
    "tekken_camera": None,
    "tekken_live_preview": False,
    "tekken_live_tracking": False
}

storedVariables = {}

def getVal(key):
    global storedVariables
    if key not in storedVariables:
        storedVariables[key] = defaultVariables[key]
    return storedVariables[key] 

def setVal(key, value):
    global storedVariables
    storedVariables[key] = value

# --- Callback --- #

def onPlayerCollisionChange(self, context):
    TK.setPlayerCollision(self.p1_collision)

def onLivePreviewChange(self, context):
    setVal("tekken_live_preview", self.tekken_live_preview_check)
    
    if getVal("tekken_live_preview") and getVal("tekken_live_tracking"):
        setVal("tekken_live_tracking", False)
        setVal("tekken_live_tracking_check", False)
    
    TK.setPreview(getVal("tekken_live_preview"))
    
    print("onLivePreviewChange", self.tekken_live_preview_check)
    
def onLiveTrackingChange(self, context):
    setVal("tekken_live_tracking", self.tekken_live_tracking_check == 1)
    
    if getVal("tekken_live_tracking") and getVal("tekken_live_preview"):
        setVal("tekken_live_preview", False)
        setVal("tekken_live_preview_check", False)
        
    TK.setTracking(getVal("tekken_live_tracking"))
    
    print("onLiveTrackingChange", self.tekken_live_tracking_check)
    
def onAttachPlayerChange(self, context):
    setVal("tekken_attach_other_player", self.tekken_attach_other_player_check == 1)
    
    TK.attach_player = getVal("tekken_attach_other_player")
    print("onAttachPlayerChange", self.tekken_attach_other_player_check)
    
def onRangeLimitationToggle(self, context):
    setVal("tekken_range_limitation", self.tekken_range_limitation_check == 1)
    
    TK.setRangeLimitation(not getVal("tekken_range_limitation"))
    print("onRangeLimitationToggle", self.tekken_range_limitation_check)
    
def onCameraPreviewChange(self, context):
    setVal("tekken_camera_preview", self.tekken_camera_preview_check == 1)
    
    TK.setCameraPreview(getVal("tekken_camera_preview"))
    print("onCameraPreviewChange", self.tekken_camera_preview_check)
    
def onCameraTrackChange(self, context):
    setVal("tekken_camera_track", self.tekken_camera_track_check == 1)
    
    TK.setCameraTracking(getVal("tekken_camera_track"))
    print("onCameraTrackChange", self.tekken_camera_track_check)
    
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
        "var_name": "tekken_range_limitation_check",
        "name": "Disable range limitation",
        "default": False,
        "callback": onRangeLimitationToggle
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
        "callback": onCameraTrackChange
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
        
            if getVal("tekken_armature") != None:
                try: bpy.context.scene.objects[getVal("tekken_armature")].show_name = False
                except: pass
                
            setVal("tekken_armature", context.active_object.name)
            TK.setActiveSkeleton(0, context.active_object.name)
            context.active_object.show_name = True
            self.report({'INFO'}, "New armature selected")
        else:
            setVal("tekken_armature", None)
            self.report({'INFO'}, "(1p) Not a valid armature")
            TK.armature_name = None
            
        return {'FINISHED'}

# Sets 2P's active skeleton
class Set2pActiveSkeletonBtn(bpy.types.Operator):
    bl_idname = "tekken.set_skeleton_2p"
    bl_label = "2P - Set active skeleton"
    
    def execute(self, context):
        if context.object.type == 'ARMATURE':
        
            if getVal("tekken_armature_2p") != None:
                try: bpy.context.scene.objects[getVal("tekken_armature_2p")].show_name = False
                except: pass
                
            setVal("tekken_armature_2p", context.active_object.name)
            TK.setActiveSkeleton(1, context.active_object.name)
            context.active_object.show_name = True
            self.report({'INFO'}, "New 2P armature selected")
        else:
            setVal("tekken_armature_2p", None)
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
            
            setVal("tekken_camera", context.active_object.name)
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
        l.prop(context.scene, "tekken_range_limitation_check")
        
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
        update = onPlayerCollisionChange
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