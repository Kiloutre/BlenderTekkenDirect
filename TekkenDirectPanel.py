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
    "tekken_playerid": 0,
    "tekken_armature": None,
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
    
    TK.setTarget(getVal("tekken_playerid"))
    TK.armature = getVal("tekken_armature")
    TK.setPreview(getVal("tekken_live_preview"))
    
    print("onLivePreviewChange", self.tekken_live_preview_check)
    
def onLiveTrackingChange(self, context):
    setVal("tekken_live_tracking", self.tekken_live_tracking_check == 1)
    
    if getVal("tekken_live_tracking") and getVal("tekken_live_preview"):
        setVal("tekken_live_preview", False)
        setVal("tekken_live_preview_check", False)
        
    TK.setTarget(getVal("tekken_playerid"))
    TK.armature = getVal("tekken_armature")
    TK.setTracking(getVal("tekken_live_tracking"))
    
    print("onLiveTrackingChange", self.tekken_live_tracking_check)
    
def onAttachPlayerChange(self, context):
    setVal("tekken_attach_other_player", self.tekken_attach_other_player_check == 1)
    
    TK.attach_player = getVal("tekken_attach_other_player")
    
    print("onAttachPlayerChange", self.tekken_attach_other_player_check)
    
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
        "default": True,
        "callback": onAttachPlayerChange
    },
    
]

# --- Functions --- #7

def init_variables():
    scene = bpy.context.scene
    
    for variableName in defaultVariables:
        scene[variableName] = defaultVariables[variableName]

def selectPlayer(context, playerid):
    setVal("tekken_playerid", playerid)
    TK.setTarget(getVal("tekken_playerid"))
    print("Selected player", playerid)
    
    
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

# Sets active skeleton
class SetActiveSkeletonBtn(bpy.types.Operator):
    bl_idname = "tekken.set_skeleton"
    bl_label = "Set active skeleton"
    
    def execute(self, context):
        if context.object.type == 'ARMATURE':
        
            if getVal("tekken_armature") != None:
                try: bpy.context.scene.objects[getVal("tekken_armature")].show_name = False
                except: pass
                
            setVal("tekken_armature", context.active_object.name)
            TK.armature_name = context.active_object.name
            context.active_object.show_name = True
            print("New armature selected")
        else:
            print("Invalid armature : not selecting")
        return {'FINISHED'}
    
# Sets 1P as target
class SelectFirstPlayerBtn(bpy.types.Operator):
    bl_idname = "tekken.p_select"
    bl_label = "1p"
    player_id = 0
    
    @classmethod
    def poll(cls, context):
        return getVal("tekken_playerid") != 0
    
    def execute(self, context):
        selectPlayer(context, self.player_id)
        return {'FINISHED'}
    
# Sets 2P as target
class SelectSecondPlayerBtn(bpy.types.Operator):
    bl_idname = "tekken.p2_select"
    bl_label = "2p"
    player_id = 1
    
    @classmethod
    def poll(cls, context):
        return getVal("tekken_playerid") != 1
    
    def execute(self, context):
        selectPlayer(context, self.player_id)
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
        #l.label(text='Live tracking (Game -> Blender)')
        #trackingRow = l.row()
        #trackingRow.operator(PLACEHOLDER_YES.bl_idname)
        #trackingRow.operator(PLACEHOLDER_NO.bl_idname)
        
        l.label(text='____________________________________')
        #l.separator(factor=2.0)
        
        # Live preview #
        
        l.prop(context.scene, "tekken_live_preview_check")
        #l.label(text='Live Preview (Blender -> Game)')
        #previewRow = l.row()
        #previewRow.operator(PLACEHOLDER_YES.bl_idname)
        #previewRow.operator(PLACEHOLDER_NO.bl_idname)
        
        #armatureFirstRow = l.row()
        l.label(text='Skeleton :')
        #armatureFirstRow.label(text='AAA')
        l.operator(SetActiveSkeletonBtn.bl_idname)
        
        l.label(text='Target :')
        targetRow = l.row()
        targetRow.operator(SelectFirstPlayerBtn.bl_idname)
        targetRow.operator(SelectSecondPlayerBtn.bl_idname)
        
        l.prop(context.scene, "tekken_attach_other_player_check")
        
        l.prop(context.scene, "p1_collision")
        
        
# --- Registering --- #

classes = [
    PLACEHOLDER_YES,
    PLACEHOLDER_NO,
    SelectFirstPlayerBtn,
    SelectSecondPlayerBtn,
    SetActiveSkeletonBtn,
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
        name = "P1 Collision",
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