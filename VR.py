import openvr
import numpy as np

def get_controller_ids(vrsys=None):
    if vrsys is None:
        vrsys = openvr.VRSystem()
    else:
        vrsys = vrsys
    left = None
    right = None
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vrsys.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_Controller:
            role = vrsys.getControllerRoleForTrackedDeviceIndex(i)
            if role == openvr.TrackedControllerRole_RightHand:
                right = i
            if role == openvr.TrackedControllerRole_LeftHand:
                left = i
    return left, right
    
def getPosFromMat(mat):
    A = np.matrix(
        ((mat[0][0], mat[0][1], mat[0][2], mat[0][3]),
         (mat[1][0], mat[1][1], mat[1][2], mat[1][3]),
         (mat[2][0], mat[2][1], mat[2][2], mat[2][3]),)
    , np.float32)
    return [float(f) for f in A.dot(np.matrix('0; 0; 0; 1'))]

class VRIntegrator:
    def __init__(self):
        self.hmd_id = None
        self.left_id = None
        self.right_id = None
    
    def start(self):
        openvr.init(openvr.VRApplication_Scene)
        self.getControllersIds()
        
    def setIKFromVR(self, armature):
        p = self.getControllerPoses()
        
        hmd_x, hmd_y, hmd_z = p['hmd']
        left_x, left_y, left_z = p['left']
        right_x, right_y, right_z = p['right']
        
        #print(left_x, left_y, left_z)
        
        hmd_x = -hmd_x
        hmd_y -= 1.6
        hmd_z = hmd_z
        
        left_x = (left_x) + 0.65
        left_y = 1.6 - left_y
        left_z = (-left_z) 
        
        right_x = (right_x) - 0.65
        right_y = 1.6 - right_y
        right_z = (-right_z) 
        
        armature.pose.bones["L_Hand2"].location = left_x, left_y, left_z
        #armature.pose.bones["L_Hand2"].rotation_euler = 0, 0, 0
        
        armature.pose.bones["HeadIK"].location = hmd_x, hmd_y, -hmd_z
        #armature.pose.bones["HeadIK"].rotation_euler = 0, 0, 0
        
        armature.pose.bones["R_Hand2"].location = right_x, right_y, right_z
        #armature.pose.bones["R_Hand2"].rotation_euler = 0, 0, 0
        
        #armature.pose.bones["BODY_SCALE__group"].location = 0, 0, 0

    def getControllerPoses(self):
        poses = []
        poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)
        return {
            'hmd': getPosFromMat(poses[self.hmd_id].mDeviceToAbsoluteTracking),
            'left': getPosFromMat(poses[self.left_id].mDeviceToAbsoluteTracking),
            'right': getPosFromMat(poses[self.right_id].mDeviceToAbsoluteTracking)
        }

    def getControllersIds(self):
        self.hmd_id = openvr.k_unTrackedDeviceIndex_Hmd
        self.left_id, self.right_id = get_controller_ids()
        