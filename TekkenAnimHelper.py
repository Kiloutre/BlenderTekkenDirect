import struct
import bpy
import numpy as np
from math import pi, asin, atan2
from .characterFaces import getCharacterFacePos
from copy import deepcopy

halfpi = pi / 2

def quaternionToRotationMatrix(Q):
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix
    
def get_quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  return [qx, qy, qz, qw]

def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)
   
def getRotationFromMatrix(te, mode = 0):
    m11 = te[ 0 ][0]
    m12 = te[ 0 ][1]
    m13 = te[ 0 ][2]
    
    m21 = te[ 1 ][0]
    m22 = te[ 1 ][1]
    m23 = te[ 1 ][2]
    
    m31 = te[ 2 ][0]
    m32 = te[ 2 ][1]
    m33 = te[ 2 ][2]

    if mode == 0: #XYZ
        y = asin( clamp( m13, - 1, 1 ) )
        if abs( m13 ) < 0.9999999:
            x = atan2( - m23, m33 )
            z = atan2( - m12, m11 )
        else:
            x = atan2( m32, m22 )
            z = 0
            
    elif mode == 1: #YXZ
        x = asin( - clamp( m23, - 1, 1 ) )
        if abs( m23 ) < 0.9999999:
            y = atan2( m13, m33 )
            z = atan2( m21, m22 )
        else:
            y = atan2( - m31, m11 )
            z = 0
            
    elif mode == 2: #ZXY
        x = asin( clamp( m32, - 1, 1 ) )
        if abs( m32 ) < 0.9999999:
            y = atan2( - m31, m33 )
            z = atan2( - m12, m22 )
        else:
            y = 0
            z = atan2( m21, m11 )
            
    elif mode == 3: #ZYX
        y = asin( -  clamp( m31, - 1, 1 ) )
        if abs( m31 ) < 0.9999999:
            x = atan2( m32, m33 )
            z = atan2( m21, m11 )
        else:
            x = 0
            z = atan2( - m12, m22 )
            
    elif mode == 4: #YZX
        z = asin( clamp( m21, - 1, 1 ) )
        if abs( m21 ) < 0.9999999:
            x = atan2( - m23, m22 )
            y = atan2( - m31, m11 )
        else:
            x = 0
            y = atan2( m13, m33 )
            
    elif mode == 5: #XZY
        z = asin( - clamp( m12, - 1, 1 ) )
        if abs( m12 ) < 0.9999999:
            x = atan2( m32, m22 )
            y = atan2( m13, m11 )
        else:
            x = atan2( - m23, m33 )
            y = 0
        
    return x, y, z

# --------
    
class TekkenAnimation:
    def _getHeaderSizeFromArgs_(bone_count):
        return bone_count * 0x4 + 0x8
        
    def _getFramesizeFromArgs_(bone_count):
        return bone_count * 0xC

    def __init__(self, data=None, type="body"):
        if data == None:
            self.data = TekkenAnimation.createAnim(type)
        else:
            self.data = data
            
        self.type = self.byte(0)
        self.bone_count = self.byte(2)
        self.length = self.getLength()
        self.offset = self.getOffset()
        self.frame_size = self.getFramesize()
        self.field_count = int(self.frame_size / 4)
        self.recalculateSize() #Crop or add missing bytes if needed
        
    def createAnim(type="body"): 
        if type not in ["body", "hand", "face"]:
            raise KeyError(type + " is not an acceptable animation type. Try 'body', 'hand' or 'face'.")
            
        data = []
        bone_count = {
            "body": 0x17,
            "hand": 0x13,
            "face": 0x5A,
        }.get(type)
        
        data += [0xC8, 0] #anim type
        data += [bone_count, 0]
        data += [0x1, 0, 0, 0] #anim length: 1 frame
        
        if type == "body":
            for i in range(bone_count):
                boneInfo = 0x07
                
                if i in [0, 1, 6]:
                    boneInfo = 0x0B
                elif i == 2:
                    boneInfo = 0x05
                elif i in [11, 15, 18, 21]:
                    boneInfo = 0x06
                
                data += [boneInfo, 0, 0, 0]
        elif type == "hand":
            for i in range(bone_count):
                data += [0x07, 0, 0, 0]
        elif type == "face":
            for i in range(bone_count):
                boneInfo = 0xB
                
                if 0x57 <= i <= 0x5A:
                    boneInfo = 0x3
                elif i & 1 == 1:
                    boneInfo = 0x7
                
                data += [boneInfo, 0, 0, 0]
            
        return data

    def recalculateSize(self):
        pastSize = len(self.data)
        self.size = self.calculateSize()
        if self.size > pastSize:
            self.data += [0] * (self.size - pastSize)
        elif self.size < pastSize:
            self.data = self.data[:self.size]

    def calculateSize(self):
        return self.getOffset() + (self.getFramesize() * self.length)
        
    def getOffset(self):
        if self.type == 0xC8:
            return self.bone_count * 0x4 + 0x8
        
    def getFramesize(self):
        return self.bone_count * 0xC #= * 3 fields * 4 bytes each
        
    def setLength(self, length):
        self.length = length
        self.writeInt(length, 4)
        
    def getLength(self):
        return self.int(4)
    
    def bToInt(self, offset, length):
        return int.from_bytes(bytes(self.data[offset:offset+length]), 'little')
    
    def int(self, offset):
        return self.bToInt(offset, 4)
    
    def short(self, offset):
        return self.bToInt(offset, 2)
        
    def byte(self, offset):
        return self.bToInt(offset, 1)
        
    def float(self, offset):
        return struct.unpack('f', bytes(self.data[offset:offset + 4]))[0]
            
    def writeInt(self, value, offset):
        for i in range(4):
            byteValue = (value >> (i * 8)) & 0xFF
            self.data[offset + i] = byteValue
            
    def writeFloat(self, value, offset):
        byteData = struct.pack('f', value)
        for i in range(4):
            self.data[offset + i] = int(byteData[i])
        
    def getFieldOffset(self, frame, fieldId):
        if fieldId > self.field_count:
            raise
        return self.offset + (frame * self.frame_size) + (4 * fieldId)
        
    def getField(self, frame, fieldId):
        if fieldId > self.field_count:
            raise
        return self.float(self.getFieldOffset(frame, fieldId))
                
    def setField(self, value, frame, fieldId):
        if fieldId > self.field_count:
            raise
        self.writeFloat(value, self.offset + (frame * self.frame_size) + (4 * fieldId))
             
def __get_visual_rotations__(armature, bones):
    poses = {}
    for prefix in ["L_", "R_"]: # set default values
        for bone in ["UpLeg", "Leg", "Foot", "Hand", "ForeArm", "Arm", "Shoulder"]:
            b = prefix + bone
            #rot_source = bones[b].rotation_euler
            rot_source = getEulerVisualRotation(b, armature.name)
            poses[b] = { 'x': rot_source.x, 'y': rot_source.y, 'z': rot_source.z}
            
    #to be used if you have IK for those bones too
    """
    for b in ["BASE", "Hip", "Head", "Neck", "Spine1"]:
        #rot_source = bones[b].rotation_euler
        rot_source = getEulerVisualRotation(b, armature.name)
        poses[b] = { 'x': rot_source.x, 'y': rot_source.y, 'z': rot_source.z}
    """
    return poses
    
    
def getEulerVisualRotation(boneName, armatureName):
    bone        = bpy.data.armatures[armatureName].bones[boneName]
    bone_ml     = bone.matrix_local
    bone_pose   = bpy.data.objects[armatureName].pose.bones[boneName]
    bone_pose_m = bone_pose.matrix
    
    if bone.parent:
        #
        parent        = bone.parent
        parent_ml     = parent.matrix_local
        parent_pose   = bone_pose.parent
        parent_pose_m = parent_pose.matrix
        
        object_diff = parent_ml.inverted() @ bone_ml
        pose_diff   = parent_pose_m.inverted() @ bone_pose_m
        local_diff  = object_diff.inverted() @ pose_diff
        
    else:
        local_diff = bone_ml.inverted() @ bone_pose_m
    
    return local_diff.to_quaternion().to_euler(bone_pose.rotation_mode)
    
# expects x y z blender rotation, outputs tekken rotation
def convertArmToTekkenXYZ(x, y, z):
    x, y, z = -x, -z, y
    
    orig_quat = get_quaternion_from_euler(halfpi, 0, halfpi)
    orig_mat = quaternionToRotationMatrix(orig_quat)
    orig_mat = np.linalg.inv(orig_mat)

    quat = get_quaternion_from_euler(x, y, z)
    mat = quaternionToRotationMatrix(quat)

    mat = np.matmul(mat, orig_mat)
    
    x, y, z = getRotationFromMatrix(mat, mode=3)
    
    return x, -y, -z

# expects x y z tekken rotation, outputs blender rotation
def convertArmToBlenderXYZ(x, y, z):
    orig_quat = get_quaternion_from_euler(-halfpi, halfpi, 0)

    orig_mat = quaternionToRotationMatrix(orig_quat)
    orig_mat = np.linalg.inv(orig_mat)

    quat = get_quaternion_from_euler(x, y, z)
    mat = quaternionToRotationMatrix(quat)

    mat = np.matmul(mat, orig_mat)
    
    x, y, z = getRotationFromMatrix(mat, mode=1)
    
    return -z, y, -x

# expects x y z Blender rotation, outputs unreal rotation
def convertCameraToUnrealRot(x, y, z):
    x, y, z = -x, -z, y
    
    orig_quat = get_quaternion_from_euler(-halfpi, 0, -halfpi)

    orig_mat = quaternionToRotationMatrix(orig_quat)
    orig_mat = np.linalg.inv(orig_mat)

    quat = get_quaternion_from_euler(x, y, z)
    mat = quaternionToRotationMatrix(quat)

    mat = np.matmul(mat, orig_mat)
    
    x, y, z = getRotationFromMatrix(mat, mode=1)
    
    return x, y, z
    
# expects x y z Unreal rotation, outputs blender rotation
def convertCameraToBlenderRot(x, y, z):
    x, y, z = y, x, z
    
    orig_quat = get_quaternion_from_euler(1.57, 0, 1.57)

    orig_mat = quaternionToRotationMatrix(orig_quat)

    quat = get_quaternion_from_euler(x, y, z)
    mat = quaternionToRotationMatrix(quat)

    mat = np.matmul(mat, orig_mat)
    
    x, y, z = getRotationFromMatrix(mat, mode=5)
    
    return -x + pi, -z, y
             
def getAnimFrameFromBones(armature):
    bones = armature.pose.bones
    
    visualRots = __get_visual_rotations__(armature, bones)

    offset1 = -(bones["BODY_SCALE__group"].location[2] * 1000)
    offset2 = (bones["BODY_SCALE__group"].location[1]) * 1000 + 1150
    offset3 = bones["BODY_SCALE__group"].location[0] * 1000
    RotX = bones["BASE"].rotation_euler.z * -1
    RotY = bones["BASE"].rotation_euler.y
    RotZ = bones["BASE"].rotation_euler.x 
    
    UpperBody1 = bones["Spine1"].rotation_euler.z * -1
    UpperBody2 = bones["Spine1"].rotation_euler.y
    UpperBody3 = bones["Spine1"].rotation_euler.x + (halfpi)
    
    LowerBody1 = bones["Hip"].rotation_euler.z * -1
    LowerBody2 = bones["Hip"].rotation_euler.y
    LowerBody3 = bones["Hip"].rotation_euler.x - (halfpi)
    
    Neck1 = bones["Neck"].rotation_euler.z * -1
    Neck2 = bones["Neck"].rotation_euler.y
    Neck3 = bones["Neck"].rotation_euler.x
    
    Head1 = bones["Head"].rotation_euler.z + (halfpi)
    Head2 = bones["Head"].rotation_euler.x
    Head3 = bones["Head"].rotation_euler.y + (halfpi)
    
    #to be used if you have IK for those bones too
    """
    RotX = visualRots["BASE"]['z'] * -1
    RotY = visualRots["BASE"]['y']
    RotZ = visualRots["BASE"]['x'] 
    
    UpperBody1 = visualRots["Spine1"]['z'] * -1
    UpperBody2 = visualRots["Spine1"]['y']
    UpperBody3 = visualRots["Spine1"]['x'] + (halfpi)
    
    LowerBody1 = visualRots["Hip"]['z'] * -1
    LowerBody2 = visualRots["Hip"]['y']
    LowerBody3 = visualRots["Hip"]['x'] - (halfpi)
    
    Neck1 = visualRots["Neck"]['z'] * -1
    Neck2 = visualRots["Neck"]['y']
    Neck3 = visualRots["Neck"]['x']
    
    Head1 = visualRots["Head"]['z'] + (halfpi)
    Head2 = visualRots["Head"]['x']
    Head3 = visualRots["Head"]['y'] + (halfpi)
    """
    
    # --------- SHOULDER ------------
    
    #x, y, z = bones["R_Shoulder"].rotation_euler
    x, y, z = visualRots["R_Shoulder"]['x'], visualRots["R_Shoulder"]['y'], visualRots["R_Shoulder"]['z']
    x, y, z = convertArmToTekkenXYZ(x, y, z)
    RightInnerShoulder1 = x + pi
    RightInnerShoulder2 = y
    RightInnerShoulder3 = z
    
    # --------- ARM IK -----------
    
    RightOuterShoulder1 = visualRots["R_Arm"]['x'] * -1 - halfpi
    RightOuterShoulder2 = visualRots["R_Arm"]['z'] * -1
    RightOuterShoulder3 = visualRots["R_Arm"]['y'] * -1
    
    RightElbow1 = visualRots["R_ForeArm"]['x'] * -1
    RightElbow2 = visualRots["R_ForeArm"]['y']
    RightElbow3 = visualRots["R_ForeArm"]['z'] * -1
    
    RightHand1 = visualRots["R_Hand"]['x'] + halfpi
    RightHand2 = visualRots["R_Hand"]['z'] * -1
    RightHand3 = visualRots["R_Hand"]['y']
    
    # --------- SHOULDER ------------
    
    # x, y, z = bones["L_Shoulder"].rotation_euler
    x, y, z = visualRots["L_Shoulder"]['x'], visualRots["L_Shoulder"]['y'], visualRots["L_Shoulder"]['z']
    x, y, z = convertArmToTekkenXYZ(x, y, z)
    
    LeftInnerShoulder1 = x + pi
    LeftInnerShoulder2 = y + pi
    LeftInnerShoulder3 = z * -1
    
    # --------- ARM IK -----------
    
    LeftOuterShoulder1 = visualRots["L_Arm"]['x'] - halfpi
    LeftOuterShoulder2 = visualRots["L_Arm"]['z']
    LeftOuterShoulder3 = visualRots["L_Arm"]['y'] * -1
    
    LeftElbow1 = visualRots["L_ForeArm"]['x']
    LeftElbow2 = visualRots["L_ForeArm"]['y']
    LeftElbow3 = visualRots["L_ForeArm"]['z']
    
    LeftHand1 = visualRots["L_Hand"]['x'] * -1 + halfpi
    LeftHand2 = visualRots["L_Hand"]['z'] * -1
    LeftHand3 = visualRots["L_Hand"]['y'] * -1

    
    #---------------- LEG IK -----------------
    
    RightHip1 = visualRots["R_UpLeg"]['z'] * -1
    RightHip2 = visualRots["R_UpLeg"]['y']
    RightHip3 = visualRots["R_UpLeg"]['x']
    
    RightKnee1 = visualRots["R_Leg"]['z'] * -1
    RightKnee2 = visualRots["R_Leg"]['y']
    RightKnee3 = visualRots["R_Leg"]['x']

    RightFoot1 = visualRots["R_Foot"]['z'] * -1
    RightFoot2 = visualRots["R_Foot"]['y']
    RightFoot3 = visualRots["R_Foot"]['x']
    
    
    LeftHip1 = visualRots["L_UpLeg"]['z'] * -1
    LeftHip2 = visualRots["L_UpLeg"]['y']
    LeftHip3 = visualRots["L_UpLeg"]['x']
    
    LeftKnee1 = visualRots["L_Leg"]['z'] * -1
    LeftKnee2 = visualRots["L_Leg"]['y']
    LeftKnee3 = visualRots["L_Leg"]['x']

    LeftFoot1 = visualRots["L_Foot"]['z'] * -1
    LeftFoot2 = visualRots["L_Foot"]['y']
    LeftFoot3 = visualRots["L_Foot"]['x']
    
    # ------ end --------
    
    _DEF_VAL_TEST = 0 #default value for stuff we don't know or care about
    
    return [
        _DEF_VAL_TEST, #Offset = movement x
        _DEF_VAL_TEST, #Offset = height
        _DEF_VAL_TEST, #Offset = movement z
        offset1, #JumpStrength = pos x
        offset2, #JumpStrength = pos y
        offset3, #JumpStrength = pos Z
        _DEF_VAL_TEST, #Unknown = field 7
        _DEF_VAL_TEST, #Unknown = field 8
        _DEF_VAL_TEST, #Unknown = field 9
        RotX, #Mesh = rotx
        RotY, #Mesh = roty
        RotZ, #Mesh = rotz
        UpperBody1, # = spine1 x
        UpperBody2, # = spine1 x
        UpperBody3, # = spine1 z
        LowerBody1, # = hip x
        LowerBody2, # = hip y
        LowerBody3, # = hip z
        _DEF_VAL_TEST, #SpineFlexure # = spine 2
        _DEF_VAL_TEST, #SpineFlexure # = field 20
        _DEF_VAL_TEST, #SpineFlexure # = field 21
        Neck1, # = neck 22
        Neck2, # = neck 23
        Neck3, # = neck 24
        Head1, # = neck 25
        Head2, # = neck 26
        Head3, # = neck 27
        RightInnerShoulder1, #RightInnerShoulder
        RightInnerShoulder2, #RightInnerShoulder
        RightInnerShoulder3, #RightInnerShoulder
        RightOuterShoulder1, #RightOuterShoulder
        RightOuterShoulder2, #RightOuterShoulder
        RightOuterShoulder3, #RightOuterShoulder
        RightElbow1, #RightElbow
        RightElbow2, #RightElbow
        RightElbow3, #RightElbow
        RightHand1, #RightHand
        RightHand2, #RightHand
        RightHand3, #RightHand
        LeftInnerShoulder1, #LeftInnerShoulder
        LeftInnerShoulder2, #LeftInnerShoulder
        LeftInnerShoulder3, #LeftInnerShoulder
        LeftOuterShoulder1, #LeftOuterShoulder
        LeftOuterShoulder2, #LeftOuterShoulder
        LeftOuterShoulder3, #LeftOuterShoulder
        LeftElbow1, #LeftElbow
        LeftElbow2, #LeftElbow
        LeftElbow3, #LeftElbow
        LeftHand1, #LeftHand
        LeftHand2, #LeftHand
        LeftHand3, #LeftHand
        RightHip1, #RightHip
        RightHip2, #RightHip
        RightHip3, #RightHip
        RightKnee1, #RightKnee
        RightKnee2, #RightKnee
        RightKnee3, #RightKnee
        RightFoot1, #RightFoot
        RightFoot2, #RightFoot
        RightFoot3, #RightFoot
        LeftHip1, #LeftHip
        LeftHip2, #LeftHip
        LeftHip3, #LeftHip
        LeftKnee1, #LeftKnee
        LeftKnee2, #LeftKnee
        LeftKnee3, #LeftKnee
        LeftFoot1, #LeftFoot
        LeftFoot2, #LeftFoot
        LeftFoot3, #LeftFoot
    ]

def getHandAnimFrameFromBones(armature):
    return getLeftHandAnimFrameFromBones(armature), getRightHandAnimFrameFromBones(armature)

def getLeftHandAnimFrameFromBones(armature):
    bones = armature.pose.bones
    
    FingerBase_x = bones["L_FingerBase"].rotation_euler.x * -1
    FingerBase_y = bones["L_FingerBase"].rotation_euler.y
    FingerBase_z = bones["L_FingerBase"].rotation_euler.z * -1
    
    Thumb1_x = bones["L_Thumb1"].rotation_euler.x * -1
    Thumb1_y = bones["L_Thumb1"].rotation_euler.z * -1
    Thumb1_z = bones["L_Thumb1"].rotation_euler.y * -1
    
    Thumb2_x = bones["L_Thumb2"].rotation_euler.x * -1
    Thumb2_y = bones["L_Thumb2"].rotation_euler.z * -1
    Thumb2_z = bones["L_Thumb2"].rotation_euler.y * -1
    
    Thumb3_x = bones["L_Thumb3"].rotation_euler.x * -1
    Thumb3_y = bones["L_Thumb3"].rotation_euler.z * -1
    Thumb3_z = bones["L_Thumb3"].rotation_euler.y * -1
    
    Index1_x = bones["L_Index1"].rotation_euler.x * -1
    Index1_y = bones["L_Index1"].rotation_euler.y
    Index1_z = bones["L_Index1"].rotation_euler.z * -1
    
    Index2_x = bones["L_Index2"].rotation_euler.x * -1
    Index2_y = bones["L_Index2"].rotation_euler.y
    Index2_z = bones["L_Index2"].rotation_euler.z * -1
    
    Index3_x = bones["L_Index3"].rotation_euler.x * -1
    Index3_y = bones["L_Index3"].rotation_euler.y
    Index3_z = bones["L_Index3"].rotation_euler.z * -1
    
    Middle_x = bones["L_Middle"].rotation_euler.x * -1
    Middle_y = bones["L_Middle"].rotation_euler.y
    Middle_z = bones["L_Middle"].rotation_euler.z * -1
    
    Middle1_x = bones["L_Middle1"].rotation_euler.x * -1
    Middle1_y = bones["L_Middle1"].rotation_euler.y
    Middle1_z = bones["L_Middle1"].rotation_euler.z * -1
    
    Middle2_x = bones["L_Middle2"].rotation_euler.x * -1
    Middle2_y = bones["L_Middle2"].rotation_euler.y
    Middle2_z = bones["L_Middle2"].rotation_euler.z * -1
    
    Middle3_x = bones["L_Middle3"].rotation_euler.x * -1
    Middle3_y = bones["L_Middle3"].rotation_euler.y
    Middle3_z = bones["L_Middle3"].rotation_euler.z * -1
    
    Ring_x = bones["L_Ring"].rotation_euler.x * -1
    Ring_y = bones["L_Ring"].rotation_euler.y
    Ring_z = bones["L_Ring"].rotation_euler.z * -1
    
    Ring1_x = bones["L_Ring1"].rotation_euler.x * -1
    Ring1_y = bones["L_Ring1"].rotation_euler.y
    Ring1_z = bones["L_Ring1"].rotation_euler.z * -1
    
    Ring2_x = bones["L_Ring2"].rotation_euler.x * -1
    Ring2_y = bones["L_Ring2"].rotation_euler.y
    Ring2_z = bones["L_Ring2"].rotation_euler.z * -1
    
    Ring3_x = bones["L_Ring3"].rotation_euler.x * -1
    Ring3_y = bones["L_Ring3"].rotation_euler.y
    Ring3_z = bones["L_Ring3"].rotation_euler.z * -1
    
    Pinky_x = bones["L_Pinky"].rotation_euler.x * -1
    Pinky_y = bones["L_Pinky"].rotation_euler.y
    Pinky_z = bones["L_Pinky"].rotation_euler.z * -1
    
    Pinky1_x = bones["L_Pinky1"].rotation_euler.x * -1
    Pinky1_y = bones["L_Pinky1"].rotation_euler.y
    Pinky1_z = bones["L_Pinky1"].rotation_euler.z * -1
    
    Pinky2_x = bones["L_Pinky2"].rotation_euler.x * -1
    Pinky2_y = bones["L_Pinky2"].rotation_euler.y
    Pinky2_z = bones["L_Pinky2"].rotation_euler.z * -1
    
    Pinky3_x = bones["L_Pinky3"].rotation_euler.x * -1
    Pinky3_y = bones["L_Pinky3"].rotation_euler.y
    Pinky3_z = bones["L_Pinky3"].rotation_euler.z * -1
    
    return [
        FingerBase_x, #L_FingerBase
        FingerBase_y,
        FingerBase_z, 
        Thumb1_x, #L_Thumb1
        Thumb1_y,
        Thumb1_z, 
        Thumb2_x, #L_Thumb2
        Thumb2_y,
        Thumb2_z, 
        Thumb3_x, #L_Thumb3
        Thumb3_y,
        Thumb3_z, 
        Index1_x, #L_Index1
        Index1_y,
        Index1_z, 
        Index2_x, #L_Index2
        Index2_y,
        Index2_z, 
        Index3_x, #L_Index3
        Index3_y,
        Index3_z,
        Middle_x, #Middle
        Middle_y,
        Middle_z,
        Middle1_x, #Middle1
        Middle1_y,
        Middle1_z,
        Middle2_x, #Middle2
        Middle2_y,
        Middle2_z,
        Middle3_x, #Middle3
        Middle3_y,
        Middle3_z,
        Ring_x, #Ring
        Ring_y,
        Ring_z,
        Ring1_x, #Ring1
        Ring1_y,
        Ring1_z,
        Ring2_x, #Ring2
        Ring2_y,
        Ring2_z,
        Ring3_x, #Ring3
        Ring3_y,
        Ring3_z,
        Pinky_x, #Pinky
        Pinky_y,
        Pinky_z,
        Pinky1_x, #Pinky1
        Pinky1_y,
        Pinky1_z,
        Pinky2_x, #Pinky2
        Pinky2_y,
        Pinky2_z,
        Pinky3_x, #Pinky3
        Pinky3_y,
        Pinky3_z
    ]


def getRightHandAnimFrameFromBones(armature):
    bones = armature.pose.bones
    
    FingerBase_x = bones["R_FingerBase"].rotation_euler.x
    FingerBase_y = bones["R_FingerBase"].rotation_euler.y
    FingerBase_z = bones["R_FingerBase"].rotation_euler.z
    
    Thumb1_x = bones["R_Thumb1"].rotation_euler.x
    Thumb1_y = bones["R_Thumb1"].rotation_euler.z
    Thumb1_z = bones["R_Thumb1"].rotation_euler.y * -1
    
    Thumb2_x = bones["R_Thumb2"].rotation_euler.x
    Thumb2_y = bones["R_Thumb2"].rotation_euler.z
    Thumb2_z = bones["R_Thumb2"].rotation_euler.y * -1
    
    Thumb3_x = bones["R_Thumb3"].rotation_euler.x
    Thumb3_y = bones["R_Thumb3"].rotation_euler.z
    Thumb3_z = bones["R_Thumb3"].rotation_euler.y * -1
    
    Index1_x = bones["R_Index1"].rotation_euler.x
    Index1_y = bones["R_Index1"].rotation_euler.y
    Index1_z = bones["R_Index1"].rotation_euler.z
    
    Index2_x = bones["R_Index2"].rotation_euler.x
    Index2_y = bones["R_Index2"].rotation_euler.y
    Index2_z = bones["R_Index2"].rotation_euler.z
    
    Index3_x = bones["R_Index3"].rotation_euler.x
    Index3_y = bones["R_Index3"].rotation_euler.y
    Index3_z = bones["R_Index3"].rotation_euler.z * -1
    
    Middle_x = bones["R_Middle"].rotation_euler.x
    Middle_y = bones["R_Middle"].rotation_euler.y
    Middle_z = bones["R_Middle"].rotation_euler.z
    
    Middle1_x = bones["R_Middle1"].rotation_euler.x * -1
    Middle1_y = bones["R_Middle1"].rotation_euler.y
    Middle1_z = bones["R_Middle1"].rotation_euler.z * -1
    
    Middle2_x = bones["R_Middle2"].rotation_euler.x
    Middle2_y = bones["R_Middle2"].rotation_euler.y
    Middle2_z = bones["R_Middle2"].rotation_euler.z
    
    Middle3_x = bones["R_Middle3"].rotation_euler.x * -1
    Middle3_y = bones["R_Middle3"].rotation_euler.y
    Middle3_z = bones["R_Middle3"].rotation_euler.z * -1
    
    Ring_x = bones["R_Ring"].rotation_euler.x
    Ring_y = bones["R_Ring"].rotation_euler.y
    Ring_z = bones["R_Ring"].rotation_euler.z
    
    Ring1_x = bones["R_Ring1"].rotation_euler.x
    Ring1_y = bones["R_Ring1"].rotation_euler.y
    Ring1_z = bones["R_Ring1"].rotation_euler.z
    
    Ring2_x = bones["R_Ring2"].rotation_euler.x
    Ring2_y = bones["R_Ring2"].rotation_euler.y
    Ring2_z = bones["R_Ring2"].rotation_euler.z
    
    Ring3_x = bones["R_Ring3"].rotation_euler.x
    Ring3_y = bones["R_Ring3"].rotation_euler.y
    Ring3_z = bones["R_Ring3"].rotation_euler.z
    
    Pinky_x = bones["R_Pinky"].rotation_euler.x
    Pinky_y = bones["R_Pinky"].rotation_euler.y
    Pinky_z = bones["R_Pinky"].rotation_euler.z
    
    Pinky1_x = bones["R_Pinky1"].rotation_euler.x
    Pinky1_y = bones["R_Pinky1"].rotation_euler.y
    Pinky1_z = bones["R_Pinky1"].rotation_euler.z
    
    Pinky2_x = bones["R_Pinky2"].rotation_euler.x
    Pinky2_y = bones["R_Pinky2"].rotation_euler.y
    Pinky2_z = bones["R_Pinky2"].rotation_euler.z
    
    Pinky3_x = bones["R_Pinky3"].rotation_euler.x
    Pinky3_y = bones["R_Pinky3"].rotation_euler.y
    Pinky3_z = bones["R_Pinky3"].rotation_euler.z
    
    return [
        FingerBase_x, #R_FingerBase
        FingerBase_y,
        FingerBase_z, 
        Thumb1_x, #Thumb1
        Thumb1_y,
        Thumb1_z, 
        Thumb2_x, #Thumb2
        Thumb2_y,
        Thumb2_z, 
        Thumb3_x, #Thumb3
        Thumb3_y,
        Thumb3_z, 
        Index1_x, #Index1
        Index1_y,
        Index1_z, 
        Index2_x, #Index2
        Index2_y,
        Index2_z, 
        Index3_x, #Index3
        Index3_y,
        Index3_z,
        Middle_x, #Middle
        Middle_y,
        Middle_z,
        Middle1_x, #Middle1
        Middle1_y,
        Middle1_z,
        Middle2_x, #Middle2
        Middle2_y,
        Middle2_z,
        Middle3_x, #Middle3
        Middle3_y,
        Middle3_z,
        Ring_x, #Ring
        Ring_y,
        Ring_z,
        Ring1_x, #Ring1
        Ring1_y,
        Ring1_z,
        Ring2_x, #Ring2
        Ring2_y,
        Ring2_z,
        Ring3_x, #Ring3
        Ring3_y,
        Ring3_z,
        Pinky_x, #Pinky
        Pinky_y,
        Pinky_z,
        Pinky1_x, #Pinky1
        Pinky1_y,
        Pinky1_z,
        Pinky2_x, #Pinky2
        Pinky2_y,
        Pinky2_z,
        Pinky3_x, #Pinky3
        Pinky3_y,
        Pinky3_z
    ]
    
face_bonesTransformTypes = {
    #zxy
    
    "Bero1_Joint": 1, #-zyx
    "Bero1_Joint_pos": 1,
    "Bero2_Joint": 1,
    "Bero3_Joint": 1,
    
    "M_LipD_Joint": -1, #xyz
    "L_LipD_Joint": -1,
    "R_LipD_Joint": -1,
    "L_LipE_Joint": -1,
    "R_LipE_Joint": -1,
}
face_bonesRotationTypes = {
    "Jaw_Joint": 1
}
    
def getBonePos(boneName, location, defaultBonePos):
    x, y, z = location
    
    type = face_bonesTransformTypes.get(boneName, 0)
    if type == 0:
        x, y, z = z, x, y
    elif type == 1:
        x, y, z = -z, y, x
        
    x = x * 1000 + defaultBonePos['x']
    y = y * 1000 + defaultBonePos['y']
    z = z * 1000 + defaultBonePos['z']
    
    return (x, y, z)
    
def getBoneRot(boneName, rotation, defaultBonePos):
    x, y, z = rotation
    
    type = face_bonesRotationTypes.get(boneName, 0)
    if type == 1:
        x, y, z = x, z * -1, y
    
    return (x + defaultBonePos['rot_x'],
            y + defaultBonePos['rot_y'],
            z + defaultBonePos['rot_z'])

def getFaceAnimFrameFromBones(armature, face_base_pose, characterId):

    bones = armature.pose.bones
    correctedBones = deepcopy(face_base_pose)
    
    for boneName in face_base_pose:
        x, y, z = getBonePos(boneName, bones[boneName].location, face_base_pose[boneName])
        correctedBones[boneName]['x'] = x
        correctedBones[boneName]['y'] = y
        correctedBones[boneName]['z'] = z
        
        x, y, z = getBoneRot(boneName, bones[boneName].rotation_euler, face_base_pose[boneName])
        correctedBones[boneName]['rot_x'] = x
        correctedBones[boneName]['rot_y'] = y
        correctedBones[boneName]['rot_z'] = z


    Bero2_Joint_x = bones["Bero2_Joint"].location.x * 1000
    Bero2_Joint_y = bones["Bero2_Joint"].location.y * 1000
    Bero2_Joint_z = bones["Bero2_Joint"].location.z * 1000

    Bero2_Joint_rot_x = bones["Bero2_Joint"].rotation_euler.x
    Bero2_Joint_rot_y = bones["Bero2_Joint"].rotation_euler.y
    Bero2_Joint_rot_z = bones["Bero2_Joint"].rotation_euler.z

    Bero2_Joint_scale_x = bones["Bero2_Joint"].scale.x
    Bero2_Joint_scale_y = bones["Bero2_Joint"].scale.y
    Bero2_Joint_scale_z = bones["Bero2_Joint"].scale.z


    Bero3_Joint_x = bones["Bero3_Joint"].location.x * 1000 + 18.521000
    Bero3_Joint_y = bones["Bero3_Joint"].location.y * 1000 + 0.000100
    Bero3_Joint_z = bones["Bero3_Joint"].location.z * 1000 + -0.000200

    Bero3_Joint_rot_x = bones["Bero3_Joint"].rotation_euler.x
    Bero3_Joint_rot_y = bones["Bero3_Joint"].rotation_euler.y
    Bero3_Joint_rot_z = bones["Bero3_Joint"].rotation_euler.z
    
    return [
        correctedBones["L_Mayu1_Joint"]["x"], #L_Mayu1_Joint
        correctedBones["L_Mayu1_Joint"]["y"],
        correctedBones["L_Mayu1_Joint"]["z"],
        correctedBones["L_Mayu1_Joint"]["rot_x"],
        correctedBones["L_Mayu1_Joint"]["rot_y"],
        correctedBones["L_Mayu1_Joint"]["rot_z"],

        correctedBones["L_Mayu2_Joint"]["x"], #L_Mayu2_Joint
        correctedBones["L_Mayu2_Joint"]["y"],
        correctedBones["L_Mayu2_Joint"]["z"],
        correctedBones["L_Mayu2_Joint"]["rot_x"],
        correctedBones["L_Mayu2_Joint"]["rot_y"],
        correctedBones["L_Mayu2_Joint"]["rot_z"],

        correctedBones["L_Mayu3_Joint"]["x"], #L_Mayu3_Joint
        correctedBones["L_Mayu3_Joint"]["y"],
        correctedBones["L_Mayu3_Joint"]["z"],
        correctedBones["L_Mayu3_Joint"]["rot_x"],
        correctedBones["L_Mayu3_Joint"]["rot_y"],
        correctedBones["L_Mayu3_Joint"]["rot_z"],

        correctedBones["R_Mayu1_Joint"]["x"], #R_Mayu1_Joint
        correctedBones["R_Mayu1_Joint"]["y"],
        correctedBones["R_Mayu1_Joint"]["z"],
        correctedBones["R_Mayu1_Joint"]["rot_x"],
        correctedBones["R_Mayu1_Joint"]["rot_y"],
        correctedBones["R_Mayu1_Joint"]["rot_z"],

        correctedBones["R_Mayu2_Joint"]["x"], #R_Mayu2_Joint
        correctedBones["R_Mayu2_Joint"]["y"],
        correctedBones["R_Mayu2_Joint"]["z"],
        correctedBones["R_Mayu2_Joint"]["rot_x"],
        correctedBones["R_Mayu2_Joint"]["rot_y"],
        correctedBones["R_Mayu2_Joint"]["rot_z"],

        correctedBones["R_Mayu3_Joint"]["x"], #R_Mayu3_Joint
        correctedBones["R_Mayu3_Joint"]["y"],
        correctedBones["R_Mayu3_Joint"]["z"],
        correctedBones["R_Mayu3_Joint"]["rot_x"],
        correctedBones["R_Mayu3_Joint"]["rot_y"],
        correctedBones["R_Mayu3_Joint"]["rot_z"],

        correctedBones["L_Eyecorner"]["x"], #L_Eyecorner
        correctedBones["L_Eyecorner"]["y"],
        correctedBones["L_Eyecorner"]["z"],
        correctedBones["L_Eyecorner"]["rot_x"],
        correctedBones["L_Eyecorner"]["rot_y"],
        correctedBones["L_Eyecorner"]["rot_z"],

        correctedBones["R_Eyecorner"]["x"], #R_Eyecorner
        correctedBones["R_Eyecorner"]["y"],
        correctedBones["R_Eyecorner"]["z"],
        correctedBones["R_Eyecorner"]["rot_x"],
        correctedBones["R_Eyecorner"]["rot_y"],
        correctedBones["R_Eyecorner"]["rot_z"],

        correctedBones["L_UwaMabuta_Joint"]["x"], #L_UwaMabuta_Joint
        correctedBones["L_UwaMabuta_Joint"]["y"],
        correctedBones["L_UwaMabuta_Joint"]["z"],
        correctedBones["L_UwaMabuta_Joint"]["rot_x"],
        correctedBones["L_UwaMabuta_Joint"]["rot_y"],
        correctedBones["L_UwaMabuta_Joint"]["rot_z"],

        correctedBones["L_UpEyelid1"]["x"], #L_UpEyelid1
        correctedBones["L_UpEyelid1"]["y"],
        correctedBones["L_UpEyelid1"]["z"],
        correctedBones["L_UpEyelid1"]["rot_x"],
        correctedBones["L_UpEyelid1"]["rot_y"],
        correctedBones["L_UpEyelid1"]["rot_z"],

        correctedBones["L_UpEyelid2"]["x"], #L_UpEyelid2
        correctedBones["L_UpEyelid2"]["y"],
        correctedBones["L_UpEyelid2"]["z"],
        correctedBones["L_UpEyelid2"]["rot_x"],
        correctedBones["L_UpEyelid2"]["rot_y"],
        correctedBones["L_UpEyelid2"]["rot_z"],

        correctedBones["R_UwaMabuta_Joint"]["x"], #R_UwaMabuta_Joint
        correctedBones["R_UwaMabuta_Joint"]["y"],
        correctedBones["R_UwaMabuta_Joint"]["z"],
        correctedBones["R_UwaMabuta_Joint"]["rot_x"],
        correctedBones["R_UwaMabuta_Joint"]["rot_y"],
        correctedBones["R_UwaMabuta_Joint"]["rot_z"],

        correctedBones["R_UpEyelid1"]["x"], #R_UpEyelid1
        correctedBones["R_UpEyelid1"]["y"],
        correctedBones["R_UpEyelid1"]["z"],
        correctedBones["R_UpEyelid1"]["rot_x"],
        correctedBones["R_UpEyelid1"]["rot_y"],
        correctedBones["R_UpEyelid1"]["rot_z"],

        correctedBones["R_UpEyelid2"]["x"], #R_UpEyelid2
        correctedBones["R_UpEyelid2"]["y"],
        correctedBones["R_UpEyelid2"]["z"],
        correctedBones["R_UpEyelid2"]["rot_x"],
        correctedBones["R_UpEyelid2"]["rot_y"],
        correctedBones["R_UpEyelid2"]["rot_z"],

        correctedBones["L_Eye_Joint"]["x"], #L_Eye_Joint
        correctedBones["L_Eye_Joint"]["y"],
        correctedBones["L_Eye_Joint"]["z"],
        correctedBones["L_Eye_Joint"]["rot_x"],
        correctedBones["L_Eye_Joint"]["rot_y"],
        correctedBones["L_Eye_Joint"]["rot_z"],

        correctedBones["R_Eye_Joint"]["x"], #R_Eye_Joint
        correctedBones["R_Eye_Joint"]["y"],
        correctedBones["R_Eye_Joint"]["z"],
        correctedBones["R_Eye_Joint"]["rot_x"],
        correctedBones["R_Eye_Joint"]["rot_y"],
        correctedBones["R_Eye_Joint"]["rot_z"],

        correctedBones["L_SitaMabuta_Joint"]["x"], #L_SitaMabuta_Joint
        correctedBones["L_SitaMabuta_Joint"]["y"],
        correctedBones["L_SitaMabuta_Joint"]["z"],
        correctedBones["L_SitaMabuta_Joint"]["rot_x"],
        correctedBones["L_SitaMabuta_Joint"]["rot_y"],
        correctedBones["L_SitaMabuta_Joint"]["rot_z"],

        correctedBones["L_LowEyelid1"]["x"], #L_LowEyelid1
        correctedBones["L_LowEyelid1"]["y"],
        correctedBones["L_LowEyelid1"]["z"],
        correctedBones["L_LowEyelid1"]["rot_x"],
        correctedBones["L_LowEyelid1"]["rot_y"],
        correctedBones["L_LowEyelid1"]["rot_z"],

        correctedBones["L_LowEyelid2"]["x"], #L_LowEyelid2
        correctedBones["L_LowEyelid2"]["y"],
        correctedBones["L_LowEyelid2"]["z"],
        correctedBones["L_LowEyelid2"]["rot_x"],
        correctedBones["L_LowEyelid2"]["rot_y"],
        correctedBones["L_LowEyelid2"]["rot_z"],

        correctedBones["R_SitaMabuta_Joint"]["x"], #R_SitaMabuta_Joint
        correctedBones["R_SitaMabuta_Joint"]["y"],
        correctedBones["R_SitaMabuta_Joint"]["z"],
        correctedBones["R_SitaMabuta_Joint"]["rot_x"],
        correctedBones["R_SitaMabuta_Joint"]["rot_y"],
        correctedBones["R_SitaMabuta_Joint"]["rot_z"],

        correctedBones["R_LowEyelid1"]["x"], #R_LowEyelid1
        correctedBones["R_LowEyelid1"]["y"],
        correctedBones["R_LowEyelid1"]["z"],
        correctedBones["R_LowEyelid1"]["rot_x"],
        correctedBones["R_LowEyelid1"]["rot_y"],
        correctedBones["R_LowEyelid1"]["rot_z"],

        correctedBones["R_LowEyelid2"]["x"], #R_LowEyelid2
        correctedBones["R_LowEyelid2"]["y"],
        correctedBones["R_LowEyelid2"]["z"],
        correctedBones["R_LowEyelid2"]["rot_x"],
        correctedBones["R_LowEyelid2"]["rot_y"],
        correctedBones["R_LowEyelid2"]["rot_z"],

        correctedBones["L_Hoho_Joint"]["x"], #L_Hoho_Joint
        correctedBones["L_Hoho_Joint"]["y"],
        correctedBones["L_Hoho_Joint"]["z"],
        correctedBones["L_Hoho_Joint"]["rot_x"],
        correctedBones["L_Hoho_Joint"]["rot_y"],
        correctedBones["L_Hoho_Joint"]["rot_z"],

        correctedBones["L_Hana_Joint"]["x"], #L_Hana_Joint
        correctedBones["L_Hana_Joint"]["y"],
        correctedBones["L_Hana_Joint"]["z"],
        correctedBones["L_Hana_Joint"]["rot_x"],
        correctedBones["L_Hana_Joint"]["rot_y"],
        correctedBones["L_Hana_Joint"]["rot_z"],

        correctedBones["L_Kobana_Joint"]["x"], #L_Kobana_Joint
        correctedBones["L_Kobana_Joint"]["y"],
        correctedBones["L_Kobana_Joint"]["z"],
        correctedBones["L_Kobana_Joint"]["rot_x"],
        correctedBones["L_Kobana_Joint"]["rot_y"],
        correctedBones["L_Kobana_Joint"]["rot_z"],

        correctedBones["L_Chobo_Joint"]["x"], #L_Chobo_Joint
        correctedBones["L_Chobo_Joint"]["y"],
        correctedBones["L_Chobo_Joint"]["z"],
        correctedBones["L_Chobo_Joint"]["rot_x"],
        correctedBones["L_Chobo_Joint"]["rot_y"],
        correctedBones["L_Chobo_Joint"]["rot_z"],

        correctedBones["R_Hoho_Joint"]["x"], #R_Hoho_Joint
        correctedBones["R_Hoho_Joint"]["y"],
        correctedBones["R_Hoho_Joint"]["z"],
        correctedBones["R_Hoho_Joint"]["rot_x"],
        correctedBones["R_Hoho_Joint"]["rot_y"],
        correctedBones["R_Hoho_Joint"]["rot_z"],

        correctedBones["R_Hana_Joint"]["x"], #R_Hana_Joint
        correctedBones["R_Hana_Joint"]["y"],
        correctedBones["R_Hana_Joint"]["z"],
        correctedBones["R_Hana_Joint"]["rot_x"],
        correctedBones["R_Hana_Joint"]["rot_y"],
        correctedBones["R_Hana_Joint"]["rot_z"],

        correctedBones["R_Kobana_Joint"]["x"], #R_Kobana_Joint
        correctedBones["R_Kobana_Joint"]["y"],
        correctedBones["R_Kobana_Joint"]["z"],
        correctedBones["R_Kobana_Joint"]["rot_x"],
        correctedBones["R_Kobana_Joint"]["rot_y"],
        correctedBones["R_Kobana_Joint"]["rot_z"],

        correctedBones["R_Chobo_Joint"]["x"], #R_Chobo_Joint
        correctedBones["R_Chobo_Joint"]["y"],
        correctedBones["R_Chobo_Joint"]["z"],
        correctedBones["R_Chobo_Joint"]["rot_x"],
        correctedBones["R_Chobo_Joint"]["rot_y"],
        correctedBones["R_Chobo_Joint"]["rot_z"],

        correctedBones["L_LipU_Joint"]["x"], #L_LipU_Joint
        correctedBones["L_LipU_Joint"]["y"],
        correctedBones["L_LipU_Joint"]["z"],
        correctedBones["L_LipU_Joint"]["rot_x"],
        correctedBones["L_LipU_Joint"]["rot_y"],
        correctedBones["L_LipU_Joint"]["rot_z"],

        correctedBones["R_LipU_Joint"]["x"], #R_LipU_Joint
        correctedBones["R_LipU_Joint"]["y"],
        correctedBones["R_LipU_Joint"]["z"],
        correctedBones["R_LipU_Joint"]["rot_x"],
        correctedBones["R_LipU_Joint"]["rot_y"],
        correctedBones["R_LipU_Joint"]["rot_z"],

        correctedBones["M_LipU_Joint"]["x"], #M_LipU_Joint
        correctedBones["M_LipU_Joint"]["y"],
        correctedBones["M_LipU_Joint"]["z"],
        correctedBones["M_LipU_Joint"]["rot_x"],
        correctedBones["M_LipU_Joint"]["rot_y"],
        correctedBones["M_LipU_Joint"]["rot_z"],

        correctedBones["Jaw_Joint"]["x"], #Jaw_Joint
        correctedBones["Jaw_Joint"]["y"],
        correctedBones["Jaw_Joint"]["z"],
        correctedBones["Jaw_Joint"]["rot_x"],
        correctedBones["Jaw_Joint"]["rot_y"],
        correctedBones["Jaw_Joint"]["rot_z"],

        correctedBones["L_LipD_Joint"]["x"], #L_LipD_Joint
        correctedBones["L_LipD_Joint"]["y"],
        correctedBones["L_LipD_Joint"]["z"],
        correctedBones["L_LipD_Joint"]["rot_x"],
        correctedBones["L_LipD_Joint"]["rot_y"],
        correctedBones["L_LipD_Joint"]["rot_z"],

        correctedBones["R_LipD_Joint"]["x"], #R_LipD_Joint
        correctedBones["R_LipD_Joint"]["y"],
        correctedBones["R_LipD_Joint"]["z"],
        correctedBones["R_LipD_Joint"]["rot_x"],
        correctedBones["R_LipD_Joint"]["rot_y"],
        correctedBones["R_LipD_Joint"]["rot_z"],

        correctedBones["M_LipD_Joint"]["x"], #M_LipD_Joint
        correctedBones["M_LipD_Joint"]["y"],
        correctedBones["M_LipD_Joint"]["z"],
        correctedBones["M_LipD_Joint"]["rot_x"],
        correctedBones["M_LipD_Joint"]["rot_y"],
        correctedBones["M_LipD_Joint"]["rot_z"],

        correctedBones["L_LipE_Joint"]["x"], #L_LipE_Joint
        correctedBones["L_LipE_Joint"]["y"],
        correctedBones["L_LipE_Joint"]["z"],
        correctedBones["L_LipE_Joint"]["rot_x"],
        correctedBones["L_LipE_Joint"]["rot_y"],
        correctedBones["L_LipE_Joint"]["rot_z"],

        correctedBones["R_LipE_Joint"]["x"], #R_LipE_Joint
        correctedBones["R_LipE_Joint"]["y"],
        correctedBones["R_LipE_Joint"]["z"],
        correctedBones["R_LipE_Joint"]["rot_x"],
        correctedBones["R_LipE_Joint"]["rot_y"],
        correctedBones["R_LipE_Joint"]["rot_z"],

        correctedBones["Bero1_Joint"]["x"], #Bero1_Joint
        correctedBones["Bero1_Joint"]["y"],
        correctedBones["Bero1_Joint"]["z"],
        correctedBones["Bero1_Joint"]["rot_x"],
        correctedBones["Bero1_Joint"]["rot_y"],
        correctedBones["Bero1_Joint"]["rot_z"],

        correctedBones["Bero1_Joint_pos"]["x"], #Bero1_Joint_pos
        correctedBones["Bero1_Joint_pos"]["y"],
        correctedBones["Bero1_Joint_pos"]["z"],
        correctedBones["Bero1_Joint_pos"]["rot_x"],
        correctedBones["Bero1_Joint_pos"]["rot_y"],
        correctedBones["Bero1_Joint_pos"]["rot_z"],

        #idk after this
        
        0,
        0,
        0,
        
        0,
        0,
        0,
        
        0,
        0,
        0,
        
        0,
        0,
        0,


        0,
        0,
        0,
        0,
        0,
        0,
        #Bero1_Joint_pos_x,
        #Bero1_Joint_pos_y,
        #Bero1_Joint_pos_z,
        #Bero1_Joint_pos_rot_x,
        #Bero1_Joint_pos_rot_y,
        #Bero1_Joint_pos_rot_z,

        Bero2_Joint_scale_x, #Bero2_Joint #scale
        Bero2_Joint_scale_y,  #scale
        Bero2_Joint_scale_z,  #scale
        Bero2_Joint_x, #pos
        Bero2_Joint_y, #pos
        Bero2_Joint_z, #pos

        0,
        0,
        0,
        0,
        0,
        0,
        #Bero3_Joint_x, #Bero3_Joint
        #Bero3_Joint_y,
        #Bero3_Joint_z,
        #Bero3_Joint_rot_x,
        #Bero3_Joint_rot_y,
        #Bero3_Joint_rot_z,
    ]

def applyRotationFromAnimdata(armature, animdata):
    offset_bone = armature.pose.bones['BODY_SCALE__group']
    base_bone = armature.pose.bones['BASE']
    upper_body_bone = armature.pose.bones['Spine1']
    lower_body_bone = armature.pose.bones['Hip']
    neck_bone = armature.pose.bones['Neck']
    head_bone = armature.pose.bones['Head']

    right_inner_shoulder = armature.pose.bones['R_Shoulder']
    right_outer_shoulder = armature.pose.bones['R_Arm']
    right_elbow = armature.pose.bones['R_ForeArm']
    right_hand = armature.pose.bones['R_Hand']

    left_inner_shoulder = armature.pose.bones['L_Shoulder']
    left_outer_shoulder = armature.pose.bones['L_Arm']
    left_elbow = armature.pose.bones['L_ForeArm']
    left_hand = armature.pose.bones['L_Hand']

    right_hip = armature.pose.bones['R_UpLeg']
    right_knee = armature.pose.bones['R_Leg']
    right_foot = armature.pose.bones['R_Foot']

    left_hip = armature.pose.bones['L_UpLeg']
    left_knee = armature.pose.bones['L_Leg']
    left_foot = armature.pose.bones['L_Foot']
    
    offset_bone.location.x = animdata[3] / 1000
    offset_bone.location.y = animdata[4] / 1000 - 1.15
    offset_bone.location.z = animdata[5] / 1000

    base_bone.rotation_euler.x = animdata[11]
    base_bone.rotation_euler.y = animdata[10]
    base_bone.rotation_euler.z = -animdata[9]

    upper_body_bone.rotation_euler.x = animdata[14] - halfpi
    upper_body_bone.rotation_euler.y = animdata[13]
    upper_body_bone.rotation_euler.z = animdata[12] * -1

    lower_body_bone.rotation_euler.x = animdata[17] + halfpi
    lower_body_bone.rotation_euler.y = animdata[16]
    lower_body_bone.rotation_euler.z = animdata[15] * -1

    neck_bone.rotation_euler.x = animdata[23]
    neck_bone.rotation_euler.y = animdata[22]
    neck_bone.rotation_euler.z = animdata[21] * -1

    head_bone.rotation_euler.x = animdata[25]
    head_bone.rotation_euler.y = animdata[26] - halfpi
    head_bone.rotation_euler.z = animdata[24] - halfpi

    # --------------------------------------------------------
   
    x, y, z = convertArmToBlenderXYZ(animdata[27], animdata[28], animdata[29])
    
    right_inner_shoulder.rotation_euler.x = x
    right_inner_shoulder.rotation_euler.y = y
    right_inner_shoulder.rotation_euler.z = z

    right_outer_shoulder.rotation_euler.x = animdata[30] * -1 - halfpi
    right_outer_shoulder.rotation_euler.y = animdata[32] * -1
    right_outer_shoulder.rotation_euler.z = animdata[31] * -1


    right_elbow.rotation_euler.x = animdata[33] * -1
    right_elbow.rotation_euler.y = animdata[34]
    right_elbow.rotation_euler.z = animdata[35] * -1

    right_hand.rotation_euler.x = animdata[36] - halfpi
    right_hand.rotation_euler.y = animdata[38]
    right_hand.rotation_euler.z = animdata[37] * -1
    # --------------------------------------------------------

    x, y, z = convertArmToBlenderXYZ(animdata[39], animdata[40], animdata[41])
    
    left_inner_shoulder.rotation_euler.x = 0 - x
    left_inner_shoulder.rotation_euler.y = y
    left_inner_shoulder.rotation_euler.z = z + pi

    left_outer_shoulder.rotation_euler.x = animdata[42] + halfpi
    left_outer_shoulder.rotation_euler.y = animdata[44] * -1
    left_outer_shoulder.rotation_euler.z = animdata[43]
    
    left_elbow.rotation_euler.x = animdata[45]
    left_elbow.rotation_euler.y = animdata[46]
    left_elbow.rotation_euler.z = animdata[47]

    left_hand.rotation_euler.x = animdata[48] * -1 + halfpi
    left_hand.rotation_euler.y = animdata[50] * -1
    left_hand.rotation_euler.z = animdata[49] * -1
    # --------------------------------------------------------

    right_hip.rotation_euler.x = animdata[53]
    right_hip.rotation_euler.y = animdata[52]
    right_hip.rotation_euler.z = animdata[51] * -1

    right_knee.rotation_euler.x = animdata[56]
    right_knee.rotation_euler.y = animdata[55]
    right_knee.rotation_euler.z = animdata[54] * -1

    right_foot.rotation_euler.x = animdata[59]
    right_foot.rotation_euler.y = animdata[58]
    right_foot.rotation_euler.z = animdata[57] * -1
    # --------------------------------------------------------

    left_hip.rotation_euler.x = animdata[62]
    left_hip.rotation_euler.y = animdata[61]
    left_hip.rotation_euler.z = animdata[60] * -1

    left_knee.rotation_euler.x = animdata[65]
    left_knee.rotation_euler.y = animdata[64]
    left_knee.rotation_euler.z = animdata[63] * -1

    left_foot.rotation_euler.x = animdata[68]
    left_foot.rotation_euler.y = animdata[67]
    left_foot.rotation_euler.z = animdata[66] * -1