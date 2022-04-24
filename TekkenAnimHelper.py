import pathlib
import struct
import bpy
import numpy as np
from math import pi, asin, atan2

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

def getAnimTemplate():
    filename = pathlib.Path(__file__).parent.resolve().__str__() + "\\source_anim.bin"
    data = None
    with open(filename, "rb") as f:
        data = list(f.read())
    return data

animTemplate = getAnimTemplate()
    
class TekkenAnimation:
    AnimC8OffsetTable = {
        0x17: 0x64,
        0x19: 0x6C,
        0x1B: 0x74,
        0x1d: 0x7c,
        0x1f: 0x80,
        0x21: 0x8c,
        0x23: 0x94,
        0x31: 0xcc 
    }
        
    def _getHeaderSizeFromArgs_(bone_count):
        return TekkenAnimation.AnimC8OffsetTable[bone_count]
        
    def _getFramesizeFromArgs_(bone_count):
        return bone_count * 0xC

    def __init__(self, data=None):
        if data == None:
            data = animTemplate
            pass
            
        self.data = data
        self.type = self.byte(0)
        self.type2 = self.byte(2)
        self.length = self.getLength()
        self.offset = self.getOffset()
        self.frame_size = self.getFramesize()
        self.field_count = int(self.frame_size / 4)
        self.recalculateSize() #Crop or add missing bytes if needed
        
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
        return TekkenAnimation.AnimC8OffsetTable[self.type2]
        
    def getFramesize(self):
        return self.type2 * 0xC
        
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
    
    orig_quat = get_quaternion_from_euler(pi / 2, 0, pi / 2)
    orig_mat = quaternionToRotationMatrix(orig_quat)
    orig_mat = np.linalg.inv(orig_mat)

    quat = get_quaternion_from_euler(x, y, z)
    mat = quaternionToRotationMatrix(quat)

    mat = np.matmul(mat, orig_mat)
    
    x, y, z = getRotationFromMatrix(mat, mode=3)
    
    return x, -y, -z

# expects x y z tekken rotation, outputs blender rotation
def convertArmToBlenderXYZ(x, y, z):
    orig_quat = get_quaternion_from_euler(-pi / 2, pi / 2, 0)

    orig_mat = quaternionToRotationMatrix(orig_quat)
    orig_mat = np.linalg.inv(orig_mat)

    quat = get_quaternion_from_euler(x, y, z)
    mat = quaternionToRotationMatrix(quat)

    mat = np.matmul(mat, orig_mat)
    
    x, y, z = getRotationFromMatrix(mat, mode=1)
    
    return -z, y, -x
             
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
    UpperBody3 = bones["Spine1"].rotation_euler.x + (pi / 2)
    
    LowerBody1 = bones["Hip"].rotation_euler.z * -1
    LowerBody2 = bones["Hip"].rotation_euler.y
    LowerBody3 = bones["Hip"].rotation_euler.x - (pi / 2)
    
    Neck1 = bones["Neck"].rotation_euler.z * -1
    Neck2 = bones["Neck"].rotation_euler.y
    Neck3 = bones["Neck"].rotation_euler.x
    
    Head1 = bones["Head"].rotation_euler.z + (pi / 2)
    Head2 = bones["Head"].rotation_euler.x
    Head3 = bones["Head"].rotation_euler.y + (pi / 2)
    
    #to be used if you have IK for those bones too
    """
    RotX = visualRots["BASE"]['z'] * -1
    RotY = visualRots["BASE"]['y']
    RotZ = visualRots["BASE"]['x'] 
    
    UpperBody1 = visualRots["Spine1"]['z'] * -1
    UpperBody2 = visualRots["Spine1"]['y']
    UpperBody3 = visualRots["Spine1"]['x'] + (pi / 2)
    
    LowerBody1 = visualRots["Hip"]['z'] * -1
    LowerBody2 = visualRots["Hip"]['y']
    LowerBody3 = visualRots["Hip"]['x'] - (pi / 2)
    
    Neck1 = visualRots["Neck"]['z'] * -1
    Neck2 = visualRots["Neck"]['y']
    Neck3 = visualRots["Neck"]['x']
    
    Head1 = visualRots["Head"]['z'] + (pi / 2)
    Head2 = visualRots["Head"]['x']
    Head3 = visualRots["Head"]['y'] + (pi / 2)
    """
    
    # --------- SHOULDER ------------
    
    #x, y, z = bones["R_Shoulder"].rotation_euler
    x, y, z = visualRots["R_Shoulder"]['x'], visualRots["R_Shoulder"]['y'], visualRots["R_Shoulder"]['z']
    x, y, z = convertArmToTekkenXYZ(x, y, z)
    RightInnerShoulder1 = x + pi
    RightInnerShoulder2 = y
    RightInnerShoulder3 = z
    
    # --------- ARM IK -----------
    
    RightOuterShoulder1 = visualRots["R_Arm"]['x'] * -1 - pi / 2
    RightOuterShoulder2 = visualRots["R_Arm"]['z'] * -1
    RightOuterShoulder3 = visualRots["R_Arm"]['y'] * -1
    
    RightElbow1 = visualRots["R_ForeArm"]['x'] * -1
    RightElbow2 = visualRots["R_ForeArm"]['y']
    RightElbow3 = visualRots["R_ForeArm"]['z'] * -1
    
    RightHand1 = visualRots["R_Hand"]['x'] + pi / 2
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
    
    LeftOuterShoulder1 = visualRots["L_Arm"]['x'] - pi / 2
    LeftOuterShoulder2 = visualRots["L_Arm"]['z']
    LeftOuterShoulder3 = visualRots["L_Arm"]['y'] * -1
    
    LeftElbow1 = visualRots["L_ForeArm"]['x']
    LeftElbow2 = visualRots["L_ForeArm"]['y']
    LeftElbow3 = visualRots["L_ForeArm"]['z']
    
    LeftHand1 = visualRots["L_Hand"]['x'] * -1 + pi / 2
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

    upper_body_bone.rotation_euler.x = animdata[14] - pi / 2
    upper_body_bone.rotation_euler.y = animdata[13]
    upper_body_bone.rotation_euler.z = animdata[12] * -1

    lower_body_bone.rotation_euler.x = animdata[17] + pi / 2
    lower_body_bone.rotation_euler.y = animdata[16]
    lower_body_bone.rotation_euler.z = animdata[15] * -1

    neck_bone.rotation_euler.x = animdata[23]
    neck_bone.rotation_euler.y = animdata[22]
    neck_bone.rotation_euler.z = animdata[21] * -1

    head_bone.rotation_euler.x = animdata[25]
    head_bone.rotation_euler.y = animdata[26] - pi / 2
    head_bone.rotation_euler.z = animdata[24] - pi / 2

    # --------------------------------------------------------
   
    x, y, z = convertArmToBlenderXYZ(animdata[27], animdata[28], animdata[29])
    
    right_inner_shoulder.rotation_euler.x = x
    right_inner_shoulder.rotation_euler.y = y
    right_inner_shoulder.rotation_euler.z = z

    right_outer_shoulder.rotation_euler.x = animdata[30] * -1 - pi / 2
    right_outer_shoulder.rotation_euler.y = animdata[32] * -1
    right_outer_shoulder.rotation_euler.z = animdata[31] * -1


    right_elbow.rotation_euler.x = animdata[33] * -1
    right_elbow.rotation_euler.y = animdata[34]
    right_elbow.rotation_euler.z = animdata[35] * -1

    right_hand.rotation_euler.x = animdata[36] - pi / 2
    right_hand.rotation_euler.y = animdata[38]
    right_hand.rotation_euler.z = animdata[37] * -1
    # --------------------------------------------------------

    x, y, z = convertArmToBlenderXYZ(animdata[39], animdata[40], animdata[41])
    
    left_inner_shoulder.rotation_euler.x = 0 - x
    left_inner_shoulder.rotation_euler.y = y
    left_inner_shoulder.rotation_euler.z = z + pi

    left_outer_shoulder.rotation_euler.x = animdata[42] + pi / 2
    left_outer_shoulder.rotation_euler.y = animdata[44] * -1
    left_outer_shoulder.rotation_euler.z = animdata[43]
    
    left_elbow.rotation_euler.x = animdata[45]
    left_elbow.rotation_euler.y = animdata[46]
    left_elbow.rotation_euler.z = animdata[47]

    left_hand.rotation_euler.x = animdata[48] * -1 + pi / 2
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