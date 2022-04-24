from . Addresses import AddressFile, GameClass
from . TekkenAnimHelper import TekkenAnimation, getAnimFrameFromBones, applyRotationFromAnimdata
#from . VR import VRIntegrator
from time import time, sleep
import bpy
import pathlib
import threading
import struct
from mathutils import Vector, Euler
from math import pi
import traceback

class TKDirect: #(Singleton):
    def __init__(self):
        self.running = False
        self.game_addresses = AddressFile(pathlib.Path(__file__).parent.resolve().__str__() + "\\game_addresses.txt")
        
        self.update_delay = 1/30 #30 times per second
        
        self.preview = False
        self.tracking = False
        self.attach_player = False
        self.camera_preview = False
        self.camera_tracking = False
        
        self.single_frame_preview = True
        
        self.armature_name = None
        self.armature_name_2p = None
        self.camera_name = None
        
        self.camera = None
        self.armature = None
        self.armature_2p = None
        self.p1_addr = None
        self.p2_addr = None
        self.cam_addr = None
        self.T = None
        
        
        self.allocated_frame_anim = None
        self.wrote_player_move = False
        self.preview_frame_anim = TekkenAnimation()
        
        self.allocated_frame_anim_2p = None
        self.wrote_player_move_2p = False
        self.preview_frame_anim_2p = TekkenAnimation()
        
    # -- #
        
    def start(self):
        if self.running: return
        #print("start")
    
        TekkenGame = GameClass("TekkenGame-Win64-Shipping.exe")
        TekkenGame.applyModuleAddress(self.game_addresses)
        self.T = TekkenGame
    
        self.running = True
        
        # Loop init #
        
        self.getPlayerAddresses()
        self.getCameraAddr()
        self.allocateFrameAnim()
        self.getFrameCounterAddr()
        
        
        # VR stuff, pointless
        #self.VR = VRIntegrator()
        #self.VR.start()
        
        # Loop start #
        
        newThread = threading.Thread(target=self.loop)
        newThread.start()
        
    def loop(self):
        print("Starting loop")
        self.running = True
        
        while self.running:
            try:
                #1p code
                if self.armature_name != None:
                    self.armature = bpy.context.scene.objects[self.armature_name]
                    
                    if self.tracking:
                        self.trackPlayer(self.p1_addr, self.armature)
                    elif self.preview:
                        self.previewPlayer(self.p1_addr, self.armature)
                        self.T.writeBytes(self.allocated_frame_anim, bytes(self.preview_frame_anim.data))
                        
                        #less byts written
                        #offset = self.preview_frame_anim.offset
                        #self.T.writeBytes(self.allocated_frame_anim + offset, bytes(self.preview_frame_anim.data[offset:]))
                    
                    # if we need to carry 2p around
                    if self.attach_player and not self.armature_name_2p: self.attachPlayer()
                    
                #2p code
                if self.armature_name_2p != None:
                    self.armature_2p = bpy.context.scene.objects[self.armature_name_2p]
                    
                    if self.tracking:
                        self.trackPlayer(self.p2_addr, self.armature_2p)
                    elif self.preview:
                        self.previewPlayer(self.p2_addr, self.armature_2p)
                        self.T.writeBytes(self.allocated_frame_anim_2p, bytes(self.preview_frame_anim_2p.data))
                    
                # camera
                if self.camera_name != None:
                    self.camera = bpy.context.scene.objects[self.camera_name]
                    if self.camera_preview:
                        self.previewCamera()
                    elif self.camera_tracking:
                        self.trackCamera()
                    
            except Exception:
                print(traceback.format_exc())
                self.stop()
                
            self.waitFrame()
            
        print("Loop exited")
    
    def waitFrame(self):
        #sleep(self.update_delay)
        
        currFrame = self.T.readInt(self.frame_counter, 4)
        while self.running and self.T.readInt(self.frame_counter, 4) == currFrame:
            pass
        
    
    def stop(self):
        if not self.running: return
        #print("stop()")
        
        self.cam_addr = None
        self.allocated_frame_anim = None
        self.allocated_frame_anim_2p = None
        self.wrote_player_move = False
        self.wrote_player_move_2p = False
        self.running = False
        self.preview = False
        self.tracking = False
        self.T.close()
        self.T = None
        
    # - Utils - #
    
    def lockCamera(self):
        self.T.writeBytes(self.game_addresses['camera_code_injection'] + 0xE, bytes([0x90] * 8))
        self.T.writeBytes(self.game_addresses['camera_code_injection'] + 0x25, bytes([0x90] * 6))
        self.T.writeBytes(self.game_addresses['camera_code_injection'], bytes([0x90] * 8))
        self.T.writeBytes(self.game_addresses['camera_code_injection'] + 0x1B, bytes([0x90] * 6))
        self.T.writeBytes(self.game_addresses['camera_code_injection2'], bytes([0x90] * 8))
        
    def unlockCamera(self):
        self.T.writeBytes(self.game_addresses['camera_code_injection'] + 0xE, bytes([0xF2, 0x0f, 0x11, 0x87, 0x04, 0x04, 0x0, 0x0]))
        self.T.writeBytes(self.game_addresses['camera_code_injection'] + 0x25, bytes([0x89, 0x87, 0x0c, 0x04, 0x0, 0x0]))
        self.T.writeBytes(self.game_addresses['camera_code_injection'], bytes([0xF2, 0x0F, 0x11, 0x87, 0xF8, 0x03, 0x0, 0x0])) # x
        self.T.writeBytes(self.game_addresses['camera_code_injection'] + 0x1B, bytes([0x89, 0x87, 0, 0x04, 0, 0]))
        self.T.writeBytes(self.game_addresses['camera_code_injection2'], bytes([0xF3, 0x0F, 0x11, 0x89, 0x9C, 0x03, 0x00, 0x00])) #fov
    
    # getPlayerYaw - Blender output
    def getPlayerYaw(self, playerAddress):
        return self.T.readInt(playerAddress + 0x1c0, 2) * (pi * 2 / 65535)
        
    # setPlayerYaw - Expects blender input
    def setPlayerYaw(self, playerAddr, yaw):
        while yaw > pi * 2:
            yaw -= pi * 2
        yaw = int((yaw - pi * 2) * (65535 / (pi * 2)))
        while yaw < 0: yaw += 65535
        self.T.writeInt(playerAddr + 0xEE, yaw, 2) #half circle offset
    
    # getPlayerPos - Tekken output
    def getPlayerPos(self, playerAddress):
        #x, y, z
        
        return {
            'x': self.readFloat(playerAddress + 0xFC),
            'y': -self.readFloat(playerAddress + 0xF4),
            'z': self.getPlayerFloorHeight(playerAddress),
        }
    
    # setPlayerPos - Expects Tekken input
    def setPlayerPos(self, playerAddress, x, y): #tekken output
        self.writeFloat(playerAddress + 0xE8, x)
        self.writeFloat(playerAddress + 0xE0, -y)
    
    # getPlayerFloorHeight - Tekken output
    def getPlayerFloorHeight(self, playerAddress):
        return self.readFloat(playerAddress + 0x1B0)
    
    # setPlayerFloorHeight - Expects Tekken input
    def setPlayerFloorHeight(self, playerAddress, floorheight):
        self.writeFloat(playerAddress + 0x1B0, floorheight)
    
    
    # - #
    
    # getArmaturePos - Blender output
    def getArmaturePos(self, armature):
        return armature.location
        
    # setArmaturePos - Expects blender input
    def setArmaturePos(self, armature, x, y, z):
        armature.location = Vector((x, y, z))  #blender inverts x and y
        
    # getArmatureYaw - Blender output
    def getArmatureYaw(self, armature):
        return armature.rotation_euler[2]
        
    # setArmatureYaw - Expects blender input
    def setArmatureYaw(self, armature, yaw):
        armature.rotation_euler = Euler((pi / 2, 0.0, yaw), 'XYZ')
        
    # - #
    
    # getBlenderCameraProperties - Blender output
    def getBlenderCameraProperties(self):
    
        x, y, z = self.camera.location
        rot_x, rot_y, rot_z = self.camera.rotation_euler
        fov = self.camera.data.angle * 57 # i don't know why it's 57 but this is approximatively correct
        
        return [x, y, z, rot_x, rot_y, rot_z, fov]
        
    # setBlenderCameraProperties - Expects tekken input
    def setBlenderCameraProperties(self, camData):
        x, y, z, rot_x, rot_y, rot_z, fov = camData
        
        self.camera.location = (x, y, z)
        self.camera.rotation_euler = (rot_x, rot_z, rot_y)
        
        self.camera.data.angle = fov
    
    # setTekkenCameraProperties - Expects tekken input
    def setTekkenCameraProperties(self, x, y, z, rot_x, rot_y, rot_z, fov):
        camAddr = self.cam_addr
    
        self.writeFloat(camAddr + 0x3F8, x * 100) #x
        self.writeFloat(camAddr + 0x3FC, -y * 100) #y
        self.writeFloat(camAddr + 0x400, z * 100) #z
        
        mult = 180 / pi
        
        rot_x -= pi / 2
        rot_z += pi / 2
        
        y = (rot_x) * mult
        p = (rot_z * -1) * mult
        tilt = (rot_y) * mult
        
        self.writeFloat(camAddr + 0x404, y) #roty
        self.writeFloat(camAddr + 0x408, p) #rotx
        self.writeFloat(camAddr + 0x40C, tilt) #tilt
        
        self.writeFloat(camAddr + 0x39C, fov) #FOV
    
    # getTekkenCameraProperties - Blender output
    def getTekkenCameraProperties(self):
        camAddr = self.cam_addr
    
        x = self.readFloat(camAddr + 0x3F8) / 100
        y = -(self.readFloat(camAddr + 0x3FC) / 100)
        z = self.readFloat(camAddr + 0x400)/ 100
        
        mult = pi / 180
        
        rot_x = self.readFloat(camAddr + 0x404) * mult + pi / 2 #roty
        rot_y = self.readFloat(camAddr + 0x408) * mult * -1 - pi / 2 #rotx
        rot_z = 0#self.readFloat(camAddr + 0x40C, tilt) #tilt
        
        fov = self.readFloat(camAddr + 0x39C) / 57
        
        return x, y, z, rot_x, rot_y, rot_z, fov
        
    # - Main functions - #
    
    def trackCamera(self):
        camData = self.getTekkenCameraProperties()
        self.setBlenderCameraProperties(camData)
        
    
    def previewCamera(self):
        camData = self.getBlenderCameraProperties()
        self.setTekkenCameraProperties(*camData)
        
    def attachPlayer(self):
        localLocation = self.armature.pose.bones["BODY_SCALE__group"].location
    
        offset1 = -(localLocation[2] * 1000)
        offset2 = (localLocation[1]) * 1000
        offset3 = localLocation[0] * 1000
        
        playerPos = self.getPlayerPos(self.p1_addr)
        
        playerPos['x'] += offset1 + 2000 #?
        playerPos['y'] += offset3 + 2000 #?
        playerPos['z'] += offset2 + 2500 #hauteur
        
        self.setPlayerFloorHeight(self.p2_addr, playerPos['z'])
        self.setPlayerPos(self.p2_addr, playerPos['x'], playerPos['y'])
        
        
    def previewPlayer(self, playerAddress, armature):
        #self.VR.setIKFromVR(self.armature)
    
        animFrame = getAnimFrameFromBones(armature)
        
        for i in range(self.preview_frame_anim.field_count):
            self.preview_frame_anim.setField(animFrame[i], 0, i)
        
        rot = self.getArmatureYaw(armature)
        self.setPlayerYaw(playerAddress, rot)
        
        x, y, z = self.getArmaturePos(armature)
        self.setPlayerPos(playerAddress, x * 1000, -y * 1000)
        self.setPlayerFloorHeight(playerAddress, z * 1000)
    
    def trackPlayer(self, playerAddress, armature):
        pos = self.getPlayerPos(playerAddress)
        self.setArmaturePos(armature, (pos['x'] / 1000), -(pos['y'] / 1000), pos['z'] / 1000)
        
        yaw = self.getPlayerYaw(playerAddress)
        self.setArmatureYaw(armature, yaw)
        
        animAddr = self.getPlayerAnimAddr(playerAddress)
        if animAddr != None and self.T.readInt(animAddr, 1) == 0xC8:
            frame = self.T.readInt(playerAddress + self.game_addresses["curr_frame_timer_offset"], 4)
            frame = min(frame, self.T.readInt(animAddr + 4, 4))
            type2 = self.T.readInt(animAddr + 2, 1)
            
            offset = {
                0x17: 0x64,
                0x19: 0x6C,
                0x1B: 0x74,
                0x1d: 0x7c,
                0x1f: 0x80,
                0x21: 0x8c,
                0x23: 0x94,
                0x31: 0xcc 
            }[self.T.readInt(animAddr + 2, 1)]
            frame_size = type2 * 0xC
            field_count = frame_size // 4
            
            start_addr = animAddr + offset + frame_size * (frame - 1)
            floats = [self.readFloat(start_addr + i * 4) for i in range(min(field_count, 69))]
            applyRotationFromAnimdata(armature, floats)
        
    # -  - #
    
    def getPlayerAddresses(self):
        self.p1_addr = self.game_addresses["t7_p1_addr"]
        self.p2_addr = self.p1_addr + self.game_addresses["t7_playerstruct_size"]
        
    def getCameraAddr(self):
        self.cam_addr = self.game_addresses['camera_ptr']
    
    def getFrameCounterAddr(self):
        if self.game_addresses['using_global_frame_counter']:
            self.frame_counter = self.game_addresses['global_frame_counter']
        else:
            self.frame_counter = self.game_addresses['game_frame_counter']
    
    def allocateFrameAnim(self):
        self.allocated_frame_anim = self.T.allocateMem(self.preview_frame_anim.size)
        self.allocated_frame_anim_2p = self.T.allocateMem(self.preview_frame_anim_2p.size)
        
        self.T.writeBytes(self.allocated_frame_anim, bytes(self.preview_frame_anim.data))
        self.T.writeBytes(self.allocated_frame_anim_2p, bytes(self.preview_frame_anim_2p.data))
        
        
        print("(1p) Allocated new anim: 0x%x" % (self.allocated_frame_anim))
        print("(2p) Allocated new anim: 0x%x" % (self.allocated_frame_anim_2p))

    def writePlayerMove(self, playerid):
        if playerid == 0 and self.wrote_player_move: return
        if playerid == 1 and self.wrote_player_move_2p: return
        
        playerAddress = self.p1_addr if playerid == 0 else self.p2_addr
        animAddr = self.allocated_frame_anim if playerid == 0 else self.allocated_frame_anim_2p
        
        currmoveAddr = self.T.readInt(playerAddress + 0x220, 8)
        if currmoveAddr != 0:
            self.T.writeInt(currmoveAddr + 0x10, animAddr, 8)
            
        if playerid == 0:
            self.wrote_player_move = True
        else:
            self.wrote_player_move_2p = True
        
    def getPlayerAnimAddr(self, playerAddress):
        currmoveAddr = self.T.readInt(playerAddress + 0x220, 8)
        return None if currmoveAddr == 0 else self.T.readInt(currmoveAddr + 0x10, 8)
    
    def writeFloat(self, addr, value):
        self.T.writeBytes(addr, struct.pack('f', value))
        
    def readFloat(self, addr):
        return struct.unpack('f', self.T.readBytes(addr, 4))[0]
    
    # -Interactions- #
    
    def setActiveSkeleton(self, playerid, armature_name):
        if playerid == 0:
            self.armature_name = armature_name
            if armature_name == None: self.wrote_player_move = False
        else:
            self.armature_name_2p = armature_name
            if armature_name == None: self.wrote_player_move = False
        
        if self.preview: self.writePlayerMove(playerid)
            
    
    def allowHeadMovement(self, playerid):
        if self.preview:
            playerAddress = self.p1_addr if playerid == 0 else self.p2_addr
            currmoveAddr = self.T.readInt(playerAddress + 0x220, 8)
            
            if currmoveAddr != 0:
                self.T.writeInt(currmoveAddr + 0x98, 0, 4)

        
    def checkStatus(self):
        if False:
            self.stop()
            
    def setPlayersStageCollision(self, collision):
        if self.running:
            self.T.writeInt(self.p1_addr + 0xae0, collision, 1)
            self.T.writeInt(self.p2_addr + 0xae0, collision, 1)
    
    def setPreview(self, enabled):
        if enabled:
            self.preview = True
            self.tracking = False
            
            if not self.running:
                self.start()
                
            if self.armature_name: self.writePlayerMove(0)
            if self.armature_name_2p: self.writePlayerMove(1)
        else:
            self.preview = False
            self.wrote_player_move = False
            self.wrote_player_move_2p = False
            if not self.camera_preview and not self.camera_tracking: self.stop()
    
    def setTracking(self, enabled):
        if enabled:
            self.tracking = True
            self.preview = False
            
            if not self.running:
                self.start()
        else:
            self.tracking = False
            if not self.camera_preview and not self.camera_tracking: self.stop()
    
    def setCameraPreview(self, enabled):
        if enabled:
            self.camera_preview = True
            self.camera_tracking = False
            
            if not self.running:
                self.start()
                
            self.lockCamera()
        else:
            self.unlockCamera()
            self.camera_preview = False
            if not self.tracking and not self.preview: self.stop()
    
    def setCameraTracking(self, enabled):
        if enabled:
            self.camera_tracking = True
            
            if not self.running:
                self.start()
                
            if self.camera_preview:
                self.camera_preview = False
                self.unlockCamera()
        else:
            self.camera_tracking = False
            if not self.tracking and not self.preview: self.stop()
            
    def setDistanceLimit(self, disabled):
        if not self.running: return
        if disabled:
            self.T.writeBytes(self.game_addresses["player_distance_limit_addr"], [0xE9, 0x76, 0x01, 0x00, 0x00, 0x90])
        else:
            self.T.writeBytes(self.game_addresses["player_distance_limit_addr"], [0x0F, 0x86, 0x75, 0x01, 0x00, 0x00])
    
    def setPlayerCollision(self, disabled):
        if not self.running: return
        if disabled:
            self.T.writeBytes(self.game_addresses["player_collision_code_addr"], [0xE9, 0xE0, 0x00, 0x00, 0x00])
        else:
            self.T.writeBytes(self.game_addresses["player_collision_code_addr"], [0x75, 0x16, 0x0F, 0x2E, 0xB3])
    
    def setWallCollision(self, disabled):
        if not self.running: return
        if disabled:
            self.T.writeBytes(self.game_addresses["wall_collision_code_addr"], [0x90] * 8)
        else:
            self.T.writeBytes(self.game_addresses["wall_collision_code_addr"], [0xF3, 0x0F, 0x11, 0x86, 0x08, 0x13, 0x00, 0x00])