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
        
        self.update_delay = 1/60#0.1
        
        self.preview = False
        self.tracking = False
        self.attach_player = False
        self.camera_preview = False
        
        self.single_frame_preview = True
        
        self.collision = 0
        self.playerid = 0
        self.armature_name = None
        self.camera_name = None
        
        self.camera = None
        self.armature = None
        self.player = None
        self.other_player = None
        self.p1_addr = None
        self.p2_addr = None
        self.cam_addr = None
        
        self.allocated_frame_anim = None
        self.wrote_player_move = False
        self.preview_frame_anim = TekkenAnimation()
        
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
        self.applyPlayerCollision()
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
                if self.armature_name == None:
                    print("No armature selected")
                else:
                    self.armature = bpy.context.scene.objects[self.armature_name]
                    if self.tracking: self.trackPlayer()
                    elif self.preview: self.previewPlayer()
                    
                    if self.attach_player: self.attachPlayer()
                    
                if self.camera_preview and self.camera_name != None:
                    self.camera = bpy.context.scene.objects[self.camera_name]
                    self.previewCamera()
            except Exception:
                print(traceback.format_exc())
                self.stop()
                
            self.waitFrame()
            
        print("Loop exited")
    
    def waitFrame(self):
        sleep(self.update_delay)
        
        #currFrame = self.T.readInt(self.frame_counter, 4)
        #while self.T.readInt(self.frame_counter, 4) == currFrame:
        #    sleep(0.001)
    
    def stop(self):
        if not self.running: return
        #print("stop()")
        
        self.cam_addr = None
        self.allocated_frame_anim = None
        self.wrote_player_move = False
        self.running = False
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
        
    
    def applyPlayerCollision(self):
        self.T.writeInt(self.player + 0xae0, self.collision, 1)
        self.T.writeInt(self.other_player + 0xae0, self.collision, 1)
    
    # getPlayerYaw - Blender output
    def getPlayerYaw(self):
        return self.T.readInt(self.player + 0x1c0, 2) * (pi * 2 / 65535)
        
    # setPlayerYaw - Expects blender input
    def setPlayerYaw(self, yaw):
        while yaw > pi * 2:
            yaw -= pi * 2
        yaw = int((yaw - pi * 2) * (65535 / (pi * 2)))
        while yaw < 0: yaw += 65535
        self.T.writeInt(self.player + 0xEE, yaw, 2) #half circle offset
    
    # getPlayerPos - Tekken output
    def getPlayerPos(self):
        playerAddress = self.player
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
    def getArmaturePos(self):
        return self.armature.location
        
    # setArmaturePos - Expects blender input
    def setArmaturePos(self, x, y, z):
        self.armature.location = Vector((x, y, z))  #blender inverts x and y
        
    # getArmatureYaw - Blender output
    def getArmatureYaw(self):
        return self.armature.rotation_euler[2]
        
    # setArmatureYaw - Expects blender input
    def setArmatureYaw(self, yaw):
        self.armature.rotation_euler = Euler((pi / 2, 0.0, yaw), 'XYZ')
        
    # - #
    
    # getBlenderCameraProperties - Blender output
    def getBlenderCameraProperties(self):
    
        x, y, z = self.camera.location
        rot_x, rot_y, rot_z = self.camera.rotation_euler
        fov = self.camera.data.angle * 50
        
        return [x, y, z, rot_x, rot_y, rot_z, fov]
    
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
        
    # - Main functions - #
    
    def previewCamera(self):
        camData = self.getBlenderCameraProperties()
        # conversion
        
        self.setTekkenCameraProperties(*camData)
        
    def attachPlayer(self):
        localLocation = self.armature.pose.bones["BODY_SCALE__group"].location
    
        offset1 = -(localLocation[2] * 1000)
        offset2 = (localLocation[1]) * 1000
        offset3 = localLocation[0] * 1000
        
        playerPos = self.getPlayerPos()
        
        playerPos['x'] += offset1 + 2000 #?
        playerPos['y'] += offset3 + 2000 #?
        playerPos['z'] += offset2 + 2500 #hauteur
        
        self.setPlayerFloorHeight(self.other_player, playerPos['z'])
        self.setPlayerPos(self.other_player, playerPos['x'], playerPos['y'])
        
        
    def previewPlayer(self):
        #self.VR.setIKFromVR(self.armature)
    
        animFrame = getAnimFrameFromBones(self.armature)
        
        for i in range(self.preview_frame_anim.field_count):
            self.preview_frame_anim.setField(animFrame[i], 0, i)
            
        if not self.wrote_player_move and self.preview: self.writePlayerMove()
            
        self.T.writeBytes(self.allocated_frame_anim, bytes(self.preview_frame_anim.data))
        
        rot = self.getArmatureYaw()
        self.setPlayerYaw(rot)
        
        x, y, z = self.getArmaturePos()
        self.setPlayerPos(self.player, x * 1000, -y * 1000)
        self.setPlayerFloorHeight(self.player, z * 1000)
    
    def trackPlayer(self):
        pos = self.getPlayerPos()
        self.setArmaturePos((pos['x'] / 1000), -(pos['y'] / 1000), pos['z'] / 1000)
        
        yaw = self.getPlayerYaw()
        self.setArmatureYaw(yaw)
        
        animAddr = self.getPlayerAnimAddr()
        if animAddr != None and self.T.readInt(animAddr, 1) == 0xC8:
            frame = self.T.readInt(self.player + self.game_addresses["curr_frame_timer_offset"], 4)
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
            applyRotationFromAnimdata(self.armature, floats)
        
    # -  - #
    
    def getPlayerAddresses(self):
        self.p1_addr = self.game_addresses["t7_p1_addr"]
        self.p2_addr = self.p1_addr + self.game_addresses["t7_playerstruct_size"]
        self.player = self.p1_addr if self.playerid == 0 else self.p2_addr
        self.other_player = self.p1_addr if self.playerid == 1 else self.p2_addr
        
    def getCameraAddr(self):
        self.cam_addr = self.game_addresses['camera_ptr']
    
    def getFrameCounterAddr(self):
        if self.game_addresses['using_global_frame_counter']:
            self.frame_counter = self.game_addresses['global_frame_counter']
        else:
            self.frame_counter = self.game_addresses['game_frame_counter']
    
    def allocateFrameAnim(self):
        self.allocated_frame_anim = self.T.allocateMem(self.preview_frame_anim.size)
        print("Allocated new anim: 0x%x" % (self.allocated_frame_anim))

    def writePlayerMove(self):
        currmoveAddr = self.T.readInt(self.player + 0x220, 8)
        if currmoveAddr == 0: return
        self.T.writeInt(currmoveAddr + 0x10, self.allocated_frame_anim, 8)
        self.wrote_player_move = True
        
    def getPlayerAnimAddr(self):
        currmoveAddr = self.T.readInt(self.player + 0x220, 8)
        return None if currmoveAddr== 0 else self.T.readInt(currmoveAddr + 0x10, 8)
    
    def writeFloat(self, addr, value):
        self.T.writeBytes(addr, struct.pack('f', value))
        
    def readFloat(self, addr):
        return struct.unpack('f', self.T.readBytes(addr, 4))[0]
    
    # -Interactions- #
    
    def allowFreeHeadMovement(self):
        if self.running:
            playerAddress = self.player
            currmoveAddr = self.T.readInt(self.player + 0x220, 8)
            if currmoveAddr != 0 :
                self.T.writeInt(currmoveAddr + 0x98, 0, 4)
                return True
        return False
        
    def checkStatus(self):
        if False:
            self.stop()
            
    def setPlayerCollision(self, collision):
        self.collision = collision
        if self.running: self.applyPlayerCollision()
            
    def setMainPlayerFloorHeight(self, floorheight):
        if self.running:
            self.setPlayerFloorHeight(self.player, float(floorheight))
            
    def setTarget(self, playerid):
        if playerid == self.playerid: return
        self.playerid = playerid
        self.player = self.p1_addr if playerid == 0 else self.p2_addr
        self.other_player = self.p1_addr if playerid == 1 else self.p2_addr
    
    def setPreview(self, enabled):
        if enabled:
            self.preview = True
            self.tracking = False
            
            if not self.running:
                self.start()
        else:
            self.preview = False
            if not self.tracking and not self.camera_preview: self.stop()
    
    def setCameraPreview(self, enabled):
        if enabled:
            self.camera_preview = True
            self.tracking = False
            
            if not self.running:
                self.start()
                
            self.lockCamera()
        else:
            self.unlockCamera()
            self.camera_preview = False
            if not self.tracking and not self.preview: self.stop()
    
    def setTracking(self, enabled):
        if enabled:
            self.tracking = True
            self.preview = False
            
            if not self.running:
                self.start()
        else:
            self.tracking = False
            if not self.preview and not self.camera_preview: self.stop()
