from . Addresses import AddressFile, GameClass
from . TekkenAnimHelper import TekkenAnimation, getAnimFrameFromBones
from time import time, sleep
import bpy
import pathlib
import threading
import struct
from mathutils import Vector, Euler
from math import pi


class TKDirect: #(Singleton):
    def __init__(self):
        self.running = False
        self.game_addresses = AddressFile(pathlib.Path(__file__).parent.resolve().__str__() + "\\game_addresses.txt")
        
        self.update_delay = 1/60#0.1
        
        self.preview = False
        self.tracking = False
        self.attach_player = False
        
        self.single_frame_preview = True
        
        self.collision = 0
        self.armature_name = None
        self.playerid = 0
        
        self.armature = None
        self.player = None
        self.other_player = None
        self.p1_addr = None
        self.p2_addr = None
        
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
        self.applyPlayerCollision()
        self.allocateFrameAnim()
        
        
        
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
            except Exception as e:
                print(e)
                self.stop()
                break
            
            sleep(self.update_delay)
            
        print("Loop exited")
    
    def stop(self):
        if not self.running: return
        #print("stop()")
        
        self.allocated_frame_anim = None
        self.wrote_player_move = False
        self.running = False
        self.T.close()
        self.T = None
        
    # - Utils - #
    
    def applyPlayerCollision(self):
        self.T.writeInt(self.player + 0xae0, self.collision, 1)
        self.T.writeInt(self.other_player + 0xae0, self.collision, 1)
    
    # getPlayerYaw - Blender output
    def getPlayerYaw(self):
        return pi * 2 + self.T.readInt(self.player + 0x1c0, 2) * (pi * 2 / 65535) #half circle offset
        
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
            'z': 0,
        }
    """
    # setPlayerPos - Expects Tekken input
    def setPlayerPos(self, x, y, z): #tekken output
        playerAddress = self.player
        #x, y, z
        
        self.writeFloat(playerAddress + 0xFC, x)
        self.writeFloat(playerAddress + 0xF4, y)
        #self.writeFloat(playerAddress + , z)
    """
    
    # - #
    
    # getArmaturePos - Blender output
    def getArmaturePos(self):
        x, y, z = self.armature.location
        
    # getArmaturePos - Expects blender input
    def setArmaturePos(self, x, y, z):
        self.armature.location = Vector((x, y, z))  #blender inverts x and y
        
    # getArmaturePos - Blender output
    def getArmatureYaw(self):
        return self.armature.rotation_euler[2]
        
    # getArmaturePos - Expects blender input
    def setArmatureYaw(self, yaw):
        self.armature.rotation_euler = Euler((pi / 2, 0.0, yaw), 'XYZ')
        
    # - Main functions - #
        
    def attachPlayer(self):
        pass
        
    def previewPlayer(self):
        #we'll have to remove pos stuff
        #pos = self.getArmaturePos(x, y, z)
        #setPlayerPos(y * 1000, x * 1000, z * 1000)  #blender inverts x and y
        
        animFrame = getAnimFrameFromBones(self.armature)
        
        for i in range(self.preview_frame_anim.field_count):
            self.preview_frame_anim.setField(animFrame[i], 0, i)
            
        if not self.wrote_player_move and self.preview: self.writePlayerMove()
            
        self.T.writeBytes(self.allocated_frame_anim, bytes(self.preview_frame_anim.data))
        
        rot = self.getArmatureYaw()
        self.setPlayerYaw(rot)
    
    def trackPlayer(self):
        pos = self.getPlayerPos()
        self.setArmaturePos(-(pos['y'] / 1000), -(pos['x'] / 1000), pos['z'] / 1000)
        
        yaw = self.getPlayerYaw()
        self.setArmatureYaw(yaw)
    
    def getPlayerAddresses(self):
        self.p1_addr = self.game_addresses["t7_p1_addr"]
        self.p2_addr = self.p1_addr + self.game_addresses["t7_playerstruct_size"]
        self.player = self.p1_addr if self.playerid == 0 else self.p2_addr
        self.other_player = self.p1_addr if self.playerid == 1 else self.p2_addr
        
    # -- #
    
    def allocateFrameAnim(self):
        self.allocated_frame_anim = self.T.allocateMem(self.preview_frame_anim.size)
        print("Allocated new anim: 0x%x" % (self.allocated_frame_anim))

    def writePlayerMove(self):
        self.wrote_player_move = True
        currmoveAddr = self.T.readInt(self.player + 0x220, 8)
        self.T.writeInt(currmoveAddr + 0x10, self.allocated_frame_anim, 8)
    
    def writeFloat(self, addr, value):
        self.T.writeBytes(addr, struct.pack('f', value))
        
    def readFloat(self, addr):
        return struct.unpack('f', self.T.readBytes(addr, 4))[0]
    
    # -- #
        
    def checkStatus(self):
        if False:
            self.stop()
            
    def setPlayerCollision(self, collision):
        self.collision = collision
        if self.running: self.applyPlayerCollision()
            
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
            if not self.tracking: self.stop()
            
    def setArmature(self, armature):
        self.armature = armature
    
    def setTracking(self, enabled):
        if enabled:
            self.tracking = True
            self.preview = False
            
            if not self.running:
                self.start()
        else:
            self.tracking = False
            if not self.preview: self.stop()
