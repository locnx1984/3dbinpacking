#!/usr/bin/env python 
# Generate dataset for Online-3D Bin Packing
# prepare training data following 2021 paper: Online 3D Bin Packing with Constrained Deep Reinforcement Learning - Hang Zhao1, Qijin She1, Chenyang Zhu1, Yin Yang2, Kai Xu1
# Link: https://arxiv.org/pdf/2006.14978.pdf
# Authors: Loc Nguyen 
# Solomon Technology Corp.
# Copyright - 2021 
# 
# The software contains proprietary information of Solomon Technology Corp.  
# It is provided under a license agreement containing restrictions on use and disclosure 
# and is also protected by copyright law. Reverse engineering of the software is prohibited. 
# 
# No part of this publication may be reproduced, stored in a retrieval system, 
# or transmitted in any form or by any means, electronic, mechanical, photocopying, recording or otherwise 
# without the prior written permission of Solomon Technology Corp. 
#  
from . import Packer, Bin, Item
import numpy as np
import open3d as o3d
import random 
from .auxiliary_methods import rect_intersect,intersect, set_to_decimal
from .constants import Axis
import time

class Online3DPackingDataGenerator:
    def __init__(self,_bin_L =60 ,_bin_W =50 ,_bin_H =40,_item_min_dim=5,_item_max_dim=25):
         
        #Bin Dimension
        self.bin_L =_bin_L #X   
        self.bin_W =_bin_W #Y    
        self.bin_H =_bin_H #Z   
        
        #Item dimension range
        self.item_min_dim=_item_min_dim
        self.item_max_dim=_item_max_dim

        #packing result
        self.packer=None
        self.online_items=None

        #display param
        self.vis=None
        self.box_id=0
        self.item_list=None
        
    def generate_data(self):
        self.packer = Packer()
       
        bin_volume=self.bin_W*self.bin_H*self.bin_L
        self.packer.add_bin(Bin('Bin',self.bin_L , self.bin_W, self.bin_H,  1))

        current_volume=0
        index=0
        while (current_volume<bin_volume): 
            index+=1
            item_W =random.randint(self.item_min_dim,self.item_max_dim)
            item_H =random.randint(self.item_min_dim,self.item_max_dim)
            item_L =random.randint(self.item_min_dim,self.item_max_dim)
            current_volume+=item_W*item_H*item_L
            self.packer.add_item(Item('Item '+str(index), item_W, item_H, item_L, 0))
        start=time.time()
        self.packer.pack()
        print("Packing...FINISHED. Time=",time.time()-start)   
            
        if (len(self.packer.bins)>0):
        #for b in self.packer.bins:
            b=self.packer.bins[0]
            print("================================================================================")
            print("number of items:",len(b.items))
            print("number of unplaced_items:",len(b.unplaced_items))
            print("number of unfitted_items:",len(b.unfitted_items))

            Zero_items=[item for item in b.items if (item.position[0]+item.position[1]+item.position[2]<=0.00001)]
            other_items=[item for item in b.items if (item.position[0]+item.position[1]+item.position[2]>0.00001)]
            b.items =[]
            if len(Zero_items)>0:
                b.items.append(Zero_items[0])
                Zero_items.pop(0)
            b.items.extend(other_items)
            b.unfitted_items.extend(Zero_items)
 
            print("AFTER number of items:",len(b.items))
            print("AFTER number of unplaced_items:",len(b.unplaced_items))
            print("AFTER number of unfitted_items:",len(b.unfitted_items))

            #SORT ITEMS BY Z POSITION then by X+Y
            self.online_items=b.items
            for item in self.online_items:
                l,w,h=item.get_dimension()
                item.length, item.width, item.height = l,w,h
                item.rotation_type=0

            self.online_items.sort(key=lambda x: (x.position[2]+ item.height,x.position[0]+x.position[1]), reverse=False) 
            #b.items.sort(key=lambda x: (x.position[2]*2+x.position[0]+x.position[1]), reverse=False) 
            #extend dimension
            start=time.time()
            print("Extending dimension...") 

            for i in range(len(self.online_items)):
                item1=self.online_items[i]
                min_length=0#item1.position[0]
                max_length=self.bin_L#item1.position[0]+item1.length
                min_width=0#item1.position[1] 
                max_width=self.bin_W#item1.position[1]+item1.width
                min_height=0#item1.position[2]
                max_height=self.bin_H#item1.position[2]+item1.height
 
                for j in range(len(self.online_items)):
                    if (j!=i):
                        item2=self.online_items[j]
                        #X:Length
                        if (rect_intersect(item1, item2, Axis.HEIGHT, Axis.WIDTH)):
                            if (item2.position[0]>=item1.position[0]+item1.length) and (item2.position[0]<max_length):
                                max_length=item2.position[0]
                            if (item2.position[0]+item2.length<=item1.position[0]) and (item2.position[0]+item2.length>min_length):
                                min_length=item2.position[0]+item2.length
                        #Y:Width
                        if (rect_intersect(item1, item2,Axis.LENGTH, Axis.HEIGHT)):
                            if (item2.position[1]>=item1.position[1]+item1.width) and (item2.position[1]<max_width):
                                max_width=item2.position[1]
                            if (item2.position[1]+item2.width<=item1.position[1]) and (item2.position[1]+item2.width>min_width):
                                min_width=item2.position[1]+item2.width
                        #Z:Height
                        if (rect_intersect(item1, item2, Axis.LENGTH, Axis.WIDTH)):
                            if (item2.position[2]>=item1.position[2]+item1.height) and (item2.position[2]<max_height):
                                max_height=item2.position[2]
                            if (item2.position[2]+item2.height<=item1.position[2]) and (item2.position[2]+item2.length>min_height):
                                min_height=item2.position[2]+item2.height
                
                item1.position[0]=min_length
                item1.length=max_length-min_length
                item1.position[1]=min_width
                item1.width=max_width-min_width
                item1.position[2]=min_height
                item1.height=max_height-min_height
            print("Extending dimension...FINISHED. Time=",time.time()-start)   
            
        print(":::::::::::", b.string()) 
        print("FITTED ITEMS:")
        for item in self.online_items:
            print(item.string()) 
        print("UNFITTED ITEMS:")
        for item in b.unfitted_items:
            print(item.string())

        print("***************************************************")
        print("***************************************************") 
         
    def pre_box_callback(self,vis):
        self.box_id=((self.box_id-1+len(self.online_items))%len(self.online_items))
        
        self.vis.clear_geometries()
        self.vis.add_geometry(self.item_list[0])
        self.vis.add_geometry(self.item_list[1])
        for id in range(0,self.box_id+1):
            self.vis.add_geometry(self.item_list[id+2])
        for id in range(self.box_id,len(self.online_items)):
            self.vis.remove_geometry(self.item_list[id+2])
         
        print("pre box! id=",self.box_id,"/",len(self.online_items))
    def next_box_callback(self,vis): 
        self.box_id=((self.box_id+1+len(self.online_items))%len(self.online_items))
       
        self.vis.clear_geometries()
        self.vis.add_geometry(self.item_list[0])
        self.vis.add_geometry(self.item_list[1])

        for id in range(0,self.box_id+1):
            self.vis.add_geometry(self.item_list[id+2])
        for id in range(self.box_id,len(self.online_items)):
            self.vis.remove_geometry(self.item_list[id+2])
         
        print("next box! id=",self.box_id,"/",len(self.online_items))
    def create_mesh_box(self,width, height, depth, dx=0, dy=0, dz=0):
        box = o3d.geometry.TriangleMesh()
        vertices = np.array([[0,0,0],
                            [width,0,0],
                            [0,0,depth],
                            [width,0,depth],
                            [0,height,0],
                            [width,height,0],
                            [0,height,depth],
                            [width,height,depth]])
        vertices[:,0] += dx
        vertices[:,1] += dy
        vertices[:,2] += dz
        triangles = np.array([[4,7,5],[4,6,7],[0,2,4],[2,6,4],
                            [0,1,2],[1,3,2],[1,5,7],[1,7,3],
                            [2,3,7],[2,7,6],[0,4,1],[1,4,5]])
        box.vertices = o3d.utility.Vector3dVector(vertices)
        box.triangles = o3d.utility.Vector3iVector(triangles)
        return box
    def generate_data_test(self,folder_name=""):
         
        print("generate training data...")
         
    def display_current_result(self):
        #draw bin origin 
        self.item_list=[]
        bin_origin=o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0, origin= [0., 0., 0.] )
        self.item_list.append(bin_origin)
        mesh_bin = self.create_mesh_box(width = self.bin_L, height = self.bin_W, depth = self.bin_H)#, dx=b.position[0], dy=b.position[1], dz=b.position[2])#position
        mesh_bin.compute_vertex_normals()
        mesh_bin.paint_uniform_color([random.uniform(0,1), random.uniform(0,1), random.uniform(0,1)])
        lineset_bin=o3d.geometry.LineSet.create_from_triangle_mesh(mesh_bin)
        self.item_list.append(lineset_bin)
         
        for item in self.online_items: 
            dim=item.get_dimension()
            mesh_box = self.create_mesh_box(width = dim[0], height = dim[1], depth = dim[2], dx=item.position[0], dy=item.position[1], dz=item.position[2])#position
            mesh_box.compute_vertex_normals()
            mesh_box.paint_uniform_color([random.uniform(0.2,0.8), random.uniform(0.2,0.8), random.uniform(0.2,0.8)])
            self.item_list.append(mesh_box)
          
        #o3d.visualization.draw_geometries(item_list)
        self.vis = o3d.visualization.VisualizerWithKeyCallback()#draw_geometries_with_editing 
        self.vis.create_window("Online 3D Bin Packing")  
        
        self.vis.register_key_callback(78,self.next_box_callback) #press N
        self.vis.register_key_callback(32,self.next_box_callback)
        self.vis.register_key_callback(80,self.pre_box_callback) #press P
        self.vis.register_key_callback(8,self.pre_box_callback) #press P
 
        for pcd1 in self.item_list:
            self.vis.add_geometry(pcd1)
        self.box_id=len(self.item_list)-1 
        self.vis.run()  # user picks points
