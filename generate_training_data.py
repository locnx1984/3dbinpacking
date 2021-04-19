from py3dbp import Packer, Bin, Item
import numpy as np
import open3d as o3d
import random 
from py3dbp.auxiliary_methods import rect_intersect,intersect, set_to_decimal
from py3dbp.constants import Axis
import time
class Online3DPackingDataGenerator:
    def __init__(self):
        
        # self.bin_W_range = [300,500]
        # self.bin_H_range = [500,700]
        # self.bin_L_range = [300,500]
        self.bin_L =60 #X   #random.uniform(self.bin_L_range[0],self.bin_L_range[1])
        self.bin_W =50 #Y   #random.uniform(self.bin_W_range[0],self.bin_W_range[1])
        self.bin_H =40 #Z   #random.uniform(self.bin_H_range[0],self.bin_H_range[1])
         
        # self.item_L_range = [self.bin_L_range[0]*0.05,self.bin_L_range[1]*0.3]
        # self.item_W_range = [self.bin_W_range[0]*0.05,self.bin_W_range[1]*0.3]
        # self.item_H_range = [self.bin_H_range[0]*0.05,self.bin_H_range[1]*0.3]
        
        self.item_min_dim=5
        self.item_max_dim=20

        self.packer=None
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
        
        self.packer.pack()

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
            b.items.sort(key=lambda x: (x.position[2],x.position[0]+x.position[1]), reverse=False) 
            #b.items.sort(key=lambda x: (x.position[2]*2+x.position[0]+x.position[1]), reverse=False) 
            #extend dimension
            start=time.time()
            print("Extending dimension...")

            for item in b.items:
                item.length, item.width, item.height =item.get_dimension()
                item.rotation_type=0

            for i in range(len(b.items)):
                item1=b.items[i]
                min_length=0#item1.position[0]
                max_length=self.bin_L#item1.position[0]+item1.length
                min_width=0#item1.position[1] 
                max_width=self.bin_W#item1.position[1]+item1.width
                min_height=0#item1.position[2]
                max_height=self.bin_H#item1.position[2]+item1.height
 
                for j in range(len(b.items)):
                    if (j!=i):
                        item2=b.items[j]
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
            
            #draw bin origin 
            self.item_list=[]
            bin_origin=o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0, origin= [0., 0., 0.] )
            self.item_list.append(bin_origin)
            print(":::::::::::", b.string()) 
            mesh_bin = self.create_mesh_box(width = b.length, height = b.width, depth = b.height)#, dx=b.position[0], dy=b.position[1], dz=b.position[2])#position
            mesh_bin.compute_vertex_normals()
            mesh_bin.paint_uniform_color([random.uniform(0,1), random.uniform(0,1), random.uniform(0,1)])
            lineset_bin=o3d.geometry.LineSet.create_from_triangle_mesh(mesh_bin)
            self.item_list.append(lineset_bin)
            print("FITTED ITEMS:")
            for item in b.items:
                print("====> ", item.string())
                dim=item.get_dimension()
                mesh_box = self.create_mesh_box(width = dim[0], height = dim[1], depth = dim[2], dx=item.position[0], dy=item.position[1], dz=item.position[2])#position
                mesh_box.compute_vertex_normals()
                mesh_box.paint_uniform_color([random.uniform(0.2,0.8), random.uniform(0.2,0.8), random.uniform(0.2,0.8)])
                self.item_list.append(mesh_box)
            print("UNFITTED ITEMS:")
            for item in b.unfitted_items:
                print("====> ", item.string())

            print("***************************************************")
            print("***************************************************") 
        
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
        # self.vis.destroy_window() 
    def pre_box_callback(self,vis):
        self.box_id=((self.box_id-1+len(self.item_list))%len(self.item_list))
        #skip 2 drawing components
        if (self.box_id<2):
            self.box_id+=2
        self.vis.clear_geometries()
        self.vis.add_geometry(self.item_list[0])
        self.vis.add_geometry(self.item_list[1])
        for id in range(2,self.box_id+1):
            self.vis.add_geometry(self.item_list[id])
        for id in range(self.box_id,len(self.item_list)):
            self.vis.remove_geometry(self.item_list[id])
         
        print("pre box! id=",self.box_id-2,"/",str(len(self.item_list)-2))

    def next_box_callback(self,vis): 
        self.box_id=((self.box_id+1+len(self.item_list))%len(self.item_list))
        #skip 2 drawing components
        if (self.box_id<2):
            self.box_id+=2
        self.vis.clear_geometries()
        self.vis.add_geometry(self.item_list[0])
        self.vis.add_geometry(self.item_list[1])

        for id in range(2,self.box_id+1):
            self.vis.add_geometry(self.item_list[id])
        for id in range(self.box_id,len(self.item_list)):
            self.vis.remove_geometry(self.item_list[id])
         
        print("next box! id=",self.box_id-2,"/",str(len(self.item_list)-2))

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

#Start a test
generator=Online3DPackingDataGenerator()
generator.generate_data()