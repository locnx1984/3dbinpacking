from py3dbp import Packer, Bin, Item
import numpy as np
import open3d as o3d
import random

class Online3DPackingDataGenerator:
    def __init__(self):
        
        self.bin_W_range = [300,500]
        self.bin_H_range = [500,700]
        self.bin_L_range = [300,500]
        self.bin_L =60 #X   #random.uniform(self.bin_L_range[0],self.bin_L_range[1])
        self.bin_W =50 #Y   #random.uniform(self.bin_W_range[0],self.bin_W_range[1])
        self.bin_H =40 #Z   #random.uniform(self.bin_H_range[0],self.bin_H_range[1])
         
        self.item_L_range = [self.bin_L_range[0]*0.05,self.bin_L_range[1]*0.3]
        self.item_W_range = [self.bin_W_range[0]*0.05,self.bin_W_range[1]*0.3]
        self.item_H_range = [self.bin_H_range[0]*0.05,self.bin_H_range[1]*0.3]
        
        self.item_min_dim=5
        self.item_max_dim=20

        self.packer=None
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

        item_list=[]
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

            #SORT ITEMS BY Z POSITION
            b.items.sort(key=lambda x: (x.position[2],x.position[0]+x.position[1]), reverse=False) 

            #extend dimension
            for item in b.items:
                item.length, item.width, item.height =item.get_dimension()
                item.rotation_type=0

            #draw bin origin
            bin_origin=o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0, origin= [0., 0., 0.] )
            item_list.append(bin_origin)
            print(":::::::::::", b.string()) 
            mesh_bin = self.create_mesh_box(width = b.length, height = b.width, depth = b.height)#, dx=b.position[0], dy=b.position[1], dz=b.position[2])#position
            mesh_bin.compute_vertex_normals()
            mesh_bin.paint_uniform_color([random.uniform(0,1), random.uniform(0,1), random.uniform(0,1)])
            lineset_bin=o3d.geometry.LineSet.create_from_triangle_mesh(mesh_bin)
            item_list.append(lineset_bin)
            print("FITTED ITEMS:")
            for item in b.items:
                print("====> ", item.string())
                dim=item.get_dimension()
                mesh_box = self.create_mesh_box(width = dim[0], height = dim[1], depth = dim[2], dx=item.position[0], dy=item.position[1], dz=item.position[2])#position
                mesh_box.compute_vertex_normals()
                mesh_box.paint_uniform_color([random.uniform(0.2,0.8), random.uniform(0.2,0.8), random.uniform(0.2,0.8)])
                item_list.append(mesh_box)
            print("UNFITTED ITEMS:")
            for item in b.unfitted_items:
                print("====> ", item.string())

            print("***************************************************")
            print("***************************************************") 
        
        o3d.visualization.draw_geometries(item_list)

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
    def generate_data_test(sefl):

        packer = Packer()

        packer.add_bin(Bin('small-envelope', 11.5, 6.125, 0.25, 10))
        packer.add_bin(Bin('large-envelope', 15.0, 12.0, 0.75, 15))
        packer.add_bin(Bin('small-box', 8.625, 5.375, 1.625, 70.0))
        packer.add_bin(Bin('medium-box', 11.0, 8.5, 5.5, 70.0))
        packer.add_bin(Bin('medium-2-box', 13.625, 11.875, 3.375, 70.0))
        packer.add_bin(Bin('large-box', 12.0, 12.0, 5.5, 70.0))
        packer.add_bin(Bin('large-2-box', 23.6875, 11.75, 3.0, 70.0))

        packer.add_item(Item('50g [powder 1]', 3.9370, 1.9685, 1.9685, 1))
        packer.add_item(Item('50g [powder 2]', 3.9370, 1.9685, 1.9685, 2))
        packer.add_item(Item('50g [powder 3]', 3.9370, 1.9685, 1.9685, 3))
        packer.add_item(Item('250g [powder 4]', 7.8740, 3.9370, 1.9685, 4))
        packer.add_item(Item('250g [powder 5]', 7.8740, 3.9370, 1.9685, 5))
        packer.add_item(Item('250g [powder 6]', 7.8740, 3.9370, 1.9685, 6))
        packer.add_item(Item('250g [powder 7]', 7.8740, 3.9370, 1.9685, 7))
        packer.add_item(Item('250g [powder 8]', 7.8740, 3.9370, 1.9685, 8))
        packer.add_item(Item('250g [powder 9]', 7.8740, 3.9370, 1.9685, 9))

        packer.pack()

        for b in packer.bins:
            print(":::::::::::", b.string())

            print("FITTED ITEMS:")
            for item in b.items:
                print("====> ", item.string())

            print("UNFITTED ITEMS:")
            for item in b.unfitted_items:
                print("====> ", item.string())

            print("***************************************************")
            print("***************************************************")

#Start a test
generator=Online3DPackingDataGenerator()
generator.generate_data()
