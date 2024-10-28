#!/usr/bin/env python3

import ifcopenshell
import ifcopenshell.util.placement
import ifcopenshell.util.selector
import ifcopenshell.util.shape
import ifcopenshell.geom
import sys
import matplotlib.pyplot as plt
import numpy as np
import math
import rospy
import rospkg
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout
# from heuristic_planners.srv import *
from grid_message.srv import *

class grid3d():
    def __init__(self):

        # Inicializacion del archivo IFC

        rospack = rospkg.RosPack()
        # model_path = str(rospack.get_path('heuristic_planners') )+ str("/scripts/model.ifc")
        model_path = str(rospack.get_path('heuristic_planners') )+ str("/scripts/atlas_1F.ifc")
        # model_path = str(rospack.get_path('heuristic_planners') )+ str("/scripts/casoplonv2.ifc")
        self.model = ifcopenshell.open(model_path)
        self.settings = ifcopenshell.geom.settings()
        self.settings.set(self.settings.USE_WORLD_COORDS, True)

        # Elementos a representar
        self.walls = self.model.by_type('IfcWall')
        self.doors = self.model.by_type('IfcDoor')
        self.columns = self.model.by_type('IfcColumn')
        self.furniture = self.model.by_type("IfcFurnishingElement")
        self.stairs = self.model.by_type("IfcStairFlight")
        self.plates = self.model.by_type("IfcPlate")
        self.lamps = self.model.by_type("IfcFlowTerminal")
        self.slabs = self.model.by_type("IfcSlab")
        self.windows = self.model.by_type("IfcWindow")
        # self.glasses = ifcopenshell.util.selector.filter_elements(self.model, "IfcElement, material=Glass")

        # self.semantic_objects = [self.walls, self.doors, self.columns, self.furniture, self.stairs, self.plates, self.lamps, self.glasses]
        self.semantic_objects = [self.walls, self.doors, self.columns, self.furniture, self.stairs, self.plates, self.lamps]

        rospy.loginfo("Termina carga de IFC")
        # Model Atlas
        # self.world_size_x = rospy.get_param('~world_size_x', 220)
        # self.world_size_y = rospy.get_param('~world_size_y', 220)
        # self.world_size_z = rospy.get_param('~world_size_z', 20)
        # self.resolution = rospy.get_param('~resolution', 0.2)

        # Model casoplonv2
        self.world_size_x = rospy.get_param('~world_size_x', 220)
        self.world_size_y = rospy.get_param('~world_size_y', 220)
        self.world_size_z = rospy.get_param('~world_size_z', 20)
        self.resolution = rospy.get_param('~resolution', 0.2)

        self.x_y_size = self.world_size_x * self.world_size_y

        self.semantic_grid_x = []
        self.semantic_grid_y = []
        self.semantic_grid_z = []
        self.semantic_grid_v = []

        rospy.loginfo("Termina carga de parametros")
        for elements in self.semantic_objects:
            index = self.semantic_objects.index(elements)
            for e in elements:
                
                shape = ifcopenshell.geom.create_shape(self.settings, e)
                verts = ifcopenshell.util.shape.get_vertices(shape.geometry)
                faces = ifcopenshell.util.shape.get_faces(shape.geometry)

                
                verts = verts / self.resolution
                verts = verts.astype(int)
                
                for face in faces:

                    v1 = verts[face[0]]
                    v2 = verts[face[1]]
                    v3 = verts[face[2]]

                    # Update maximum values
                    max_x = max(v1[0], v2[0], v3[0])
                    max_y = max(v1[1], v2[1], v3[1])
                    max_z = max(v1[2], v2[2], v3[2])

                    # Update minimum values
                    min_x = min(v1[0], v2[0], v3[0])
                    min_y = min(v1[1], v2[1], v3[1])
                    min_z = min(v1[2], v2[2], v3[2])

                    for x in range(min_x, max_x + 1):
                        for y in range(min_y, max_y + 1):
                            for z in range(min_z, max_z + 1):
                                # print(f'{x},{y},{z},1')
                                self.semantic_grid_x.append(int(x))
                                self.semantic_grid_y.append(int(y))
                                self.semantic_grid_z.append(int(z))
                                self.semantic_grid_v.append(int(index + 1))


                
                

        # Obtén el valor mínimo de cada array
        min_x = np.min(self.semantic_grid_x)
        min_y = np.min(self.semantic_grid_y)
        min_z = np.min(self.semantic_grid_z)


        # Verifica si el valor mínimo es negativo y suma su valor absoluto a todos los elementos
        self.semantic_grid_x -= abs(min_x)
        self.semantic_grid_y -= abs(min_y)
        self.semantic_grid_z -= abs(min_z)
        if min_x < 0:
            print(f'Hay offset de X: {min_x}')
            self.semantic_grid_x += abs(min_x)

        if min_y < 0:
            print(f'Hay offset de Y: {min_y}')
            self.semantic_grid_y += abs(min_y)

        if min_z < 0:
            print(f'Hay offset de Z: {min_z}')
            self.semantic_grid_z += abs(min_z)

        for i in range(len(self.semantic_grid_x)):
            x = self.semantic_grid_x[i]
            y = self.semantic_grid_y[i]
            z = self.semantic_grid_z[i]
            v = self.semantic_grid_v[i]


            index=x+(y*(self.world_size_x/self.resolution))+(z*((self.world_size_x/self.resolution)*(self.world_size_y/self.resolution)))
            
            zx = int(index / ((self.world_size_x/self.resolution)*(self.world_size_y/self.resolution)))
            ind = index - ( zx * ((self.world_size_x/self.resolution)*(self.world_size_y/self.resolution)))
            yx = int(ind / (self.world_size_x / self.resolution))
            xx = int(ind % (self.world_size_x / self.resolution))

            # print(f'{xx},{yx},{zx},{v}')
            
            self.semantic_grid_x[i] = int(index)

            
        print(len(self.semantic_grid_x))
        print((self.world_size_x/self.resolution)*(self.world_size_y/self.resolution)*(self.world_size_z/self.resolution))

    rospy.loginfo("Termina de rellenar las matrices")

    
    def get_semantic_grid(self, request):

        response = GetSemanticGridResponse()
        layout = self.create_layout()
        response.shape = [dim.size for dim in layout.dim]
        response.semantic_grid_x= self.semantic_grid_x
        response.semantic_grid_y= self.semantic_grid_y
        response.semantic_grid_z= self.semantic_grid_z
        response.semantic_grid_v= self.semantic_grid_v

        return response
    
    def calcular_extremos(self, array):
    # Obtener los valores máximos y mínimos en cada dimensión
        min_x = np.min(array[:,0])
        max_x = np.max(array[:,0])
        min_y = np.min(array[:,1])
        max_y = np.max(array[:,1])
        min_z = np.min(array[:,2])
        max_z = np.max(array[:,2])

        # Crear arrays para los mínimos y máximos de cada dimensión
        extremos_x = np.array([min_x, max_x])
        extremos_y = np.array([min_y, max_y])
        extremos_z = np.array([min_z, max_z])

        # Crear combinaciones de puntos
        combinaciones = []

        for x in extremos_x:
            for y in extremos_y:
                for z in extremos_z:
                    combinaciones.append([x, y, z])

        return np.array(combinaciones)
    
    def create_layout(self):
        layout = MultiArrayLayout()

        # Create a dimension for the first axis (x)
        dim_x = MultiArrayDimension()
        dim_x.label = "x"
        dim_x.size = int(self.world_size_x / self.resolution)
        dim_x.stride = self.x_y_size
        layout.dim.append(dim_x)

        # Create a dimension for the second axis (y)
        dim_y = MultiArrayDimension()
        dim_y.label = "y"
        dim_y.size = int(self.world_size_y / self.resolution)
        dim_y.stride = self.world_size_x
        layout.dim.append(dim_y)

        # Create a dimension for the third axis (z)
        dim_z = MultiArrayDimension()
        dim_z.label = "z"
        dim_z.size = int(self.world_size_z / self.resolution)
        dim_z.stride = 1  # Since it's the innermost dimension
        layout.dim.append(dim_z)

        layout.data_offset = 0  # This is usually 0

        return layout
    
if __name__ == '__main__':
    try:
        rospy.init_node("semantic_grid", anonymous=False)

        g = grid3d()
        rospy.loginfo("Semantic Grid Avaliable")
        print("Semantic Grid Avaliable")
        service = rospy.Service('get_semantic_grid', GetSemanticGrid, g.get_semantic_grid)
        rospy.spin()


    except Exception as e:
        rospy.loginfo(e)
        rospy.loginfo(f"Error: {e}")
