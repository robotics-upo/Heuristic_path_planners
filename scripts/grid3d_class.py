#!/usr/bin/env python3

import ifcopenshell
import ifcopenshell.util.placement
import ifcopenshell.util.selector
import ifcopenshell.geom
import sys
import matplotlib.pyplot as plt
import numpy as np
import math
import rospy
import rospkg
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout
from heuristic_planners.srv import *


class grid3d():
    def __init__(self):

        # Inicializacion del archivo IFC

        rospack = rospkg.RosPack()
        model_path = str(rospack.get_path('heuristic_planners') )+ str("/scripts/model.ifc")
        self.model = ifcopenshell.open(model_path)
        self.settings = ifcopenshell.geom.settings()
        self.settings.set(self.settings.USE_WORLD_COORDS, True)

        # Elementos a representar
        self.walls = self.model.by_type('IfcWall')
        self.doors = self.model.by_type('IfcDoor')
        self.columns = self.model.by_type('IfcColumn')

        rospy.loginfo("Termina carga de IFC")
        self.world_size_x = rospy.get_param('~world_size_x', 220)
        self.world_size_y = rospy.get_param('~world_size_y', 220)
        self.world_size_z = rospy.get_param('~world_size_z', 20)
        self.resolution = rospy.get_param('~resolution', 1)
        self.x_y_size = self.world_size_x * self.world_size_y

        self.semantic_grid_x = []
        self.semantic_grid_y = []
        self.semantic_grid_z = []
        self.semantic_grid_v = []

        rospy.loginfo("Termina carga de parametros")
        
        for wall in self.walls:
            
            matrix = ifcopenshell.util.placement.get_local_placement(wall.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, wall)

            
            points = np.array(shape.geometry.verts).reshape(-1, 3)
            if(len(points) > 8):
                points = self.calcular_extremos(points)
            
            
            
            points = points / self.resolution

            # Calcular valores máximos
            max_x_local = int(np.max(points[:, 0]))
            max_y_local = int(np.max(points[:, 1]))
            max_z_local = int(np.max(points[:, 2]))

            # Calcular valores minimos
            min_x_local = int(np.min(points[:, 0]))
            min_y_local = int(np.min(points[:, 1]))
            min_z_local = int(np.min(points[:, 2]))

            
            # Bucles for para recorrer el espacio entre mínimos y máximos en cada coordenada
            for x in range(min_x_local, max_x_local + 1):
                for y in range(min_y_local, max_y_local + 1):
                    for z in range(min_z_local, max_z_local + 1):
                            self.semantic_grid_x.append(int(x))
                            self.semantic_grid_y.append(int(y))
                            self.semantic_grid_z.append(int(z))
                            self.semantic_grid_v.append(int(1))
                         
            
        for door in self.doors:
            matrix = ifcopenshell.util.placement.get_local_placement(door.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, door)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
                
            # Sumar el centro a las coordenadas de los puntos
            points = points / self.resolution
            

         # Calcular valores máximos
            max_x_local = int(np.max(points[:, 0]))
            max_y_local = int(np.max(points[:, 1]))
            max_z_local = int(np.max(points[:, 2]))

            # Calcular valores minimos
            min_x_local = int(np.min(points[:, 0]))
            min_y_local = int(np.min(points[:, 1]))
            min_z_local = int(np.min(points[:, 2]))

            for x in range(min_x_local, max_x_local + 1):
                for y in range(min_y_local, max_y_local + 1):
                    for z in range(min_z_local, max_z_local + 1):
                        self.semantic_grid_x.append(int(x))
                        self.semantic_grid_y.append(int(y))
                        self.semantic_grid_z.append(int(z))
                        self.semantic_grid_v.append(int(2))
                        
                        
    
        for column in self.columns:
            matrix = ifcopenshell.util.placement.get_local_placement(column.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, column)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la puerta
            
            # Sumar el centro a las coordenadas de los puntos
            points = points / self.resolution
            
            # Calcular valores máximos
            max_x_local = int(np.max(points[:, 0]))
            max_y_local = int(np.max(points[:, 1]))
            max_z_local = int(np.max(points[:, 2]))

            # Calcular valores minimos
            min_x_local = int(np.min(points[:, 0]))
            min_y_local = int(np.min(points[:, 1]))
            min_z_local = int(np.min(points[:, 2]))

            for x in range(min_x_local, max_x_local + 1):
                for y in range(min_y_local, max_y_local + 1):
                    for z in range(min_z_local, max_z_local + 1):
                        self.semantic_grid_x.append(int(x))
                        self.semantic_grid_y.append(int(y))
                        self.semantic_grid_z.append(int(z))
                        self.semantic_grid_v.append(int(3))
                        

        for furnishing in self.model.by_type("IfcFurnishingElement"):
            matrix = ifcopenshell.util.placement.get_local_placement(furnishing.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, furnishing)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la puerta
            # Sumar el centro a las coordenadas de los puntos
            points = points / self.resolution
            
        

            # Calcular valores máximos
            max_x_local = int(np.max(points[:, 0]))
            max_y_local = int(np.max(points[:, 1]))
            max_z_local = int(np.max(points[:, 2]))

            # Calcular valores minimos
            min_x_local = int(np.min(points[:, 0]))
            min_y_local = int(np.min(points[:, 1]))
            min_z_local = int(np.min(points[:, 2]))

            for x in range(min_x_local, max_x_local + 1):
                for y in range(min_y_local, max_y_local + 1):
                    for z in range(min_z_local, max_z_local + 1):
                        self.semantic_grid_x.append(int(x))
                        self.semantic_grid_y.append(int(y))
                        self.semantic_grid_z.append(int(z))
                        self.semantic_grid_v.append(int(4))
                        
                        
    
        for stair in self.model.by_type("IfcStairFlight"):
            matrix = ifcopenshell.util.placement.get_local_placement(stair.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, stair)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la puerta
            
            # Sumar el centro a las coordenadas de los puntos
            points = points / self.resolution
            
            # Calcular valores máximos
            max_x_local = int(np.max(points[:, 0]))
            max_y_local = int(np.max(points[:, 1]))
            max_z_local = int(np.max(points[:, 2]))

            # Calcular valores minimos
            min_x_local = int(np.min(points[:, 0]))
            min_y_local = int(np.min(points[:, 1]))
            min_z_local = int(np.min(points[:, 2]))

            for x in range(min_x_local, max_x_local + 1):
                for y in range(min_y_local, max_y_local + 1):
                    for z in range(min_z_local, max_z_local + 1):
                        self.semantic_grid_x.append(int(x))
                        self.semantic_grid_y.append(int(y))
                        self.semantic_grid_z.append(int(z))
                        self.semantic_grid_v.append(int(5))
                        
                            

        for plate in self.model.by_type("IfcPlate"):
            matrix = ifcopenshell.util.placement.get_local_placement(plate.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, plate)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la puerta
            
            # Sumar el centro a las coordenadas de los puntos
            points = points / self.resolution
            
        

            # Calcular valores máximos
            max_x_local = int(np.max(points[:, 0]))
            max_y_local = int(np.max(points[:, 1]))
            max_z_local = int(np.max(points[:, 2]))

            # Calcular valores minimos
            min_x_local = int(np.min(points[:, 0]))
            min_y_local = int(np.min(points[:, 1]))
            min_z_local = int(np.min(points[:, 2]))

            for x in range(min_x_local, max_x_local + 1):
                for y in range(min_y_local, max_y_local + 1):
                    for z in range(min_z_local, max_z_local + 1):
                        self.semantic_grid_x.append(int(x))
                        self.semantic_grid_y.append(int(y))
                        self.semantic_grid_z.append(int(z))
                        self.semantic_grid_v.append(int(6))
                        
                        

        for lamp in self.model.by_type("IfcFlowTerminal"):
            shape = ifcopenshell.geom.create_shape(self.settings, lamp)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la puerta
            
            # Sumar el centro a las coordenadas de los puntos
            points = points / self.resolution
            
        

            # Calcular valores máximos
            max_x_local = int(np.max(points[:, 0]))
            max_y_local = int(np.max(points[:, 1]))
            max_z_local = int(np.max(points[:, 2]))

            # Calcular valores minimos
            min_x_local = int(np.min(points[:, 0]))
            min_y_local = int(np.min(points[:, 1]))
            min_z_local = int(np.min(points[:, 2]))

            for x in range(min_x_local, max_x_local + 1):
                for y in range(min_y_local, max_y_local + 1):
                    for z in range(min_z_local, max_z_local + 1):
                        self.semantic_grid_x.append(int(x))
                        self.semantic_grid_y.append(int(y))
                        self.semantic_grid_z.append(int(z))
                        self.semantic_grid_v.append(int(7))

                        

        for glass in ifcopenshell.util.selector.filter_elements(self.model, "IfcElement, material=Glass"):
            matrix = ifcopenshell.util.placement.get_local_placement(glass.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, glass)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la puerta
            
            # Sumar el centro a las coordenadas de los puntos
            points = points / self.resolution
            
        

            # Calcular valores máximos
            max_x_local = int(np.max(points[:, 0]))
            max_y_local = int(np.max(points[:, 1]))
            max_z_local = int(np.max(points[:, 2]))

            # Calcular valores minimos
            min_x_local = int(np.min(points[:, 0]))
            min_y_local = int(np.min(points[:, 1]))
            min_z_local = int(np.min(points[:, 2]))

            for x in range(min_x_local, max_x_local + 1):
                for y in range(min_y_local, max_y_local + 1):
                    for z in range(min_z_local, max_z_local + 1):
                        self.semantic_grid_x.append(int(x))
                        self.semantic_grid_y.append(int(y))
                        self.semantic_grid_z.append(int(z))
                        self.semantic_grid_v.append(int(8))

        # Obtén el valor mínimo de cada array
        min_x = np.min(self.semantic_grid_x)
        min_y = np.min(self.semantic_grid_y)
        min_z = np.min(self.semantic_grid_z)


        # Verifica si el valor mínimo es negativo y suma su valor absoluto a todos los elementos
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

            print(f'{xx},{yx},{zx},{v}')
            
            self.semantic_grid_x[i] = int(index)

            


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
    def calcular_puntos_medios(self, points, n=1):
        new_points = list(points)
        for _ in range(n):
            temp_points = list(new_points)
            for i in range(len(points)):
                p1 = temp_points[i]
                p2 = temp_points[(i + 1) % len(points)]  # El último punto se conecta con el primero
                medio = (p1 + p2) / 2.0
                # Verificar si el punto medio ya está en la lista antes de agregarlo
                if not any(np.array_equal(medio, p) for p in new_points):
                    new_points.append(medio)
        return np.array(new_points)
    def calcular_centro(self, vertices):
        vertices = np.array(vertices).reshape(-1, 3)
        centro = np.mean(vertices, axis=0)
        return centro
    def getWorldIndex(self, x, y, z):

        return z * self.x_y_size + y * self.world_size_x + x 
    def getDiscreteWorldPositionFromIndex(self, index):

        z = index / self.x_y_size
        ind = index - (z * self.x_y_size)
        y = ind / self.world_size_x
        x = ind % self.world_size_x

        return (x, y, z)
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
