#!/usr/bin/env python3

import ifcopenshell
import ifcopenshell.util.placement
import ifcopenshell.util.selector
import ifcopenshell.geom
import sys
import matplotlib.pyplot as plt
import numpy as np
import rospy
import rospkg
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout
from IfcGrid.srv import *

class grid3d():
    def __init__(self):

        # Inicializacion del archivo IFC

        rospack = rospkg.RosPack()
        model_path = str(rospack.get_path('heuristic_planners') )+ str("/scripts/model.ifc")
        self.model = ifcopenshell.open(model_path)
        self.settings = ifcopenshell.geom.settings()

        # Elementos a representar
        self.walls = self.model.by_type('IfcWallStandardCase')
        self.doors = self.model.by_type('IfcDoor')
        self.columns = self.model.by_type('IfcColumn')

        rospy.loginfo("Termina carga de IFC")
        self.world_size_x = rospy.get_param('~world_size_x', 220)
        self.world_size_y = rospy.get_param('~world_size_y', 220)
        self.world_size_z = rospy.get_param('~world_size_z', 20)
        self.resolution = rospy.get_param('~resolution', 0.2)
        self.x_y_size = self.world_size_x * self.world_size_y

        self.semantic_grid = np.zeros((int(self.world_size_x / self.resolution), int(self.world_size_y / self.resolution), int(self.world_size_z / self.resolution)), dtype=int)
        self.occ_grid = self.semantic_grid

        rospy.loginfo("Termina carga de parametros")

        for wall in self.walls:
            matrix = ifcopenshell.util.placement.get_local_placement(wall.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, wall)

            
            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la pared
            centro = matrix[:,3][:3]

            rotation_matrix  = matrix[:3, :3]

            
            points = np.dot(points, rotation_matrix)

            # Sumar el centro a las coordenadas de los puntos para obtener su posicion global
            points = (points + centro) / self.resolution
            

            if(len(points) > 8):
                points = self.calcular_extremos(points)
            
            
            points = np.round(points)  # Redondear a un decimal
            
            
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
                        if(self.semantic_grid[int(x), int(y), int(z)] == 0):
                            self.semantic_grid[x, y, z] = 1
                            self.occ_grid[x, y, z] = 1
                        #rospy.loginfo("Pared rellenada")

        for door in self.doors:
            matrix = ifcopenshell.util.placement.get_local_placement(door.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, door)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la puerta
            centro = matrix[:,3][:3]
            
            rotation_matrix  = matrix[:3, :3]

            points = np.dot(points, rotation_matrix)
            # Sumar el centro a las coordenadas de los puntos
            points = (points + centro) / self.resolution
            points = np.round(points)  # Redondear a un decimal

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
                        self.semantic_grid[x, y, z] = 2
                        self.occ_grid[x, y, z] = 1
                        
    
        for column in self.columns:
            matrix = ifcopenshell.util.placement.get_local_placement(column.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, column)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la puerta
            centro = matrix[:,3][:3]
            
            rotation_matrix  = matrix[:3, :3]

            points = np.dot(points, rotation_matrix)
            # Sumar el centro a las coordenadas de los puntos
            points = (points + centro) / self.resolution
            points = np.round(points)  # Redondear a un decimal
        

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
                        self.semantic_grid[x, y, z] = 3
                        self.occ_grid[x, y, z] = 1

        for furnishing in self.model.by_type("IfcFurnishingElement"):
            matrix = ifcopenshell.util.placement.get_local_placement(furnishing.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, furnishing)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la puerta
            centro = matrix[:,3][:3]
            
            rotation_matrix  = matrix[:3, :3]

            points = np.dot(points, rotation_matrix)
            # Sumar el centro a las coordenadas de los puntos
            points = (points + centro) / self.resolution
            points = np.round(points)  # Redondear a un decimal
        

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
                        self.semantic_grid[x, y, z] = 4
                        self.occ_grid[x, y, z] = 1
                        
    
        for stair in self.model.by_type("IfcStairFlight"):
            matrix = ifcopenshell.util.placement.get_local_placement(stair.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, stair)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la puerta
            centro = matrix[:,3][:3]
            
            rotation_matrix  = matrix[:3, :3]

            points = np.dot(points, rotation_matrix)
            # Sumar el centro a las coordenadas de los puntos
            points = (points + centro) / self.resolution
            points = np.round(points)  # Redondear a un decimal
        

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
                            self.semantic_grid[x, y, z] = 5
                            self.occ_grid[x, y, z] = 1

        for plate in self.model.by_type("IfcPlate"):
            matrix = ifcopenshell.util.placement.get_local_placement(plate.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, plate)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la puerta
            centro = matrix[:,3][:3]
            
            rotation_matrix  = matrix[:3, :3]

            points = np.dot(points, rotation_matrix)
            # Sumar el centro a las coordenadas de los puntos
            points = (points + centro) / self.resolution
            points = np.round(points)  # Redondear a un decimal
        

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
                        self.semantic_grid[x, y, z] = 6
                        self.occ_grid[x, y, z] = 1

        for lamp in self.model.by_type("IfcFlowTerminal"):
            matrix = ifcopenshell.util.placement.get_local_placement(lamp.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, lamp)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la puerta
            centro = matrix[:,3][:3]
            
            rotation_matrix  = matrix[:3, :3]

            points = np.dot(points, rotation_matrix)
            # Sumar el centro a las coordenadas de los puntos
            points = (points + centro) / self.resolution
            points = np.round(points)  # Redondear a un decimal
        

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
                        self.semantic_grid[x, y, z] = 7
                        self.occ_grid[x, y, z] = 1

        for glass in ifcopenshell.util.selector.filter_elements(self.model, "IfcElement, material=Glass"):
            matrix = ifcopenshell.util.placement.get_local_placement(glass.ObjectPlacement)
            shape = ifcopenshell.geom.create_shape(self.settings, glass)

            points = np.array(shape.geometry.verts).reshape(-1, 3)
            
            # Calcular el centro de la puerta
            centro = matrix[:,3][:3]
            
            rotation_matrix  = matrix[:3, :3]

            points = np.dot(points, rotation_matrix)
            # Sumar el centro a las coordenadas de los puntos
            points = (points + centro) / self.resolution
            points = np.round(points)  # Redondear a un decimal
        

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
                            self.semantic_grid[x, y, z] = 8
                            self.occ_grid[x, y, z] = 1
    rospy.loginfo("Termina de rellenar las matrices")

    # def showgrid(self):
    #     mlab.figure(bgcolor=(1, 1, 1))  # Configura el color de fondo
    #     contour = mlab.contour3d(self.occ_grid, color=(1, 0.2, 0.2), opacity=0.5, transparent=True)
    #     mlab.colorbar(contour, title='Valor')
    #     mlab.show()

    def get_semantic_grid(self, request):
        response = GetSemanticGridResponse()
        layout = self.create_layout()
        response.shape = [dim.size for dim in layout.dim]
        response.semantic_grid = self.semantic_grid.flatten().tolist()
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
        # g.showgrid()
        rospy.loginfo("Semantic Grid Avaliable")
        print("Semantic Grid Avaliable")
        service = rospy.Service('get_semantic_grid', GetSemanticGrid, g.get_semantic_grid)
        rospy.spin()


    except Exception as e:
        rospy.loginfo(e)
        rospy.loginfo(f"Error: {e}")

