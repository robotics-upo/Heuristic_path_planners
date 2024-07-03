import multiprocessing
import ifcopenshell
import ifcopenshell.geom
import numpy as np
from collections import defaultdict
import open3d as o3d
import json

# Constants
FILENAME = 'nube_atlas_recortado.pcd'
RES = 0.1
GRID_SIZEX = 220
GRID_SIZEY = 220
GRID_SIZEZ = 20

# IFC File Handling
def setup_ifc_geometry(ifc_file_path):
    ifc_file = ifcopenshell.open(ifc_file_path)
    settings = ifcopenshell.geom.settings()
    settings.set(settings.DISABLE_OPENING_SUBTRACTIONS, False)
    settings.set(settings.USE_WORLD_COORDS, True)
    return ifc_file, settings

def initialize_iterator(settings, ifc_file):
    iterator = ifcopenshell.geom.iterator(settings, ifc_file, multiprocessing.cpu_count())
    tree = ifcopenshell.geom.tree()
    if iterator.initialize():
        while True:
            shape = iterator.get_native()
            tree.add_element(shape)
            if not iterator.next():
                break
    return tree

# Point Cloud Processing
def load_point_cloud(filename):
    pcd = o3d.io.read_point_cloud(filename)
    return np.asarray(pcd.points)

def point2grid(x, y, z, onedivres, grid_stepy, grid_stepz):
    index = np.floor(x * onedivres) + np.floor(y * onedivres) * grid_stepy + np.floor(z * onedivres) * grid_stepz
    return int(index)

# Grid Initialization and Processing
def process_points(points, tree, onedivres, grid_stepy, grid_stepz):
    result_dict = defaultdict(int)
    size = int(np.floor(GRID_SIZEX * onedivres) * np.floor(GRID_SIZEY * onedivres) * np.floor(GRID_SIZEZ * onedivres))
    
    # Create grids for storing assigned integers and zeros
    semantic_grid_ints = np.zeros(size)
    semantic_grid_zeros = np.zeros(size)
    
   
    global_id_to_int = {}
    current_int = 0

    for x, z, y in points:
        search_point = (float(x), float(y) - 68.6, float(z) - 1)
        elements = tree.select(search_point, extend=0.01)

        if elements:
            for item in elements:
                result_dict[item.GlobalId] += 1

                if item.GlobalId not in global_id_to_int:
                    global_id_to_int[item.GlobalId] = current_int
                    current_int += 1

                assigned_int = global_id_to_int[item.GlobalId]
                index = point2grid(x, y, z, onedivres, grid_stepy, grid_stepz)
                semantic_grid_ints[index] = assigned_int
                print(f'{x},{y},{z}')

    return result_dict, global_id_to_int, semantic_grid_ints, semantic_grid_zeros

# Saving Results
def save_results(global_id_to_int, result_dict, semantic_grid_ints, semantic_grid_zeros):
    with open('global_id_mapping.json', 'w') as file:
        json.dump(global_id_to_int, file, indent=4)

    with open('result_dict.json', 'w') as file:
        json.dump(result_dict, file, indent=4)

    # Save the semantic grids as npy files
    np.save('semantic_grid_ints.npy', semantic_grid_ints)
    np.save('semantic_grid_zeros.npy', semantic_grid_zeros)

# Main Execution
if __name__ == "__main__":
    ifc_file_path = 'models/model.ifc'
    ifc_file, settings = setup_ifc_geometry(ifc_file_path)
    tree = initialize_iterator(settings, ifc_file)

    points = load_point_cloud(FILENAME)
    onedivres = 1 / RES
    grid_stepy = GRID_SIZEX
    grid_stepz = GRID_SIZEX * GRID_SIZEY

    result_dict, global_id_to_int, semantic_grid_ints, semantic_grid_zeros = process_points(points, tree, onedivres, grid_stepy, grid_stepz)
    save_results(global_id_to_int, result_dict, semantic_grid_ints, semantic_grid_zeros)
