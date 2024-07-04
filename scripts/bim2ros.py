#!/usr/bin/python3
import numpy as np
import ifcopenshell
import ifcopenshell.util.placement
import ifcopenshell.util.selector
import ifcopenshell.util.shape
import ifcopenshell.geom
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_geometry_msgs import do_transform_point

  # Global variable to store element occurrences

def load_ifc_model(model_path):
    rospy.loginfo("Loading IFC model...")
    try:
        model = ifcopenshell.open(model_path)
        rospy.loginfo("IFC model loaded.")
        return model
    except Exception as e:
        rospy.logerr(f"Failed to load IFC model: {e}")
        return None

def find_closest_objects_to_point_cloud(points):
    rospy.loginfo("Finding closest objects to point cloud...")
    for point in points:
        index = point2grid(point[0], point[1], point[2], onedivres, grid_stepy, grid_stepz)
        if loaded_data_zeros[index] == 0 and loaded_data[index]:
            element_value = loaded_data[index]
            element_occurrences[element_value] = element_occurrences.get(element_value, 0) + 1
            loaded_data_zeros[index] = 1

    for element_value, count in element_occurrences.items():
        rospy.loginfo(f"Value {element_value}: {count} times")

    return []

def point2grid(x, y, z, onedivres, grid_stepy, grid_stepz):
    index = int(np.floor(x * onedivres) + np.floor(y * onedivres) * grid_stepy + np.floor(z * onedivres) * grid_stepz)
    return index

def crop_cloud(cl, mindist=0.00, maxdist=100):
    cldist = np.linalg.norm(cl[:, 0:2], axis=1)
    cropped_cloud = cl[(cldist > mindist) & (cldist <= maxdist), :]
    return cropped_cloud

def save_element_occurrences(filename):
    try:
        with open(filename, 'w') as f:
            for element_value, count in element_occurrences.items():
                f.write(f"{element_value}: {count}\n")
        rospy.loginfo(f"Element occurrences saved to {filename}")
    except Exception as e:
        rospy.logerr(f"Failed to save element occurrences: {e}")

def pointcloud_callback(msg):
    rospy.loginfo("Received point cloud message.")
    points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pcl_example = [(point[0], point[1], point[2]) for point in points]
    pcl_example = crop_cloud(np.array(pcl_example))
    rospy.loginfo(f"Reduced number of LiDAR points to {len(pcl_example)}")

    transformed_points = []

    for point in pcl_example:
        point_stamped = PointStamped()
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.header.frame_id = "velodyne_base_link"
        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_stamped.point.z = point[2]
        try:
            transformed_point_c = tf_buffer.transform(point_stamped, target_frame='map', timeout=rospy.Duration(0.1))
            transformed_points.append((transformed_point_c.point.x, transformed_point_c.point.y, transformed_point_c.point.z))
        except tf2_ros.TransformException as ex:
            rospy.logwarn(f"Transform failed: {ex}")

    rospy.loginfo("Applied transformations.")
    rospy.loginfo("Finding objects")

    closest_objects = find_closest_objects_to_point_cloud(transformed_points)

    # Trigger to save element_occurrences (example: save when point cloud processed)
    save_element_occurrences("element_occurrences.txt")

def main():
    global tf_buffer, onedivres, grid_stepy, grid_stepz, loaded_data, loaded_data_zeros, element_occurrences

    rospy.init_node('ifc_processor')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    element_occurrences = {}

    RES = 0.1
    GRID_SIZEX = 220
    GRID_SIZEY = 220
    GRID_SIZEZ = 20
    onedivres = 1 / RES
    grid_stepy = GRID_SIZEX
    grid_stepz = GRID_SIZEX * GRID_SIZEY

    file = "/home/rva_container/rva_exchange/catkin_ws/src/Heuristic_path_planners/scripts/semantic_grid_ints.npy"
    loaded_data = np.load(file)
    file = "/home/rva_container/rva_exchange/catkin_ws/src/Heuristic_path_planners/scripts/semantic_grid_zeros.npy"
    loaded_data_zeros = np.load(file)

    rospy.Subscriber("velodyne_points", PointCloud2, pointcloud_callback)

    rospy.spin()

if __name__ == "__main__":
    main()
