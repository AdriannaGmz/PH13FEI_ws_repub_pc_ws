#!/usr/bin/env python
import rospy
import struct
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, Image, CompressedImage
from sensor_msgs.point_cloud2 import read_points, create_cloud
from std_msgs.msg import Header
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
import json
from scipy.spatial.transform import Rotation as R
import message_filters

import cv2


def encode_rgb_to_float(rgb):
    """
    Encode RGB values [R, G, B] into a single 32-bit float for PointCloud2.
    """
    r, g, b = rgb
    packed = (int(r) << 16) | (int(g) << 8) | int(b)
    return struct.unpack('f', struct.pack('I', packed))[0]


def scale_camera_matrix(camera_matrix, original_size, compressed_size):
    """
    Scales the camera intrinsic matrix to match the resolution of the compressed image.

    Args:
        camera_matrix (numpy.ndarray): Original 3x3 intrinsic matrix from CameraInfo.
        original_size (tuple): Original image size as (width, height).
        compressed_size (tuple): Compressed image size as (width, height).

    Returns:
        numpy.ndarray: Scaled intrinsic matrix.
    """
    orig_width, orig_height = original_size
    comp_width, comp_height = compressed_size

    # Calculate scaling factors
    s_x = comp_width / orig_width
    s_y = comp_height / orig_height

    # Scale the intrinsic matrix
    scaled_matrix = np.copy(camera_matrix)
    scaled_matrix[0, 0] *= s_x  # Scale fx
    scaled_matrix[1, 1] *= s_y  # Scale fy
    scaled_matrix[0, 2] *= s_x  # Scale cx
    scaled_matrix[1, 2] *= s_y  # Scale cy

    return scaled_matrix


def rgb_to_hsv(color):
    """
    Convert an RGB color to HSV.
    Args:
        color (array-like): RGB color as [R, G, B].
    Returns:
        tuple: (Hue, Saturation, Value) in ranges H [0, 360], S [0, 1], V [0, 1].
    """
    # color= encode_rgb_to_float(color)
    r, g, b = color / 255.0  # Normalize RGB to [0, 1]
    max_c = max(r, g, b)
    min_c = min(r, g, b)
    delta = max_c - min_c

    # Calculate Hue
    if delta == 0:
        h = 0
    elif max_c == r:
        h = (60 * ((g - b) / delta) + 360) % 360
    elif max_c == g:
        h = (60 * ((b - r) / delta) + 120) % 360
    elif max_c == b:
        h = (60 * ((r - g) / delta) + 240) % 360

    # Calculate Saturation
    s = 0 if max_c == 0 else (delta / max_c)

    # Calculate Value
    v = max_c

    return h, s, v


def calculate_traversability_ganav(color):
    """
    Calculate traversability score based on nearest RGB match.

    Args:
        color (list): RGB color as [R, G, B] (0–255).

    Returns:
        float: Traversability score (0.0–1.0).
    """
# OK THIS!!!!
    # Predefined RGB values and their scores
    predefined_colors = {
        (0, 0, 0): 1.0,         # Black  - Non-traversable (Background)
        (255, 0, 0): 1.0,       # Red    (Rviz) - Non-traversable                   (Blue in BGR Visualizer)
        (0, 0, 128): 1.0,       # Blue   (Rviz)- Special case / Non-traversable     (Red in BGR Visualizer)
        (255, 128, 0): 0.8,     # Orange (Rviz) - Low traversable                   (Skyblue in BGR Visualizer)
        (255, 255, 0): 0.5,     # Cyan   (Rviz) - Medium traversable                (Yellow in BGR Visualizer)
        (0, 128, 0): 0.0        # Green  (Rviz) - Traversable                       (Green in BGR Visualizer)
    }

# # ONLY FOR INSIDE HALL TEST!!!!
#     # Predefined RGB values and their scores
#     predefined_colors = {
#         (255, 0, 0): 0.0,       # Red    (Rviz) - Traversable                       (Blue in BGR Visualizer)
#         (255, 128, 0): 0.5,     # Orange (Rviz) - Medium traversable                   (Skyblue in BGR Visualizer)
#         (255, 255, 0): 0.8,     # Cyan   (Rviz) - Low traversable                (Yellow in BGR Visualizer)
#         (0, 128, 0): 1.0,        # Green  (Rviz) - NON Traversable                   (Green in BGR Visualizer)
#         (0, 0, 128): 1.0,       # Blue   (Rviz) - Special case / Non-traversable     (Red in BGR Visualizer)
#         (0, 0, 0): 1.0,         # Black         - Non-traversable (Background)
#     }


    # # Predefined BGR values and their scores
    # predefined_colors = {
    #     (0, 0, 0): 1.0,         # Black - Non-traversable (Background)
    #     (0, 0, 255): 1.0,       # Red - Non-traversable
    #     (128, 0, 0): 1.0,        # Blue - Special case / Non-traversable
    #     (0, 128, 255): 0.8,     # Orange - Low traversable
    #     (0, 255, 255): 0.5,     # Yellow - Medium traversable
    #     (0, 128, 0): 0.0       # Green - Traversable
    # }

    # Predefined RGB values and their scores
    # predefined_colors = {
    #     (0, 0, 0): 0.0,         # Black - Non-traversable
    #     (0, 128, 0): 1.0,       # Green - Traversable
    #     (255, 255, 0): 0.5,     # Yellow - Medium traversable
    #     (255, 128, 0): 0.2,     # Orange - Low traversable
    #     (255, 0, 0): 254,       # Red - Non-traversable
    #     (0, 0, 128): 254        # Blue - Special case / Non-traversable
    # }

    # Calculate Euclidean distance between the input color and predefined colors
    color_array = np.array(color)
    # print(color)
    best_match = None
    min_distance = float('inf')
    # for predefined_color, score in predefined_colors.items():
    #     predefined_array = np.array(predefined_color)
    #     distance = np.linalg.norm(color_array - predefined_array)  # Euclidean distance
    #     if distance < min_distance:
    #         min_distance = distance            
    #         best_match = score
    #         if (best_match!=0.0)&(best_match!=1.0): rospy.loginfo("Intensity: "+str(best_match))
    r, g, b  = color
    delta_pixel  = 100
    for predefined_color, score in predefined_colors.items():  
        r_predefined, g_predefined, b_predefined = predefined_color  
        predefined_array = np.array(predefined_color)
        # if (r == r_predefined) & (g == g_predefined) & (b == b_predefined):
        # if (r >= r_predefined-delta_pixel) & (r <= r_predefined+delta_pixel):
        #     if (b >= b_predefined-delta_pixel) & (b <= b_predefined+delta_pixel):
        #         if (g >= g_predefined-delta_pixel) & (g <= g_predefined+delta_pixel):

        if (r >= r_predefined-delta_pixel) & (r <= r_predefined+delta_pixel) & \
            (b >= b_predefined-delta_pixel) & (b <= b_predefined+delta_pixel) & \
                (g >= g_predefined-delta_pixel) & (g <= g_predefined+delta_pixel):

            best_match = score

            #if abs(best_match - 0.5) < 1e-6:
            #    print("INSIDEEEEE i am cyan")
            break
            # if (best_match>=0.0)&(best_match<=1.0): rospy.loginfo("Intensity: "+str(best_match)+str(color))

        # else:
        #     rospy.loginfo("NO MATCH was found"+str(color_array)+str(r_predefined)+str(g_predefined)+str(b_predefined))

        else:
            best_match=1.0
    # return (1 - best_match) #+ 1
    # return best_match + 1
    #if abs(best_match - 0.5) < 1e-6:
    #    print("i am cyan")
    return best_match + 1.





def calculate_traversability_terrapn(color):
    """
    Calculate traversability score based on a Jet colormap in the unnormalized RGB range.

    Args:
        color (list): RGB color as [R, G, B] (0–255).

    Returns:
        float: Traversability score (0.0–1.0).
    """
    # Generate Jet colormap in the unnormalized [0, 255] range
    jet_colormap = plt.cm.get_cmap("jet", 256)  # Jet colormap with 256 colors
    colormap_colors = (jet_colormap(np.linspace(0, 1, 256))[:, :3] * 255).astype(int)

    # Calculate Euclidean distance between the input color and colormap colors
    color_array = np.array(color)
    distances = np.linalg.norm(colormap_colors - color_array, axis=1)
    closest_index = np.argmin(distances)

    # Map the closest color index to a normalized traversability score (0.0–1.0)
    traversability_score = closest_index / (len(colormap_colors) - 1)

    return traversability_score





class TraversabilityNode:
    def __init__(self):
        # rospy.init_node("traversability_node")
        rospy.loginfo("Loading the calibration")
        # self.lidartocam_file = '/home/datasets/husky/calibration_husky.json'
        self.lidartocam_file = '/home/husky/ws/repub_pc_ws/src/repub_traversability_pc/cfg/calibration_husky.json'

        self.lidar_sub = message_filters.Subscriber("/ouster/points", PointCloud2, queue_size=1, buff_size=16777216)
        #self.image_sub = message_filters.Subscriber("/segmented/image", Image) #, queue_size=1, buff_size=16777216)
        self.image_sub = message_filters.Subscriber("/segmented/image/compressed", CompressedImage, queue_size=1, buff_size=16777216)
        self.camera_info_sub = message_filters.Subscriber("/camera_array/cam1Right/camera_info", CameraInfo)#, queue_size=1)
        ts = message_filters.ApproximateTimeSynchronizer([ self.lidar_sub , self.image_sub , self.camera_info_sub ], 10, 0.9)
        ts.registerCallback(self.ts_callback)
        self.img_pub = rospy.Publisher('/encoded/image', Image, queue_size=10)
        self.pcl_pub = rospy.Publisher("/traversable_lidar", PointCloud2, queue_size=10)

        self.rgb_image = None
        self.camera_matrix = None
        self.scaled_camera_matrix = None
        self.dist_coeffs = None
        self.original_size = None

    def ts_callback(self, lidar_msg, image_msg, camera_info_msg):
        
        """         Callbacks for synchronized messages.  """


        """
        Callback for the Camera Info topic.
        Extracts intrinsic matrix and distortion coefficients.
        """
        self.camera_matrix = np.array(camera_info_msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(camera_info_msg.D)
        self.original_size = (camera_info_msg.width, camera_info_msg.height)


        print("\tDELAY ", (lidar_msg.header.stamp-image_msg.header.stamp).to_sec() )



        """
        Callback for the RGB image topic.
        Converts ROS Image to NumPy array and computes the scaled intrinsics.
        """
        print("I am in image callback")
        # Initialize the CvBridge
        # self.bridge = CvBridge()

        # Convert the ROS Image message to a NumPy array
        #self.rgb_image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
        np_arr = np.fromstring(image_msg.data, np.uint8)
        self.rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # rospy.loginfo("BEFORE FLIPPING")
        # rospy.loginfo(self.rgb_image)
        # self.rgb_image = self.rgb_image[:, :, ::-1]
        # self.rgb_image = self.rgb_image[..., ::-1]


        # self.rgb_image = self.rgb_image[:, :, ::-1].astype(np.uint32)
        # rospy.loginfo(" AFTER FLIPPING")
        # rospy.loginfo(self.rgb_image)
        

        # self.rgb_image = ros_numpy.numpify(image_msg)
        compressed_size = (self.rgb_image.shape[1], self.rgb_image.shape[0])  # (width, height)
        print(compressed_size)


        # Scale the camera matrix to match the compressed image size
        if self.camera_matrix is not None and self.original_size is not None:
            self.scaled_camera_matrix = scale_camera_matrix(self.camera_matrix, self.original_size, compressed_size)


        """
        Callback for the LiDAR point cloud topic.
        Projects the RGB image onto the LiDAR point cloud and publishes the updated cloud.
        """
        if self.rgb_image is None or self.scaled_camera_matrix is None:
            rospy.logwarn("Waiting for image or camera info to be available...")
            return

        # Convert ROS PointCloud2 to NumPy array
        points = np.array(list(read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)))
        lidar_scan = o3d.geometry.PointCloud()
        lidar_scan.points = o3d.utility.Vector3dVector(points[:, :3])

        # Process point cloud
        intensities = self.process_point_cloud(lidar_scan)

        print("points with intensity 0.5: {}".format(np.sum(intensities[...,3] == 1.5)))
        # Create a new PointCloud2 message with updated intensities
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = lidar_msg.header.frame_id

        fields = lidar_msg.fields
        fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)
        
]
        # print(header.frame_id, np.max(intensities[:, 3]), np.min(intensities[:, 3]))
        print(header.frame_id, (intensities[:, 3]).shape )

        new_cloud = create_cloud(header, fields, intensities)
        self.pcl_pub.publish(new_cloud)

    def project_rgb_onto_lidar(self, lidar_scan, rgb_image):
        """
        Project the RGB image onto the front-facing part of the LiDAR point cloud.

        Args:
            lidar_scan: LiDAR point cloud as Open3D PointCloud object.
            rgb_image: RGB image as a numpy array.

        Returns:
            LiDAR point cloud with RGB color projected onto the front-facing part.
        """
        
        cam2lidar = self.load_lidar_to_camera_matrix(self.lidartocam_file)
        # transform the LiDAR points to the camera coordinate system
        lidar_scan.transform(np.linalg.inv(cam2lidar))
        
        # Extract points from the LiDAR point cloud
        points = np.asarray(lidar_scan.points)
        
        # Keep only the points in front of the LiDAR sensor
        front_points = points[points[:, 2] > 0]
        
        # Project points to 2D using camera intrinsics
        uv = np.dot(self.scaled_camera_matrix, front_points.T).T
        uv = uv[:, :2] / uv[:, 2][:, np.newaxis]

        # Map the 2D projected points to the RGB image
        valid_mask = (uv[:, 0] >= 0) & (uv[:, 0] < rgb_image.shape[1]) & \
                        (uv[:, 1] >= 0) & (uv[:, 1] < rgb_image.shape[0])
        uv = uv[valid_mask]
        front_points = front_points[valid_mask]
        # Transform front points back to the original LiDAR coordinate system
        front_points_original = np.dot(
            np.hstack((front_points, np.ones((front_points.shape[0], 1)))),
            cam2lidar.T
        )[:, :3]

        colors = rgb_image[uv[:, 1].astype(int), uv[:, 0].astype(int)] / 255.0


        # Create a colored point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(front_points_original)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        return uv, front_points_original, pcd
    

    def process_point_cloud(self, lidar_scan):
        """
        Projects the RGB image onto the LiDAR point cloud and computes traversability scores.
        """
        uv, front_points, pcd = self.project_rgb_onto_lidar(lidar_scan, self.rgb_image)
        # print("\t here is image shape", self.rgb_image.shape)
        # print("\t here is Lidar shape", np.array(lidar_scan.points).shape)

        # Get pixel colors corresponding to projected points
        pixel_colors = self.rgb_image[uv[:, 1].astype(int), uv[:, 0].astype(int)]
        print("\t here are pixel colors", pixel_colors.shape)


        # Compute traversability scores based on colors
        scores = np.array([calculate_traversability_ganav(color) for color in pixel_colors])
        print("\t here are scores", scores.shape)
        print("scores with intensity 0.5: {}".format(np.sum(scores == 1.5)))


        # Create a new array with points and their corresponding scores
        intensities = np.zeros((front_points.shape[0], 4))
        intensities[:, :3] = front_points
        intensities[:, 3] = scores

        return intensities
    
    def load_lidar_to_camera_matrix(self, file_path):
        """
        Load the lidar_to_camera transformation parameters from a JSON file and compute cam2lidar matrix.

        Args:
            file_path (str): Path to the JSON file.

        Returns:
            np.ndarray: 4x4 transformation matrix from camera to LiDAR.
        """
        # Load JSON data
        with open(file_path, 'r') as f:
            data = json.load(f)
        
        # Extract lidar_to_camera transformation
        lidar_to_camera = data["lidar_to_camera"]
        translation = lidar_to_camera["translation"]
        rotation = lidar_to_camera["rotation"]
        
        # Create translation vector
        t = np.array([translation["x"], translation["y"], translation["z"]])
        
        # Create rotation matrix from quaternion
        quaternion = [rotation["qx"], rotation["qy"], rotation["qz"], rotation["qw"]]
        r = R.from_quat(quaternion)
        rotation_matrix = r.as_matrix()

        # Construct the lidar_to_camera transformation matrix
        lidar_to_camera_matrix = np.eye(4)
        lidar_to_camera_matrix[:3, :3] = rotation_matrix
        lidar_to_camera_matrix[:3, 3] = t

        # Compute the inverse for cam2lidar
        # cam2lidar_matrix = np.linalg.inv(lidar_to_camera_matrix)
        cam2lidar_matrix = lidar_to_camera_matrix

        return cam2lidar_matrix

    

if __name__ == "__main__":
    rospy.init_node("traversability_node", anonymous=True)
    print("Running")
    try:
        node = TraversabilityNode()
        rospy.spin()
    except KeyboardInterrupt:
        node.outVid.release()
    