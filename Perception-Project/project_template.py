#!/usr/bin/env python

# Import modules
import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster
#Helper function to convert RGB to HSV
def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized


bins_range=(0, 256)
nbins = 32
#Helper function to compute color histograms
def compute_color_histograms(cloud, using_hsv=False):

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])

    # Compute histograms
    # Compute the histogram of the HSV channels separately
    h_hist = np.histogram(channel_1_vals, bins=nbins, range=bins_range)
    s_hist = np.histogram(channel_2_vals, bins=nbins, range=bins_range)
    v_hist = np.histogram(channel_3_vals, bins=nbins, range=bins_range)
    # Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((h_hist[0], s_hist[0], v_hist[0])).astype(np.float64)
    # Normalize the result
    normed_features = hist_features / np.sum(hist_features)
    
    return normed_features 

#Helper function to compute normal histograms
def compute_normal_histograms(normal_cloud):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # TODO: Compute histograms of normal values (just like with color)
    x_hist = np.histogram(norm_x_vals, bins=nbins, range =bins_range)
    y_hist = np.histogram(norm_y_vals, bins=nbins, range =bins_range)
    z_hist = np.histogram(norm_z_vals, bins=nbins, range =bins_range)
    # TODO: Concatenate and normalize the histograms
    hist_features = np.concatenate((x_hist[0], y_hist[0], z_hist[0])).astype(np.float64)
    normed_features = hist_features/ np.sum(hist_features)


    return normed_features

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    print type(yaml_dict["arm_name"]), type(yaml_dict["pick_pose"])
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w+') as outfile:
	yaml.dump(data_dict, outfile, default_flow_style=False)
    print "done yaml"

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

#  Convert ROS msg to PCL data
    pcl_data=ros_to_pcl(pcl_msg)
    # Voxel Grid filter
    # Create a VoxelGrid filter object for our input point cloud
    vox = pcl_data.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    # Note: this (1) is a poor choice of leaf size   
    # Experiment and find the appropriate size!
    LEAF_SIZE = 0.008   

    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()
    # Much like the previous filters, we start by creating a filter object: 
    cloud_filter = cloud_filtered.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    cloud_filter.set_mean_k(50)

    # Set threshold scale factor
    x = 1.0

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    cloud_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_filtered = cloud_filter.filter()
    
    # PassThrough filter
    # Create a PassThrough filter object.
    passthrough1 = cloud_filtered.make_passthrough_filter()
  

    # Assign axis and range to the passthrough filter object.
    filter_axis1 = 'z'
    passthrough1.set_filter_field_name(filter_axis1)
    axis_min1 = 0.6 
    axis_max1 = 1.1
    passthrough1.set_filter_limits(axis_min1, axis_max1)

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_p1_filtered = passthrough1.filter()
    # Create a PassThrough filter object.
    passthrough2 = cloud_p1_filtered.make_passthrough_filter()
  



    # Assign axis and range to the passthrough filter object.
    filter_axis2 = 'y'
    passthrough2.set_filter_field_name(filter_axis2)
    axis_min2 = -0.55
    axis_max2 = 0.55
    passthrough2.set_filter_limits(axis_min2, axis_max2)

    cloud_p_filtered = passthrough2.filter()
    # RANSAC plane segmentation
    # Create the segmentation object
    seg = cloud_p_filtered.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = 0.03
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers
    extracted_inliers = cloud_p_filtered.extract(inliers, negative=False)


    # Extract outliers
    extracted_outliers = cloud_p_filtered.extract(inliers, negative=True)


    #  Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_outliers) # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(3000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    #  Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                        white_cloud[indice][1],
                                        white_cloud[indice][2],
                                         rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    
    #  Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    #  Publish ROS messages
    pcl_cluster_cloud_pub.publish(ros_cluster_cloud)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
   
 

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    labeled_features =[]
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
	pcl_cluster = extracted_outliers.extract(pts_list)
	ros_cluster = pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists)).astype(np.float64)
        #detected_objects.append([feature])
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
	do.label = label
	do.cloud = ros_cluster

	
        detected_objects.append(do)
	
    # Publish the list of detected objects

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    detected_objects_pub.publish(detected_objects)
    
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:

        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(detected):

    # TODO: Initialize variables
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()

    dict_list = []
    yaml_filename = 'output_3.yaml' #Change for different worlds
    test_scene_num.data = 3         #Change for different worlds

    labels = []
    centroids = []
    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
    # TODO: Parse parameters into individual variables

    for obj in detected:
	    #print obj.label
            labels.append(obj.label)
	    points_arr = ros_to_pcl(obj.cloud).to_array()
            centroids.append(np.mean(points_arr, axis=0)[:3])
    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for i in range(0, len(object_list_param)):
        object_name.data = object_list_param[i]['name']
	object_group = object_list_param[i]['group']
        for j in range(0,len(labels)):
            if object_name.data == labels[j]:
		pick_pose.position.x = np.asscalar(centroids[j][0])
		pick_pose.position.y = np.asscalar(centroids[j][1])
		pick_pose.position.z = np.asscalar(centroids[j][2])
		#print pick_pose
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        
        # TODO: Create 'place_pose' for the object
	for j in range(0, len(dropbox_param)):
	    if object_group == dropbox_param[j]['group']:
	       place_pose.position.x = dropbox_param[j]['position'][0]
               place_pose.position.y = dropbox_param[j]['position'][1]
               place_pose.position.z = dropbox_param[j]['position'][2]
        # TODO: Assign the arm to be used for pick_place
	if object_group =='green':
	    arm_name.data = 'right'
	elif object_group == 'red':
	    arm_name.data = 'left'
        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
	print "Test_num:",type(test_scene_num),"Arm_name:", type(arm_name),"Ob_name:", type(object_name),"Pick_pose:", type(pick_pose),"Place_pose:", type(place_pose)
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
	
	dict_list.append(yaml_dict)
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        #try:
            #pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            #resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            #print ("Response: ",resp.success)

        #except rospy.ServiceException, e:
            #print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file

    send_to_yaml(yaml_filename, dict_list)


if __name__ == '__main__':

     # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # TODO: Create Publishers
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_cloud_pub = rospy.Publisher("/pcl_clusters", PointCloud2, queue_size=1)
    # Initialize color_list
    get_color_list.color_list = []
    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
