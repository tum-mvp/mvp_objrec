#!/usr/bin/env python

import roslib; roslib.load_manifest('objrec_ros_integration')

import rospy

import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import visualization_msgs.msg as visualization_msgs
import objrec_msgs.msg as objrec_msgs

def main():
    # Initialization
    rospy.init_node('objrec_spoofer')

    # Get spoofer parameters
    frame_id = rospy.get_param('~frame_id','/world')
    publish_period = rospy.get_param('~publish_period',1.0)
    publish_rate = rospy.Rate(1.0/publish_period)

    # Get the model info
    models = rospy.get_param('~models')
    stl_uris = {}
    for model in models:
        stl_uris[model] = rospy.get_param('~stl_uris/'+model)

    # Get objects from rosparam
    spoofed_objects = rospy.get_param('~spoofed_objects')

    # Create the publishers
    obj_pub = rospy.Publisher('/recognized_objects',objrec_msgs.RecognizedObjects)
    obj_marker_pub = rospy.Publisher('/recognized_objects_markers',visualization_msgs.MarkerArray)

    while not rospy.is_shutdown():
        # Initialize objects message
        objects_msg = objrec_msgs.RecognizedObjects()
        objects_msg.header.stamp = rospy.Time.now()
        objects_msg.header.frame_id = frame_id

        # Initialize marker array message
        marker_array = visualization_msgs.MarkerArray()
        marker_id = 0

        for obj in spoofed_objects:
            # Construct and populate an object  message
            pss_msg = objrec_msgs.PointSetShape()
            pss_msg.label = obj['label']
            pss_msg.confidence = obj['confidence']
            pss_msg.pose = geometry_msgs.Pose(geometry_msgs.Point(*obj['position']), geometry_msgs.Quaternion(*obj['orientation']))
            # Store the message
            objects_msg.objects.append(pss_msg)

            # Construct and populate a marker message
            marker = visualization_msgs.Marker()

            marker.header = objects_msg.header
            marker.ns = "objrec"
            marker.type = visualization_msgs.Marker.MESH_RESOURCE
            marker.action = visualization_msgs.Marker.ADD
            marker.lifetime = rospy.Duration(publish_period)
            marker.scale = geometry_msgs.Vector3(0.001, 0.001, 0.001)
            marker.color = std_msgs.ColorRGBA(1.0, 0.2, 0.4, 0.5)

            marker.id = marker_id
            marker.pose = pss_msg.pose
            marker.mesh_resource = stl_uris[obj['label']]

            # Store the message
            marker_array.markers.append(marker)
            marker_id += 1

        #  Publish the object information
        obj_pub.publish(objects_msg)
        #  Publish the markers
        obj_marker_pub.publish(marker_array)

        publish_rate.sleep()


if __name__ == '__main__':
    main()
