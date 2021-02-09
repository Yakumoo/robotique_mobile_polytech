#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap
import tf
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
#import cv2

map=None

def get_map_client(static_map):
    map_service = "static_map" if static_map else 'dynamic_map'
    rospy.wait_for_service(map_service)
    get_map = rospy.ServiceProxy(map_service , GetMap)
    return get_map().map


def main(static_map=True):
    rospy.init_node('map_reader')

    if not static_map:
        #http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('base_footprint', 'map', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
                #print("tf base_footprint-map failed!")
            rate.sleep()
        print(f"trans={trans}")
        print(f"rot={rot}")


    map = get_map_client(static_map)
    print(f"map height={map.info.height} width={map.info.width} x={map.info.origin.position.x} y={map.info.origin.position.y} resolution={map.info.resolution:.3f}")
    print("x={} y={}".format(map.info.origin.position.x, map.info.origin.position.y)) # map.info.origin.orientation.x
    if not static_map:
        robot_pixel = np.array([map.info.width//2+trans[0]/map.info.resolution,
            map.info.height//2+trans[1]/map.info.resolution])
        print(f"robot_pixel={robot_pixel}")

    npmap = np.array(map.data, dtype=np.uint8).reshape((map.info.height, map.info.width))
    npmap = np.where(npmap==-1, 255, npmap)
    npmap = 255-npmap

    #fig = plt.figure(figsize=(20,20))
    print("xrange({:.3f} {:.3f}) yrange({:.3f} {:.3f})".format(map.info.origin.position.x, map.info.origin.position.x+map.info.width*map.info.resolution, map.info.origin.position.y, map.info.origin.position.y+map.info.height*map.info.resolution))
    plt.imshow(npmap, extent=[map.info.origin.position.x, map.info.origin.position.x+map.info.width*map.info.resolution, map.info.origin.position.y+map.info.height*map.info.resolution, map.info.origin.position.y])
    if not static_map:
        plt.scatter(trans[0], trans[1], s=100, c="r", marker="x")
    else:
        plt.scatter(0,0,s=100, c="r", marker="x")
    plt.show()
    print("end")




if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Choose static or dynamic_map')
    parser.add_argument('-static_map', action='store_true') # default to dynamic_map
    args, unknown = parser.parse_known_args()
    #print(f"args {args} {unknown}")
    main(static_map=args.static_map)
