#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker

def divide_map():
    rospy.init_node('map_divider_node')
    
    # Subscriber to receive the occupancy map
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    
    # Publisher to publish the markers for visualization
    marker_pub = rospy.Publisher('/visualization_markers', MarkerArray, queue_size=10)
    
    rospy.spin()

def map_callback(msg):
    map_width = msg.info.width
    map_height = msg.info.height
    cell_size = 10  # Define the size of each section (change as needed)
    
    num_sections_x = map_width // cell_size
    num_sections_y = map_height // cell_size
    
    marker_array = MarkerArray()
    
    for i in range(num_sections_x):
        for j in range(num_sections_y):
            marker = Marker()
            marker.header = msg.header
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = i * cell_size + (cell_size / 2.0)
            marker.pose.position.y = j * cell_size + (cell_size / 2.0)
            marker.pose.position.z = 0.0
            
            marker.scale.x = cell_size
            marker.scale.y = cell_size
            marker.scale.z = 0.1
            
            marker.color = assign_color(i, j)
            
            marker_array.markers.append(marker)
    
    marker_pub.publish(marker_array)

def assign_color(section_x, section_y):
    colors = [
        ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Red
        ColorRGBA(0.0, 1.0, 0.0, 1.0),  # Green
        ColorRGBA(0.0, 0.0, 1.0, 1.0),  # Blue
        ColorRGBA(1.0, 1.0, 0.0, 1.0),  # Yellow
        ColorRGBA(1.0, 0.0, 1.0, 1.0),  # Magenta
        ColorRGBA(0.0, 1.0, 1.0, 1.0)   # Cyan
    ]
    
    section_index = section_x + section_y * num_sections_x
    color_index = section_index % len(colors)
    
    return colors[color_index]

if __name__ == '__main__':
    divide_map()

