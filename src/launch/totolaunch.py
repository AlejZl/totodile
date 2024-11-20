from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


my_package = 'tutlebot3_slamNav'

def generate_launch_description():
    base_path = os.path.realpath(get_package_share_directory(my_package)) # also tried without realpath
    rviz_path=base_path+'/confg/tutelconfig.rviz'
    return LaunchDescription([
        
        #Launch Nav2 and SLAM Toolbox
        Node(

            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            #output='screen',
            parameters=[

                {'resolution':0.5},
                {'publish_period_sec':1.0}
            ]
        ),


        Node(

            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            #output='screen',
            parameters=[
                {'use_sim_time': False}
            ],
            arguments=[
                '-configuration_directory', '/opt/ros/humble/share/turtlebot3_cartographer/config',
                '-configuration_basename', 'turtlebot3_lds_2d.lua'
            ]

        ),
        
        
        Node(

            package='rviz2',
            executable='rviz2',
            name='rviz2',
            #output='screen',
            arguments=['-d', str(rviz_path)]
        ),

        # Node(
        # package='turtlebot3_teleop',
        # executable='teleop_keyboard',
        # output='screen',
        # prefix=['gnome-terminal --']

        # ),
        
        # Custom node for mode switching
        Node(
            
            package='tutlebot3_slamNav',
            executable='explorer_node',
            output='screen',
            prefix=['gnome-terminal --']
            )
    
    ])
