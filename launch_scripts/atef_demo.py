"""
Demo Script

"""
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node
import random
from datetime import datetime


random.seed(10)             # random seed used in script - may not be relevant in the CDemo


sim_nodes = []              # The container for launch description method

# Creating Node
sim_nodes.append(
    Node (
        package='autonomy_sample1',
        executable='autonomy_sample1',
        name = 'Autonomy_Plugin_Sample',
        output='screen',
        parameters = [
                { "WORKING_MINOR_FRAMES": [0] },
                { "ENTITY_ID": Entity_ID },
        ]
    )   
)

def generate_launch_description():
    return LaunchDescription(sim_nodes)
