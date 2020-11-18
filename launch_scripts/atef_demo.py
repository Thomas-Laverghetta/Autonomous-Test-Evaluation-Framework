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
        package='demo_atef',
        executable='demo_atef_node',
        name = 'billyBob',
        output='screen',
        parameters = [
                { "TopicInput1": "input1" },
                { "TopicOutput1": "output1" }
        ]
    )   
)

sim_nodes.append(
    Node (
        package='demo_atef',
        executable='demo_atef_node',
        name = 'billyBob2',
        output='screen',
        parameters = [
                { "TopicInput1": "output1" },
                { "TopicOutput1": "input1" }
        ]
    )   
)

def generate_launch_description():
    return LaunchDescription(sim_nodes)
