#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="sim"
        ),
        Node(
            package="turtle_pkg",
            executable="turtle_chase",
            name="turtle_chase",
            output="screen",
        ),
    ])