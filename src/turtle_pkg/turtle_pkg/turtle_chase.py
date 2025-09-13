#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim_msgs.msg import Pose
from turtlesim_msgs.srv import Spawn ,Kill
from std_msgs.msg import Int32
import math 
import random
 
class TurtleChase(Node):
    def __init__(self):
        super().__init__('turtle_chase')
        
        #score pub
        self.score =0
        self.score_publisher =self.create_publisher(Int32,'/score',10)
        #turtle1
        self.player_pose =None
        self.create_subscription (Pose ,'turtle1/pose',self.player_callback,10)
        #service-clients
        self.spawn_client = self.create_client(Spawn,'/spawn')
        self.kill_client = self.create_client(Kill,'/kill')
        self.spawn_client.wait_for_service()
        self.kill_client.wait_for_service()
        
        #enemies
        self.enemy_positions = {}
        self.enemy_names = ['enemy1', 'enemy2', 'enemy3']
        self.spawn_enemies()

        self.create_timer(0.2, self.check_collisions)
        
    def player_callback(self ,msg):
            self.player_pose = msg
    def enemy_callback(self, name):
        def callback(msg: Pose):
            self.enemy_positions[name] = msg
        return callback
          
    #game engine  >> enemy handling
    def spawn_enemies(self):
            for name in self.enemy_names:
              self.spawn_enemy(name)

    def spawn_enemy(self, name):
            x = random.uniform(1.0, 10.0)
            y = random.uniform(1.0, 10.0)

            req = Spawn.Request()
            req.x = x
            req.y = y
            req.theta = 0.0
            req.name = name

            future = self.spawn_client.call_async(req)
            
            def callback(fut):
                if fut.result() is not None:
                  self.create_subscription(Pose,f'/{name}/pose',self.enemy_callback(name), 10)
                  self.enemy_positions [name] = None
                else:
                   self.create_timer(0.5, lambda: self.spawn_enemy(name))
            
            future.add_done_callback(callback)
    def kill_enemy(self, name):
        req = Kill.Request()
        req.name = name
        future = self.kill_client.call_async(req)
        
        def callback(fut): 
            if fut.result() is not None:
                self.spawn_enemy(name)  #respawn
                
        future.add_done_callback(callback)
    def check_collisions(self):
        if self.player_pose is None:
            return

        for name, pose in list(self.enemy_positions.items()):
            if pose is None:
                continue

            dist = math.dist(
                (self.player_pose.x, self.player_pose.y),
                (pose.x, pose.y)
            )

            if dist < 0.5:
                #update score 
                self.score += 1
                msg = Int32()
                #pub it
                msg.data = self.score
                self.score_publisher.publish(msg)
                #kill & respawn
                self.kill_enemy(name)
                self.spawn_enemy(name)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleChase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
        