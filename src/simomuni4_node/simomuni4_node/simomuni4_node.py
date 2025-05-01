import rclpy
from rclpy.node import Node
from move_msgs.msg import Move
from std_msgs.msg import Int32MultiArray
import numpy as np

class simomuni4(Node):
    def __init__(self):
        super().__init__('simomuni4_node')
        self.submove = self.create_subscription(Move,"MoveTopic",self.createangular,10)
        self.pubAngvel = self.create_publisher(Int32MultiArray,'AngvelTopic',10)
        
        self.angular = np.matrix(np.zeros((4,1)))
        self.order = np.matrix(np.zeros((3,1)))
        self.r = 5
        self.d = 32
        self.G = np.array([[1, 0, self.d],
                           [0, -1,self.d],
                           [-1,0, self.d],
                           [0, 1, self.d]])
        
        self.noise_std = 0.05
    
    def createangular(self,msg):
        angular_data = Int32MultiArray()
        self.order[0,0] = msg.speed
        self.order[1,0] = msg.degree
        self.order[2,0] = msg.rotation
            
        self.angular = self.G*self.order/self.r
        noise = np.random.normal(loc=0.0,scale=self.noise_std,size=(4,1))
        self.angular += noise
        #self.get_logger().info(f"{type(self.angular)}")
        for x in range(4):
            angular_data.data.append(int(self.angular[x,0]))
        self.pubAngvel.publish(angular_data)
        
        
def main():
    rclpy.init()
    node = simomuni4()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("push ctrl+c")
    finally:
        rclpy.shutdown()
        print('Finish program')

if '__main__' == __name__:
    main()
            
        
                
