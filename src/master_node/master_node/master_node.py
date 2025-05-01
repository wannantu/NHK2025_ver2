import rclpy
from rclpy.node import Node
from ps_msgs.msg import PSMsgs
from move_msgs.msg import Move
#from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Int32MultiArray


class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        self.pubmove = self.create_publisher(Move,"MoveTopic",10)
        self.subWifi = self.create_subscription(Int16,"WifiTopic", self.controll,20)
        self.setspeedSub = self.create_subscription(Int32MultiArray,"SetSpeedTopic",self.SetSpeed,10)
        
        #デバック用
        #self.create_timer(0.1,self.showdata)
        
        #足回り、射出の速度のパラメータ
        self.movespeed = 0
        self.anglespeed = 0
        
        
    def SetSpeed(self,msg):
        self.movespeed = msg.data[0]
        self.anglespeed = msg.data[1]
        self.injpoint = msg.data[2]
        self.get_logger().info(f"{msg}")#デバック用
        
    
    
    def controll(self,msg):
        move = Move()
                
        try:
            if msg.data == -1:
                move.speed = 0
            else:
                move.speed = self.movespeed
                move.degree = msg.data
        except:
            self.get_logger().info("no data")
            return 
        #self.get_logger().info(f"{move}")
        self.pubmove.publish(move)
        
        
        
        
def main():
    rclpy.init()
    node = MasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Push Ctrl + C')
    finally:
        rclpy.shutdown()
        print('Finish program')

if __name__ == '__main__':
    main()
