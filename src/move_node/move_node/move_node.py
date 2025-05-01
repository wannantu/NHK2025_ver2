import rclpy
from rclpy.node import Node
from move_msgs.msg import Move
from std_msgs.msg import Int16MultiArray
import serial


class MoveNode(Node):
    def __init__(self):
        super().__init__('move_node')
        try:
            self.moveSerial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("connect Serial ports for move")
        except Exception as e:
            self.get_logger().error("not connect Serial ports for move")
        
        #master_nodeからのsubsriotion
        self.subMove = self.create_subscription(Move,"MoveTopic",self.sendMovedata,10)
        #slamに角速度を贈る
        self.pubAngvel = self.create_publisher(Int16MultiArray,'AngvelTopic',10)
        
    def sendMovedata(self,msg):
        try:
            self.get_logger().info(f"msg:{msg}")
            speed = (str(msg.speed)+'\r').encode()
            degree = (str(msg.degree)+'\r').encode()
            rotation = (str(msg.rotation)+'\r').encode()
            self.moveSerial.write(speed)
            self.moveSerial.write(degree)
            self.moveSerial.write(rotation)
        except Exception as e:
            self.get_logger().error(f"Error {str(e)}")
        
        self.Readdata()
    
    def Readdata(self):
        angmsg = Int16MultiArray()
        for x in range(4):
            angmsg.data.append(self.moveSerial.readline())
        
        self.pubAngvel.publish(angmsg)
        

def main():
    rclpy.init()
    node = MoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Push Ctrl + C')
    finally:
        node.moveSerial.close()
        rclpy.shutdown()
        print('Finish program')

if __name__ == '__main__':
    main()