import rclpy
from rclpy.node import Node
from omuni4_msgs.msg import Omuni4
from move_msgs.msg import Move
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import time

class EFK():
    def __init__(self):
        '''
        self.Xest = np.zeros((5,1)) 
        self.Pest = np.zeros((3,1))
        self.order = np.zeros((3,1))
        '''
        self.dt = 0.01
        self.Q = np.diag([0.1, 0.1, math.radians(1.0), 1.0,0.1])**2
        self.R = np.diag([1.0, math.radians(40.0),0.1])**2
    
    def MotionModel(self,xest,order):
        F = np.matrix([[1,0,0,0,0],
                      [0,1,0,0,0],
                      [0,0,1,0,0],
                      [0,0,0,0,0],
                      [0,0,0,0,0]])
        
        yow = math.radians(xest[2,0])
        t = self.dt
        B = np.matrix([[t*math.cos(yow)  ,0              ,0],
                      [0                ,t*math.sin(yow),0],
                      [0                ,0              ,t],
                      [1                ,0              ,0],
                      [0                ,0              ,1]])
        return F*xest + B*order
    
    #エンコーダーの読み取る値でコードをかく
    def ObservationModel(self,x):
        H = np.matrix([[1,0,0,0,0],
                      [0,1,0,0,0],
                      [0,0,1,0,0]])
        
        z = H*x
        return z
    
    def jacoF(self,x,order):
        v = order[0,0]
        yow = math.radians(x[2,0])
        t = self.dt        
        F = np.matrix([[1,0,-1*v*t*math.sin(yow),t*math.sin(yow),0],
                      [0,1,v*t*math.cos(yow),   t*math.sin(yow),0],
                      [0,0,1,                   0,              t],
                      [0,0,0,                   1,              0],
                      [0,0,0,                   0,              1]])
        
        return F
    
    def jacoH(self):
        H = np.matrix([[1,0,0,0,0],
                      [0,1,0,0,0],
                      [0,0,1,0,0]])
        
        return H
            
    def ekf(self,Xest,Pest,z,order):
        #予測
        xPred = self.MotionModel(Xest,order)
        jF = self.jacoF(xPred,order)
        PPred = jF * Pest * jF.T + self.Q
        
        #更新
        zPred =self.ObservationModel(xPred)
        e = z.T - zPred
        H = self.jacoH()
        s = H*PPred*H.T + self.R
        k = PPred*H.T * np.linalg.inv(s) 
        Xest = xPred + k*e
        PEst = (np.eye(len(Xest)) - k * H) * PPred
        
        return Xest,PEst
        
        
        
class SlamNode(Node,EFK):
    def __init__(self):
        super().__init__("slam_node")
        EFK.__init__(self)
        self.ekf = EFK()
            
        self.omuni4data = Omuni4()
        self.Xest = np.matrix(np.zeros((5,1)))
        self.Pest = np.matrix(np.eye(5))
        self.u = np.matrix(np.zeros((3,1))) #u命令値[v,θ,w]
        self.z = np.matrix(np.zeros((1,3)),dtype = float) #z観測値 [x,y,θ]
        
        self.gia = 19
        self.r = 5
        self.d = 32
        self.t = 1/(2*self.d)
        self.G = np.matrix([[1,0,-1,0],
                           [0,-1,0,1],
                           [self.t,self.t,self.t,self.t]])
        self.w = np.matrix(np.zeros((4,1)))
        self.msg = Float64MultiArray()
        self.addmove = np.matrix(np.zeros((3,1)))
        
        #命令モデルの受信
        self.suborder = self.create_subscription(Move,'MoveTopic',self.ReadOder,10)
        #自己位置情報の送信
        self.pubpos = self.create_publisher(Float64MultiArray,'LocationTopic',10)
        #オムニの角速度を受信
        self.subAngvel = self.create_subscription(Int32MultiArray,'AngvelTopic',self.ReadAngele,10)
        
        self.estpostimer = self.create_timer(0.01,self.estpos)
        
    #観測データを作成    
    def estpos(self):
        self.Xest , self.Pest = self.ekf.ekf(self.Xest,self.Pest,self.z,self.u)
        
        for x in range(3):
            self.addmove[x,0] += self.Xest[x,0]
        
        self.msg.data = [self.addmove[i,0] for i in range(3)]
        
        self.msg.data[2] *= (360/(2*math.pi))#radを°に変換
        #self.get_logger().info(f"{self.msg.data}")
        self.pubpos.publish(self.msg)
    
    def ReadAngele(self,msg):
        for x in range(4):
            self.w[x,0] = msg.data[x]
        self.w = self.w / (self.gia * 60.0) #rpmをrpsに変換
        tmp = self.G*self.w* self.r/2
        self.z += (tmp*0.01).T
        self.get_logger().info(f"{self.z}")
    
    
    def ReadOder(self,msg):
        self.u[0,0] = msg.speed
        self.u[1,0] = msg.degree + self.Xest[2,0]
        self.u[2,0] = msg.rotation
        
        
        
        
def main():
    rclpy.init()
    node = SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Push Ctrl + C')
    finally:
        rclpy.shutdown()
        print('Finish program')

if __name__ == '__main__':
    main()