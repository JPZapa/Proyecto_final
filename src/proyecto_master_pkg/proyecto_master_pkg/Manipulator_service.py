from proyecto_interfaces.srv import StartManipulationTest
from proyecto_interfaces.srv import StartNavigationTest

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from time import time
from time import sleep
import numpy as np
from math import cos
from math import sin
from math import pi
import math as mt

TOL=1e-6 # error cuadrático de respuesta
timeout=1 # timeout de 1 segundo


r1=2.75
r2=8
r3=8
r4=2.75
r5=8
A=[0,0]
D=[0,0]


g=7 #cm distancia al gripper
d=5 # distancia desde el centro de Pepe al centro coordenado del brazo
Vel=3# velocidad de las llantas
L=18# longitud de Pepe 
w=2*Vel/L# velocidad angular
tgiro=(mt.pi/2)/w



pf1=(27,1) # tupla de distancias de la ficha al centro de Pepe (x,z) plat 15 cm
pf2=(27,-4) # tupla de distancias de la ficha al centro de Pepe (x,z) plat 10 cm

Ef0=(17,4) #tupla de posicion base

plt1=[(30,4),"F"] # lista con la posicion de la plataforma y la orientacion de Pepe respecto a esta
plt2=[(30,4),"F"] # lista con la posicion de la plataforma y la orientacion de Pepe respecto a esta


        

def Newton_Rhapson(J,F,X0, TOL):
    global timeout
    tstart=time()
    tfin=time()
    Xp=X0
    X= Xp-np.linalg.inv(J(Xp)).dot(F(Xp))
    while np.linalg.norm(X-Xp)>TOL and (tfin-tstart)<=timeout:
        Xp=X
        X-=np.linalg.inv(J(X)).dot(F(X))
        tfin=time()
    if (tfin-tstart)<=timeout:
        res=X
    else:
        res=np.ones([len(X0),len(X0[0])])*np.nan
    return res


def inversa(Ef,Xa):
    global TOL
    global r1
    global r2
    global r3
    global r4
    global r5
    global A
    global D
    
    Xa=Xa*(pi/180)
    
    
    Finv=lambda X: np.array([[A[0]+r1*cos(X[0])+r2*cos(X[1])+(r4+r5)*cos(X[3])-Ef[0]],
                          [D[0]+r3*cos(X[2])+r5*cos(X[3])-Ef[0]],
                          [A[1]+r1*sin(X[0])+r2*sin(X[1])+(r4+r5)*sin(X[3])-Ef[1]],
                          [D[1]+r3*sin(X[2])+r5*sin(X[3])-Ef[1]]])





    Jinv=lambda X: np.array([[-r1*sin(X[0]), -r2*sin(X[1]), 0, -(r4+r5)*sin(X[3])],
                            [0, 0, -r3*sin(X[2]), -r5*sin(X[3])],
                            [r1*cos(X[0]), r2*cos(X[1]), 0, (r4+r5)*cos(X[3])],
                            [0, 0, r3*cos(X[2]), r5*cos(X[3])]])
    X=Newton_Rhapson(Jinv,Finv,Xa, TOL)
    
    return X*(180/pi)

def orientacion(ori):
    if ori=="F":
        ori=(0,-1)
    elif ori=="B":
        ori=(0,1)
    elif ori=="D":
        ori=(1,0)
    elif ori=="I":
        ori=(-1,0)
    return ori
    
def direc(dire0,dire1):
    if dire0[1]<0:
        if dire1[0]>0:
            mov="Der"
        elif dire1[0]<0:
            mov="Izq"
        elif dire1[1]<0:
            mov="M"
            
    elif dire0[1]>0:
        if dire1[0]>0:
            mov="Izq"
        elif dire1[0]<0:
            mov="Der"
        elif dire1[1]>0:
            mov="M"
            
    elif dire0[0]>0:
        if dire1[1]<0:
            mov="Izq"
        elif dire1[1]>0:
            mov="Der"
        elif dire1[0]<0:
            mov="M"
            
    elif dire0[0]<0:
        if dire1[1]<0:
            mov="Der"
        elif dire1[1]>0:
            mov="Izq"
        elif dire1[0]>0:
            mov="M"
    return mov

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(StartManipulationTest, '/group_8/start_manipulation_test_srv', self.manipulator_callback)
        self.publisherv = self.create_publisher(Twist, '/turtlebot_cmdVel', 10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'servo_angles', 10)
        self.cli = self.create_client(StartNavigationTest, '/group_8/start_navigation_test_srv') 
        
        Xo=[[106.0],[45.0],[45.0],[-73.0]]# lo que de despues de ejecutar cinamtica directa
        self.X=np.array(Xo)
        #self.Ef=[8,8] # lo que de despues de ejecutar cinematica directa
        self.Ef=[8,-2] # lo que de despues de ejecutar cinematica directa
        msg = Float32MultiArray()
        msg.data=[self.X[0][0],self.X[2][0],0.0]
        self.publisher_.publish(msg)
        
        self.req = StartNavigationTest.Request()

    def manipulator_callback(self, request, response):
        global pf1
        global pf2
        global plt1
        global plt2
        global Ef0
        
        global tgiro
        global w
        
        msg=Float32MultiArray()
        
        
        if request.platform=="platform_1":
            plt0=plt1
            pf0=pf1
            pltf=plt2
            pff=pf2
        elif request.platform=="platform_2":
            plt0=plt2
            pf0=pf2
            pltf=plt1
            pff=pf1
        #inicia navegacion    
        x=plt0[0][0]
        y=plt0[0][1]
        dire0=self.send_request(x,y)# se ejecuta el servicio de navegacion hasta plt0 y se obtiene la orientacio
        dire0=dire0.orientation
        dire1=orientacion(plt0[1])
        self.giro(dire0, dire1)
        #inicia movimiento brazo        
                
        Ef=[float(Ef0[0])-(g+d),float(Ef0[1])]
        print(Ef)
        Xp=inversa(Ef, self.X)
        if Xp[0]<=180 and Xp[0]>=90 and (180-Xp[2])<=180 and (180-Xp[2])>=0:
            #self.X=Xp
            self.Ef=Ef
            print(Xp)
        else:
            print("punto fuera de rango")
        msgb=[Xp[0][0],(180-Xp[2][0]),1.0] # theta3 se envia de esa forma por la forma en la que físicamente esta seteado el servo
        
        
        
        Ef=[float(pf0[0])-(g+d),float(pf0[1])]
        print(Ef)
        #Xp=inverses(Ef,self.Ef, self.X)
        Xp=inversa(Ef, self.X)
        if Xp[0]<=180 and Xp[0]>=90 and (180-Xp[2])<=180 and (180-Xp[2])>=0:
            #self.X=Xp
            self.Ef=Ef
            print(Xp)
        else:
            print("punto fuera de rango")
        msgP0=[Xp[0][0],(180-Xp[2][0]),0.0] # theta3 se envia de esa forma por la forma en la que físicamente esta seteado el servo
        
        msg.data=msgb
        print(msg.data)
        self.publisher_.publish(msg) #publish 17,0 abierto
        sleep(1)
        msg.data=msgP0
        print(msg.data)
        self.publisher_.publish(msg) #publish 27,-4 cerrado
        sleep(3)
        msgb[2]=0.0
        msg.data=msgb
        print(msg.data)
        self.publisher_.publish(msg) #publish 17,0 cerrado
        
        #regresa a la orientacion inicial
        self.giro(dire1, dire0)
        
        """se inicia el recorrido hacia la plataforma destino"""
        
        msg.angular.z=0.0
        x=pltf[0][0]
        y=pltf[0][1]
        dire0=self.send_request(x,y)# se ejecuta el servicio de navegacion hasta plt0 y se obtiene la orientacio
        dire0=dire0.orientation
        dire1=orientacion(pltf[1])
        self.giro(dire0, dire1)
                
         
             
        
        Ef=[float(pff[0])-(g+d),float(pff[1])]
        print(Ef)
        #Xp=inverses(Ef,self.Ef, self.X)
        Xp=inversa(Ef, self.X)
        if Xp[0]<=180 and Xp[0]>=90 and (180-Xp[2])<=180 and (180-Xp[2])>=0:
            #self.X=Xp
            self.Ef=Ef
            print(Xp)
        else:
            print("punto fuera de rango")
        msgPf=[Xp[0][0],(180-Xp[2][0]),1.0] # theta3 se envia de esa forma por la forma en la que físicamente esta seteado el servo
        
        msg.data=msgb
        print(msg.data)
        self.publisher_.publish(msg) #publish 17,0 cerrado
        sleep(1)
        msg.data=msgPf
        print(msg.data)
        self.publisher_.publish(msg) #publish 27,-4 abierto
        sleep(3)
        msgb[2]=1.0
        msg.data=msgb
        print(msg.data)
        self.publisher_.publish(msg) #publish 17,0 abierto
        
        #regresa a la orientacion inicial
        self.giro(dire1, dire0)
        
        response.answer="servicio aprobado"
        return response
    
    def giro(self,dire0,dire1):
        global w
        msgv=Twist()
        mov=direc(dire0, dire1)
        
        
        if mov=="Izq" or mov=="M":
            msgv.angular.z=w
        elif mov=="Der":
            msgv.angular.z=-w
        
        t0=time()
        if mov!="M":
            while time()-t0<tgiro:
                self.publisherv.publish(msgv) #publique el mensaje correspondiente
                #sleep(0.5)
        else:
            while time()-t0<2*tgiro:
                self.publisherv.publish(msgv) #publique el mensaje correspondiente
                #sleep(0.5)
        msgv.angular.z=0.0
        self.publisherv.publish(msgv)
        sleep(2)
        
    def send_request(self, x, y):
        self.req.x = x
        self.req.y = y
        self.future = self.cli.call_async(self.req)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
