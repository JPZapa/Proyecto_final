import rclpy
from rclpy.node import Node
from time import time
from geometry_msgs.msg import Twist
import math as mt
import cv2
from time import sleep
import numpy as np
from queue import PriorityQueue
from proyecto_interfaces.srv import StartNavigationTest #creae servicios y carpetas
#from modules.navigation_module import solve_maze

aj=1

L=18 # longitud del robot en cm
Vel=aj*37.7#velocidad en cm/s   
wder=2.12
wizq=2.63

tgirod=mt.pi/(aj*2*wder)
tgiroi=mt.pi/(aj*2*wizq)
maze=cv2.imread("/home/robotica/Downloads/MapaRobotica.pgm")

def direp(p1,p2):
    return(p2[0]-p1[0],p2[1]-p1[1])

def d(p1,p2):
    x=pow(p1[0]-p2[0],2)
    y=pow(p1[1]-p2[1],2)
    return pow(x+y,1/2)
    
def h(pos0,posf):  #distancia manhattan (corresponde a la heuristica del problema)
    x=abs(posf[0]-pos0[0])
    y=abs(posf[1]-pos0[1])
    
    return x+y

def cm_to_pix(p0)->tuple:
    pix=(round(p0[0]*4),round(p0[1]*4))  
    return pix

def Pix2cm(Pix):
    cm=(Pix[0]/4,Pix[1]/4)
    return cm

def clean_path(Path):
    cPath=[Path[0]]
    if Path[0][0]==Path[1][0]:
        Centinela=False
    else:
        Centinela=True  
    i=1 
    while i<len(Path)-1:
        if Path[i][0]==Path[i+1][0] and Centinela==True:
            cPath.append(Path[i])
            Centinela=False
        elif Path[i][1]==Path[i+1][1] and Centinela==False:
            cPath.append(Path[i])
            Centinela=True
        i+=1
    cPath.append(Path[i])
    return cPath

def solve_maze(pos0,posf,img):# params:(pos0,posf,maze) el maze es una imagen
    d=18 # Span de pepe en centimetros
    r=20 # parametro de salto entre vecinos (pixeles)
        
        
    d=round(round(d*4)/2)  #mitad del span de pepe en pixeles
        
    gscore=np.full((len(img), len(img[0])), np.inf)
    fscore=np.full((len(img), len(img[0])), np.inf)
        
        
    pos0=cm_to_pix(pos0)# punto inicial en pixeles
    posf=cm_to_pix(posf)# punto final en pixeles
        
    gscore[pos0[1]][pos0[0]]=0
    fscore[pos0[1]][pos0[0]]=h(pos0, posf)
        
    open=PriorityQueue()
    open.put((h(pos0, posf),h(pos0, posf),pos0))
    aPath={}
        
        
        
    while not open.empty():
            
        currPix=open.get()[2]
            
        if currPix[0] in range(posf[0]-r,posf[0]+r) and currPix[1] in range(posf[1]-r,posf[1]+r):
        #if currPix==posf:
            posf=currPix
            break
            
            
            #nodos vecindad
        izq=(currPix[0]-r,currPix[1])
        der=(currPix[0]+r,currPix[1])
        sup=(currPix[0],currPix[1]+r)
        inf=(currPix[0],currPix[1]-r)
            
        vecinos=[izq,der,sup,inf]
            
        for vecino in vecinos:
                
                #revision de celdas libres y tamano del Pepe
            available=True
            for i in range(vecino[1]-d,vecino[1]+d):
                for j in range(vecino[0]-d,vecino[0]+d):
                    if img[i][j][0]==94:
                        available=False

                        
                # analisis fscore y gscore
            if available==True:
                temp_gscore=gscore[currPix[1]][currPix[0]]+1
                temp_fscore=h(vecino,posf)+temp_gscore
                if temp_fscore<fscore[vecino[1]][vecino[0]]:
                    fscore[vecino[1]][vecino[0]]=temp_fscore
                    gscore[vecino[1]][vecino[0]]=temp_gscore
                    open.put((temp_fscore,h(vecino,posf),vecino))
                        # print(open.get())
                        # print(open.empty())
                    aPath[vecino]=currPix
                    


    cell=posf
    Path=[cell] #lista que guarda el camino de forma inversa
    while cell!=pos0:
          Path.append(aPath[cell])
          cell=aPath[cell]
    Path.reverse() #invierte la lista para tener los pasos en orden
    Path=clean_path(Path)   # Path que posee puntos o nodos entre los que se puede trazar lineas rectas
    cPath=[]
    for nodo in Path:
        cPath.append(Pix2cm(nodo))
    return cPath

def orientaciond(ori):
    if ori[1]>0:
        ori="B"
    elif ori[1]<0:
        ori="F"
    elif ori[0]>0:
        ori="D"
    elif ori[0]<0:
        ori="I"
    return ori

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



class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(StartNavigationTest, '/group_8/start_navigation_test_srv', self.navigation_callback)
        self.publisher_ = self.create_publisher(Twist, '/turtlebot_cmdVel', 10)
        
        self.pos= input("ingrese posicion inicial en cm (ej: x,y): ") # x,y
        self.dire=input("ingrese orientacion inicial(ej: F): ")  #(F,B,D,I)
        self.pos=self.pos.split(",")
        self.pos=(float(self.pos[0]),float(self.pos[1]))
        print(self.pos)
        self.dire=orientacion(self.dire)

        
        

    def navigation_callback(self, req, response):
        global maze
        global tgiroi
        global tgirod
        global Vel
        global wder
        global wizq
        
        
        
        msg=Twist()
        posf=(req.x,req.y)
        print(posf)
        Path=solve_maze(self.pos,posf,maze) #path con los nodos en cm
        #Path=[(38.0, 140.0),(38.0, 175.0),(93.0, 175.0),(93.0, 155.0),(143.0, 155.0),(143.0, 115.0),(198.0, 115.0)]
        i=0
        while i<len(Path)-1:
            mov=None
            msg.angular.z=0.0
            msg.linear.x=0.0
            dire1=direp(Path[i],Path[i+1])
            print(orientaciond(dire1))
            if i!=0:
                dire0=direp(Path[i-1],Path[i])
            else:
                dire0=self.dire
            if dire0[1]<0:
                if dire1[0]>0:
                    mov="Der"
                elif dire1[0]<0:
                    mov="Izq"
                elif dire1[1]>0:
                    mov="M"
                    
            elif dire0[1]>0:
                if dire1[0]>0:
                    mov="Izq"
                elif dire1[0]<0:
                    mov="Der"
                elif dire1[1]<0:
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
            print(mov)
            tgiro=0
            if mov=="Izq" or mov=="M":
                msg.angular.z=wizq
                tgiro=tgiroi
            elif mov=="Der":
                msg.angular.z=-wder
                tgiro=tgirod
            
            t0=time()
            if mov!="M":
                while time()-t0<tgiro:
                    self.publisher_.publish(msg) #publique el mensaje correspondiente
                    #sleep(0.5)
            else:
                while time()-t0<2*tgiro:
                    self.publisher_.publish(msg) #publique el mensaje correspondiente
            msg.angular.z=0.0
            self.publisher_.publish(msg) #publique el mensaje correspondiente
            sleep(3)
            msg.linear.x=Vel
                    
                   
            dis=d(Path[i],Path[i+1])
            t=dis/(Vel)
            t0=time()
            while time()-t0<t:
                self.publisher_.publish(msg) #publique el mensaje correspondiente
            i+=1
            msg.linear.x=0.0
            self.publisher_.publish(msg) #publique el mensaje correspondiente
            sleep(3)
        
        self.dire=direp(Path[-2],Path[-1])
        self.pos=(req.x,req.y)
        dire=orientaciond(self.dire)
        print(dire)
        
        response.answer=dire

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
