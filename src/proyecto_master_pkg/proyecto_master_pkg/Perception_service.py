from proyecto_interfaces.srv import StartNavigationTest #creae servicios y carpetas
from proyecto_interfaces.srv import StartPerceptionTest
from proyecto_interfaces.msg import Banner
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import imutils
import pytesseract
import colorgram
import math
from time import time
from time import sleep
from geometry_msgs.msg import Twist



Vel=3# velocidad de las llantas
L=18# longitud de Pepe 
w=2*Vel/L# velocidad angular
tgiro=(math.pi/2)/w

proces=False
centinela=False

banner1=[(1,2),"F"] # lista con la posicion de la plataforma y la orientacion de Pepe respecto a esta
banner2=[(1,2),"F"] # lista con la posicion de la plataforma y la orientacion de Pepe respecto a esta
banner3=[(1,2),"F"] # lista con la posicion de la plataforma y la orientacion de Pepe respecto a esta

def apply_light_filter(image):
    r, g, b = cv2.split(image)
    r_avg = cv2.mean(r)[0]
    g_avg = cv2.mean(g)[0]   
    b_avg = cv2.mean(b)[0]
    k = (r_avg + g_avg + b_avg) / 3
    kr = k / r_avg
    kg = k / g_avg
    kb = k / b_avg
    r = cv2.addWeighted(src1=r, alpha=kr, src2=0, beta=0, gamma=0)
    g = cv2.addWeighted(src1=g, alpha=kg, src2=0, beta=0, gamma=0)
    b = cv2.addWeighted(src1=b, alpha=kb, src2=0, beta=0, gamma=0)
    balance_img = cv2.merge([b, g, r]) # Placeholder para el procesamiento real del filtro de luz
    return balance_img

def detectar_figura(contorno)->str:
    perimeter = cv2.arcLength(contorno, True)
    approximate = cv2.approxPolyDP(contorno, .04 * perimeter, True)
    if len(approximate) == 3:
       shape = 'TRIANGULO'
       # Si el polígono aproximado tiene 4 lados, entonces puede ser o un cuadrado o un rectángulo.
    elif len(approximate) == 4:
       # Calculamos la relación de aspecto.
       x, y, w, h = cv2.boundingRect(approximate)
       aspect_ratio = w / float(h)
       # La figura será un cuadrado si la relación de aspecto está entre 95% y 105%, es decir, si todos los lados miden
       # más o menos lo mismo. En caso contrario, se trata de un rectángulo.
       shape = 'CUADRADO' if .95 <= aspect_ratio <= 1.05 else 'RECTANGULO'
       # Si el polígono aproximado tiene 5 lados, es un pentágono.
    elif len(approximate) == 5:
       shape = 'PENTAGONO'
    elif len(approximate) == 6:
       shape = 'HEXAGONO'
    elif len(approximate) == 7:
       shape = 'HECTAGONO'
    elif len(approximate) == 8:
        shape = 'OCTAGONO'
    else:
       shape = 'CIRCULO'

    return shape
def detectionfigura(image):
    resized = imutils.resize(image, width=380)
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    res,thresholded = cv2.threshold(blurred, 110, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresholded, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    min_area=10000
    max_area=20000
    for contour in contours:
        if min_area < cv2.contourArea(contour) < max_area:
            figueFinal = detectar_figura(contour)
    return figueFinal
           # ...
# Función para reconocer el color
def get_color_name(listacolores):
    b=listacolores[0]
    g=listacolores[1]
    r=listacolores[2]
    colores = {
        'rojo-oscuro': (128, 0, 0),
        'rojo-normal': (255, 0, 0),
        'rojo-claro': (255, 128, 128),
        'verde-oscuro': (0, 128, 0),
        'verde-normal': (0, 255, 0),
        'verde-claro': (128, 255, 128),
        'azul-oscuro': (0, 0, 128),
        'azul-normal': (0, 0, 255),
        'azul-claro': (128, 128, 255),
        'amarillo-oscuro': (128, 128, 0),
        'amarillo-normal': (255, 255, 0),
        'amarillo-claro': (255, 255, 128),
        'verdeAzul-oscuro': (0, 128, 128),
        'verdeAzul-normal': (0, 255, 255),
        'verdeAzul-claro': (128, 255, 255),
        'magenta-oscuro': (128, 0, 128),
        'magenta-normal': (255, 0, 255),
        'magenta-claro': (255, 128, 255),
        'blanco-oscuro': (128, 128, 128),
        'blanco-normal': (255, 255, 255),
        'blanco-claro': (230, 230, 230),
        'negro-oscuro': (0, 0, 32),
        'negro-normal': (0, 0, 0),
        'negro-claro': (64, 64, 64),
        'gris-oscuro': (64, 64, 64),
        'gris-normal': (128, 128, 128),
        'gris-claro': (192, 192, 192),
        'naranja-oscuro': (128, 82, 0),
        'naranja-normal': (255, 165, 0),
        'naranja-claro': (255, 203, 128),
        'morado-oscuro': (64, 0, 64),
        'morado-normal': (128, 0, 128),
        'morado-claro': (192, 128, 192),
        'rosa-oscuro': (205, 140, 149),
        'rosa-normal': (255, 192, 203),
        'rosa-claro': (255, 218, 225),
        'marrón-oscuro': (128, 0, 0),
        'marrón-normal': (165, 42, 42),
        'marrón-claro': (180, 80, 80),
        'beige-oscuro': (210, 210, 180),
        'beige-normal': (245, 245, 220),
        'beige-claro': (255, 255, 230)
    }

    min_distance = math.inf
    color_name = None
    for name, color in colores.items():
        distance = math.sqrt((color[0] - b) ** 2 + (color[1] - g) ** 2 + (color[2] - r) ** 2)
        if distance < min_distance:
            min_distance = distance
            color_name = name

    return color_name
def reconocer_color(imagen):
    height, width, _ = imagen.shape
    
    # Recortar la imagen en dos partes y quedarse con la parte inferior
    img_cortada = imagen[height // 2 : height, :]
    
    # Guardar la imagen cortada temporalmente para que colorgram pueda procesarla
    cv2.imwrite("imagen_cortada.jpg", img_cortada)
    
    # Extraer los colores dominantes de la imagen cortada
    colores = colorgram.extract("imagen_cortada.jpg", 1)  # Extraer 1 color predominante
    
    # Obtener los valores RGB del color predominante
    color_predominante = colores[0].rgb
    
    # Convertir los valores RGB en una cadena de texto "(r,g,b)"
    rgblist = (color_predominante.r,color_predominante.g,color_predominante.b)
    
    return rgblist
def reconocerpalabras(imagen):
    # Configurar la ruta de Tesseract OCR
    pytesseract.pytesseract.tesseract_cmd = r'/usr/bin/tesseract'

    # Convertir la imagen a escala de grises
    gray = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
    
    # Aplicar umbralización a la imagen en escala de grises
    _, threshold = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    
    # Realizar OCR en la imagen binarizada con Tesseract OCR
    texto = pytesseract.image_to_string(threshold)
    
    return texto


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
        super().__init__('ServiceA')
        self.srv = self.create_service(StartPerceptionTest, '/group_8/start_navigation_test_srv', self.perception)
        self.subscription = self.create_subscription(Image,'/camera/imagecamera',self.image_callback,10)
        self.cli = self.create_client(StartNavigationTest, '/group_8/start_navigation_test_srv') 
        self.publisher = self.create_publisher(Banner, "/vision/banner_group_8", 10)
        self.publisherv = self.create_publisher(Twist, '/turtlebot_cmdVel', 10)
        self.subscription  # Evita que el objeto sea destruido prematuramente por el recolector de basura
        self.cv_bridge = CvBridge()
        self.msg=Banner()
        self.req = StartNavigationTest.Request()
    def image_callback(self, msg):
        global proces
        global centinela
        if proces== True:
            try:
                # Convertir el mensaje de imagen a una imagen de OpenCV
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                # Aplica el filtro de luz a la imagen
                filtered_image = apply_light_filter(cv_image)
                figura=detectionfigura(filtered_image)
                
                # Reconocimiento de colores:
                colorpredmitad=reconocer_color(filtered_image)
                nombrecolor=get_color_name(colorpredmitad)
                # Ejemplo de reconocimiento óptico de caracteres (OCR):
                palabra=reconocerpalabras(filtered_image)
    
                # Publicar el resultad
                self.msg.figure=figura
                self.msg.word=palabra
                self.msg.color=nombrecolor
                centinela=True
            except:
                pass
        
    def perception(self, request, response):
        global centinela
        global proces
        global banner1
        global banner2
        global banner3
        
        if request.banner_a==1:
            bannera=banner1
        elif request.banner_a==2:
            bannera=banner2
        elif request.banner_a==3:
            bannera=banner3
            
        if request.banner_b==1:
            bannerb=banner1
        elif request.banner_b==2:
            bannerb=banner2
        elif request.banner_b==3:
            bannerb=banner3
        #navega al banner a
        #inicia navegacion    
        x=bannera[0][0]
        y=bannera[0][1]
        dire0=self.send_request(x,y)# se ejecuta el servicio de navegacion hasta plt0 y se obtiene la orientacio
        dire0=dire0.answer
        dire1=orientacion(bannera[1])
        self.giro(dire0, dire1)
        
        #escanea banner a y publica
        proces=True
        if centinela==True:
            proces=False
            centinela=False
        self.msg.banner=request.a
        self.publisher.publish(self.msg)
        
        self.giro(dire1,dire0)
        #navega al banner b
        x=bannerb[0][0]
        y=bannerb[0][1]
        dire0=self.send_request(x,y)# se ejecuta el servicio de navegacion hasta plt0 y se obtiene la orientacio
        dire0=dire0.answer
        dire1=orientacion(bannerb[1])
        self.giro(dire0, dire1)
        
        #escanea banner b y publica
        proces=True
        if centinela==True:
            proces=False
            centinela=False
        self.msg.banner=request.b
        self.publisher.publish(self.msg)
        
        self.giro(dire1, dire0)
        
        response.answer= 'Debo identificar el banner'+str(request.a)+'quese encuentra en las coordenadas .COMPLETAR.. y el banner'+str(request.b)+'que se encuentra en las coordenadas...COMPLETAR'
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
