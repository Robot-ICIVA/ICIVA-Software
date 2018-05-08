"""
* #################################INFORMACION###############################
*     Filename    : Trayectoria.py
*     Project     : ICIVA-ROBOT
*     Board       : Demoqe
*     Autor       : Luis Lujano (13-10775)
*     GitHub      : https://github.com/Lujano
*     Sensors     : ---
* ###########################################################################

Descripcion:

"""
import cv2
import numpy as np
import time
def draw_robot(canvas, pos, head_vector):
    dr = 5 # distancia en pixeles entre el centro geometrico y el centro del circulo de adelante
    vx, vy = head_vector
    vx, vy = int(round(vx)), int(round(vy))
    ccx, ccy = pos
    c1 = (ccx+dr*vx, ccy+dr*vy)
    c2 = (ccx-dr*vx, ccy-dr*vy)
    # cx3, cy3 = c1[0] / 2 + c2[0] / 2, c1[1] / 2 + c2[1] / 2
    # c3 = (int(cx3), int(cy3))
    cv2.circle(canvas, c1, 40, (0, 255, 0), -1)     # circulo  verde (adelante)
    cv2.circle(canvas, c2, 40, (25, 122, 255), -1) # circulo naranja (atras)
    #cv2.circle(canvas, pos, 20, (255, 0, 0), -1)    # circulo azul (centro)
    #cv2.arrowedLine(canvas, pos, c1, (255, 0, 0), 5)
    return  canvas
def reconst(canvas, pos, circle1):
    cx1, cy1, radio1, r1, g1, b1 = circle1
    cv2.circle(canvas, pos, 20, (255, 0, 0), -1)    # circulo azul (centro)
    cv2.arrowedLine(canvas, pos, (cx1, cy1), (0, 0, 0), 5)
    return canvas
def identify_head(circle1, circle2):
    cx1, cy1, radio1, r1, g1, b1 = circle1
    cx2, cy2, radio2, r2, g2, b2 = circle2
    if g1 > 200:  # circulo verde adelante
        return circle1
    else:
        return circle2

def trayectoria_circular ( canvas, r, theta, centro, w, h):
    canvas = cv2.circle(canvas, centro, r, (0, 255, 0), 2)  # circulo  verde (adelante)
    cx, cy = centro
    x = int(round(r*np.cos(theta) +cx))
    y = int(round(r*np.sin(theta)+cy))
    dx = int(round(-np.sin(theta)*w/100*2))
    dy = int(round(np.cos(theta)*h/100*2))

    return canvas, x, y, dx, dy

def mean_color (image, cx, cy, r): # color promedio  de circulo
    w, h = image.shape[1], image.shape[0]
    canvas = np.zeros((h, w, 3), dtype='uint8')
    cv2.circle(canvas, (cx, cy), r, (255,255, 255), -1)
    gray_canvas = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)  # Cambiar a escala de grises la mascara de puntos
    img2 = cv2.bitwise_and(image, image, mask= gray_canvas) # And de la mascara con cuadra y la imagen original
    mean_color = cv2.mean(img2, mask = gray_canvas) # Color extraido

    return mean_color
def draw_circle(img , circle):
    cx, cy, radio, r, g, b = circle
    cv2.circle(img, (cx, cy), radio, (b, g, r), -1)
    # draw the center of the circle
    cv2.circle(img, (cx, cy), 2, (0, 0, 0), 3)

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, "RGB = ({0:0.0f}, {1:0.0f}, {2:0.0f})".format(r, g, b), (cx - 80, cy - 50), font, 0.5,
                (20, 200, 20), 1, cv2.LINE_AA)
    return img

def get_pos ( circle1, circle2): # centro circulo 1 y dos, respectivamente
    cx1, cy1, radio1, r1, g1, b1 = circle1
    cx2, cy2, radio2, r2, g2, b2 = circle2
    cx3, cy3 = (cx1+cx2) / 2, (cy1 +cy2) / 2
    pos = (int(round(cx3)), int(round(cy3)))
    return pos # posicion del vehiculo

def main():
    # c1, circulo de cabeza de robot
    # c2, circulo de parte trasera del robot
    theta = 0
    w, h = 640, 480
    circle1 = () # circulo con formato (centro, radio, r, g, b)
    circle2 = ()
    while True:
        theta = theta+1
        canvas = 255*np.ones((h, w, 3), np.uint8)
        img_ret, cx, cy, dx, dy = trayectoria_circular(canvas.copy(), 170, np.radians(theta), (int(w/2), int(h/2)), w, h ) # centro de imagen
        pos = (cx, cy)
        vector = (dx, dy)
        img = draw_robot(canvas.copy(), pos, vector)
        img = cv2.flip(img, 0)
        time.sleep(0.1)
        #img = cv2.medianBlur(img, 5)
        cimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1,20,
                            param1=30,param2=15,minRadius=40,maxRadius=80)
        check = 0
        if type(circles) != type(None):
            circles = np.uint16(np.around(circles))
            count = 0
            for i in circles[0, :]:
                # draw the outer circle
                cx, cy = i[0], i[1]
                radio = i[2]
                b, g, r, _ = mean_color(img.copy(), cx, cy, radio)
                if count == 0:
                    circle1 = (cx, cy, radio, r, g, b)
                else:
                    circle2 = (cx, cy, radio, r, g, b)
                    check = 1
                count = count + 1

        if check == 1:
            imagen_circulo1 =  draw_circle(255*np.ones((h, w, 3), np.uint8), circle1)
            imagen_circulo2 = draw_circle(imagen_circulo1, circle2)

            reconstruccion = reconst(imagen_circulo2.copy(), get_pos(circle1, circle2), identify_head(circle1, circle2))

            cv2.imshow('detected circles', reconstruccion)
        cv2.imshow('Original', img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            break





if __name__ == "__main__": main()