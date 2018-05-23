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
    cv2.circle(canvas, c1, 20, (0, 255, 0), -1)     # circulo  verde (adelante)
    cv2.circle(canvas, c2, 20, (25, 122, 255), -1) # circulo naranja (atras)
    cv2.circle(canvas, pos, 20, (255, 0, 0), -1)    # circulo azul (centro)
    cv2.arrowedLine(canvas, pos, c1, (255, 0, 0), 5)
    return  canvas

def trayectoria_circular ( canvas, r, theta, centro, w, h):
    canvas = cv2.circle(canvas, centro, r, (0, 255, 0), 2)  # circulo  verde (adelante)
    cx, cy = centro
    x = int(round(r*np.cos(theta) +cx))
    y = int(round(r*np.sin(theta)+cy))
    dx = int(round(-np.sin(theta)*w/100*1.2))
    dy = int(round(np.cos(theta)*h/100*1.2))

    return canvas, x, y, dx, dy

def main():
    # c1, circulo de cabeza de robot
    # c2, circulo de parte trasera del robot
    theta = 0
    w, h = 1080, 720
    while True:
        theta = theta+1
        canvas = 255*np.ones((h, w, 3), np.uint8)
        img_ret, cx, cy, dx, dy = trayectoria_circular(canvas.copy(), 170, np.radians(theta), (int(w/2), int(h/2)), w, h ) # centro de imagen
        pos = (cx, cy)
        vector = (dx, dy)
        print(vector)
        img = draw_robot(canvas.copy(), pos, vector)
        img = cv2.flip(img, 0)
        time.sleep(0.1)
        cv2.imshow("drawing", img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            break





if __name__ == "__main__": main()