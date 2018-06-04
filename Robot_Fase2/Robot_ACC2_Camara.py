"""#################################INFORMACION###############################
*     Filename    : ConsolaMotores.py
*     Project     : ICIVA
*     Board       : Demoqe
*     Autor       : Luis Lujano (13-10775)
*     GitHub      : https://github.com/Lujano
*     Sensors     : ----
* ###########################################################################"""

"""
Descripcion:
    Este programa estra diseñado para que el robot matenga una distancia fija del objetivo (menor a 15 cm) empleando 
    para ello un sensor infrarrojo de distancia, y  una camara (CMUcamv2.1)
"""


import time
import numpy as np
from Robot_Fase2.CamaraLib import *
from Robot_Fase2.SerialLib import *
import matplotlib.pyplot as plt
import cv2


# La CAMARA tiene una resolucion de 80x143
cx = 40
cy = 70
offsety= 20
offsetx = 20
move_band = 0 # bandera de movimiento si es 1, se  debe mover a la dercha. Si es 2 se mueve hacia la izquierda.
 # Si es 0  no se mueve
# Rmean=128, Gmean=20, Bmean=20, Rdev=17, Gdev=3, Bdev=3
Color = "110 150 10 30 10 30"
def main():
    port = open_port()

    T_Inicio = time.time()
    T_Final = time.time()
    Dif = T_Final-T_Inicio
    poly_infra_dir = np.loadtxt('../Calibracion/Polinomio_Ajuste_Infra2.out') #Calibracion del sharp
    poly_rd_dir = np.loadtxt('../Calibracion/Polinomio_RD.out')
    poly_ri_dir = np.loadtxt('../Calibracion/Polinomio_RI.out')
    poly = np.poly1d(poly_infra_dir)
    poly_rd =np.poly1d(poly_rd_dir)
    poly_ri =np.poly1d(poly_ri_dir)

    print("Inicio")
    Mircro_reset(port)
    ACK = Micro_comfirm_ACK(port)
    while(ACK != 1):
        print("El micro no se ha resetiado")
    print("Micro reseteado")

    while True:
        force_idle(port)
        port.reset_input_buffer()
        command = "TC " + Color
        write(port, command)
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")
        ack = comfirm_ACK(port)
        print("ACK = {}".format(ack))
        mx, my, x1, y1, x2, y2, pixels, confidence = TC(port)
        if confidence > 0 and pixels > 0:
            print("Object position: mx = {}, my = {}".format(mx, my))
            giro(mx, my, 5, 5, port)
        else:

            print("No object")
        time.sleep(0.2)





        # buff = read_buffer(port)  # leer buffer serial de entrada
        # time.sleep(0.1)
        # idle, packet = idle_state(buff)
        # print("aq")
        # mx, my, x1, y1, x2, y2, pixels, confidence = decode(packet)
        # print("mx={}, my={}, x1={}, y1={}, x2={}, y2={}".format(mx, my, x1, y1, x2, y2))
        # print("pixels = {}, confidende = {}".format(pixels, confidence))
        # force_idle(port)
        #
        # if mx < cx-offsetx : # el objeto se encuentra a la izquierda
        #     move_band = 2
        # elif mx > cx+offset: # el objeto se encuenra a la derecha
        #     move_band = 1
        # else:
        #     move_band = 0
        #
        #
        #
        # PWM = 35000
        # #send_PWM(1, 0, PWM, port)  # Enviar al motor izquierdo, PWM hacia adelante
        # time.sleep(0.05)
        # #send_PWM(0, 0, PWM, port)  # Enviar al motor derecho, PWM hacia adelante



    print("Finished")
    close_port(port)



if __name__ == "__main__": main()
