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
from Robot_Fase2.MotorLib import *
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
        if ack == 0:
            print("again")
            command = ""
            write(port, command)
            time.sleep(0.1)
            write(port, command)
            time.sleep(0.1)
            write(port, command)
            time.sleep(0.5)
            port.reset_input_buffer()
        else:

            #Leo packete tipo M
            buff = np.frombuffer(port.read(1), dtype=np.uint8)
            time.sleep(0.2)
            while port.in_waiting != 0:  # Mientras hallan bytes por leer
                time.sleep(0.1)
                size_toread = port.in_waiting
                data = np.frombuffer(port.read(size_toread), dtype=np.uint8)
                for bytes in data:
                    buff = np.append(buff, [bytes], 0)  # Se guardan como entero los datos recibidos
                    if bytes == 13:
                        print("byte 13 alcanzado")
                        break


            print(buff)
            mx, my, x1, y1, x2, y2, pixels, confidence = decode(buff)
            #mx, my, x1, y1, x2, y2, pixels, confidence = TC(port)
            command = ""
            write(port, command)
            time.sleep(0.1)
            write(port, command)
            time.sleep(0.1)
            write(port, command)
            time.sleep(0.5)
            port.reset_input_buffer()
            # force_idle(port)
            print("confidence {}and pixels{}".format(confidence, pixels))
            if confidence > 0 and pixels > 0:

                print("Object position: mx = {}, my = {}".format(mx, my))
                centrado = giro(mx, my, 10, 10, port)
                if centrado ==1:
                    Amplitud_matrix = np.array([])
                    Amplitud_filtrada = np.array([])

                    # Mido el Voltaje del adc y filtro las medidas para obtener el valor de distancia
                    T_Inicio = time.time()
                    T_Final = time.time()
                    Dif = T_Final - T_Inicio
                    while (Dif < 0.2):
                        ADC = detect_data(port)
                        y = ADC * 3.1 / (2 ** 12 - 1)  # Escalamiento, el voltaje de ref de adc es 3.1
                        Amplitud_matrix = np.append(Amplitud_matrix, [y])
                        T_Final = time.time()
                        Dif = T_Final - T_Inicio

                    Valor_min = Amplitud_matrix[np.argmin(Amplitud_matrix, 0)]
                    indices, = np.where(Amplitud_matrix < (Valor_min + Valor_min * 0.1))

                    for i in indices:  # Filtrado
                        Amplitud_filtrada = np.append(Amplitud_filtrada, [Amplitud_matrix[i]])

                    distancia = poly(np.mean(Amplitud_filtrada))  # Distancia medida
                    print("Distancia = {0:0.2f} cm".format(distancia))
                    if distancia >= 25.0:  # Si la distancia es menor a 15 cm
                        delta = distancia - 25  # Cuan cerca estoy de 25. Si estoy muy cerca disminuir velocidad
                        if (delta <= 5 and delta >= 0):
                            PWM_RD = 22000
                        elif (delta > 5 and delta < 10):
                            PWM_RD = 30000
                        else:
                            PWM_RD = 42000
                        PWM_RI = PWM_RD
                        # PWM_RD = 60000#int(round(poly_rd(50)))+1000
                        # PWM_RI = 60000#int(round(poly_ri(50)))
                        send_PWM(1, PWM_RI, 1, PWM_RD, port)  # Enviar al motor izquierdo, PWM hacia adelante
                        ACK = Micro_comfirm_ACK(port)
                        if ACK == 1:
                            print("Comando recibido")
                        else:
                            print("Comando no recibido")
                    elif distancia < 15.0 and distancia > 7.0:
                        delta = 15 - distancia
                        if (delta <= 5 and delta >= 0):
                            PWM_RD = 22000
                        elif (delta > 5 and delta < 10):
                            PWM_RD = 32000
                        else:
                            PWM_RD = 42000
                        PWM_RI = PWM_RD
                        # PWM_RD = 60000#int(round(poly_rd(50)))+5000
                        # PWM_RI = 60000#Int(round(poly_ri(50)))
                        send_PWM(0, PWM_RI, 0, PWM_RD, port)  # Enviar al motor izquierdo, PWM hacia adelante ACK = Micro_comfirm_ACK(port)
                        if ACK == 1:
                            print("Comando recibido")
                        else:
                            print("Comando no recibido")

            else:
                print("No object")
                send_PWM(0, 0, 1, 30000, port)
                ACK = Micro_comfirm_ACK(port)
                if ACK == 1:
                    print("Comando recibido")
                else:
                    print("Comando no recibido")

            #time.sleep(0.2)





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
