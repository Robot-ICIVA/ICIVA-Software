
import numpy as np
from ColorBallTracker.CaracterizacionPlanta.lib.SerialLib import *
def send_PWM(Dir1, PWM1, Dir2, PWM2, port ):
    packet_size = 6+1 # 6 datos y un comando
    Trama_FREERUN = np.ones(packet_size + 2, dtype="uint8")
    Trama_FREERUN[0] = 0xff
    Trama_FREERUN[1] = 0x07
    Trama_FREERUN[2] = 0x01  # comando ADC
    velup1 = (0xff00 & PWM1)>>8
    vellow1 = 0x00ff & PWM1
    velup2 = (0xff00 & PWM2)>>8
    vellow2 = 0x00ff & PWM2
    Trama_FREERUN[3] = Dir1 # 1 hacia adelante
    Trama_FREERUN[4] = velup1
    Trama_FREERUN[5] = vellow1
    Trama_FREERUN[6] = Dir2 # 1 hacia adelante
    Trama_FREERUN[7] = velup2
    Trama_FREERUN[8] = vellow2
    number1 = (2**8)*velup2+vellow2
    number2 = (2 ** 8) * velup2 + vellow2
    #print("Motor1, Dir= {}, PWM ={}".format(Dir1, number1))
    #print("Motor2, Dir= {}, PWM ={}".format(Dir2, number2))
    port.write(bytearray(Trama_FREERUN))

    return 0

def giro(mx, my, tx, ty, port): # Recibe el centro de masa de la camara y el thresholdde la ventana
    cx = 40
    cy = 71
    if mx > cx +tx : # Mover rueda derecha, ir  a la izquierda

        send_PWM(0, 0, 1, 25000, port)
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")
        time.sleep(0.2)
        send_PWM(0, 0, 0, 0, port)
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")
    elif mx < cx-tx: # Mover a la derecha
        send_PWM(1, 25000, 0, 0, port)
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")
        time.sleep(0.2)
        send_PWM(0, 0, 0, 0, port)
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")
    else:
        print("Objeto  centrado")
        return  1


    return 0


def align(angle, th, port): # Recibe el centro de masa de la camara y el thresholdde la ventana
    if angle < 180 and angle > th : # Mover rueda derecha, ir  a la izquierda

        send_PWM(0, 0, 0, 25000, port)
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")
    elif angle > -180 and angle <-th: # Mover a la derecha
        send_PWM(1, 25000, 0, 0, port)
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")

    else:
        send_PWM(0, 0, 0, 0, port)
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")
        return 1


    return 0

def move_forward( dist_obj, dist_des, th, port):
    if dist_obj > dist_des+th :  # Si la distancia es menor a 15 cm
        PWM_RD = 33000
        PWM_RI = 30000
        # PWM_RD = 60000#int(round(poly_rd(50)))+1000
        # PWM_RI = 60000#int(round(poly_ri(50)))
        send_PWM(1, PWM_RI, 0, PWM_RD, port)  # Enviar al motor izquierdo, PWM hacia adelante
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")
    elif dist_obj < dist_des-th:
        PWM_RD = 35000
        PWM_RI = 30000
        # PWM_RD = 60000#int(round(poly_rd(50)))+1000
        # PWM_RI = 60000#int(round(poly_ri(50)))
        send_PWM(0, PWM_RI, 1, PWM_RD, port)  # Enviar al motor izquierdo, PWM hacia atras
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")
    else:
        send_PWM(0, 0, 0, 0, port)
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")

        return 1


    return 0

def control_w(angle, pwm_init, kri, krd, port):
    error = angle
    if error > 0:  # error positivo, rueda derecha
        PWM_RI = pwm_init
        PWM_RD = pwm_init+krd*error

        if PWM_RD> 65000:
            PWM_RD = 65000
        if PWM_RI> 65000:
            PWM_RI = 65000
        send_PWM(1, int(PWM_RI), 0, int(PWM_RD), port)  # Enviar al motor izquierdo, PWM hacia adelante
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")

    else: # Muevo rueda izquierda
        error = -error
        PWM_RD = pwm_init-1000
        PWM_RI = pwm_init + kri * error
        if PWM_RD> 65000:
            PWM_RD = 65000
        if PWM_RI> 65000:
            PWM_RI = 65000
        send_PWM(1, int(PWM_RI), 0, int(PWM_RD), port)  # Enviar al motor izquierdo, PWM hacia adelante
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")



    return

