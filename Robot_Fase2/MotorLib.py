
import numpy as np

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
    print("Motor1, Dir= {}, PWM ={}".format(Dir1, number1))
    print("Motor2, Dir= {}, PWM ={}".format(Dir2, number2))
    port.write(bytearray(Trama_FREERUN))

    return 0