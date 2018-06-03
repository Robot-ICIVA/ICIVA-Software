if distancia >= 20.0:  # Si la distancia es menor a 15 cm
    PWM = 35000
    send_PWM(1, 1, 33000, port)  # Enviar al motor izquierdo, PWM hacia adelante
    time.sleep(0.05)
    send_PWM(0, 1, PWM, port)  # Enviar al motor derecho, PWM hacia adelante
elif distancia < 20.0:
    PWM = 1
    send_PWM(1, 1, PWM, port)  # Enviar al motor izquierdo, PWM hacia adelante
    time.sleep(0.05)
    send_PWM(0, 1, PWM, port)  # Enviar al motor derecho, PWM hacia adelante

def send_PWM(Motor, Dir, PWM, port):
    Trama_FREERUN = bytearray([0xff, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00])
    velup = (0xff00 & PWM)>>8
    vellow = 0x00ff & PWM
    Trama_FREERUN[4] = Motor
    Trama_FREERUN[5] = Dir # 1 hacia adelante
    Trama_FREERUN[6] = velup
    Trama_FREERUN[7] = vellow
    number = (2**8)*velup+vellow
    print("Motor = {}, Dir= {}, PWM ={}".format(Motor, Dir, number))
    port.write(Trama_FREERUN)

    return