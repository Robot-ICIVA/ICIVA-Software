
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