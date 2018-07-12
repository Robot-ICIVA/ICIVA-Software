import serial
import matplotlib.pyplot as plt
import numpy as np
import time
import glob
import os

def main():
    # Cargar datos rueda derecha
    PWMrd_p1 = np.loadtxt("Prueba1/PWMrd.out")
    PWMrd_p2 = np.loadtxt("prueba2/PWMrd.out")
    PWMrd = PWMrd_p1
    PWMrd = np.append(PWMrd, [PWMrd_p2])

    # Cargar datos rueda izquierda
    PWMri_p1 = np.loadtxt("Prueba1/PWMri.out")
    PWMri_p2 = np.loadtxt("prueba2/PWMri.out")
    PWMri = PWMri_p1
    PWMri = np.append(PWMri, [PWMri_p2])

    # Cargar datos rapidez promedio
    Rapidez_p1 = np.loadtxt("Prueba1/Rapidez.out")
    Rapidez_p2 = np.loadtxt("prueba2/Rapidez.out")
    Rapidez = Rapidez_p1
    Rapidez = np.append(Rapidez, [Rapidez_p2])

    # Radio de rueda del carrito (en cm)
    r = 3.5/2.0
    rapidez_angular = Rapidez/r

    # Ajuste polinomial a data de la rueda derecha
    order_rd =3 # Orden del polinomio
    poly_rd = np.polyfit(Rapidez, PWMrd, order_rd)
    p_rd = np.poly1d(poly_rd)
    np.savetxt('Poly_rd.out', poly_rd, fmt='%1.12e')
    print(np.poly1d(p_rd))
    
    # Ajuste polinomial a data de la rueda izquierda
    order_ri = 3 # Orien del polinomio
    poly_ri = np.polyfit(Rapidez, PWMri, order_ri)
    p_ri = np.poly1d(poly_ri)
    np.savetxt('Poly_ri.out', poly_ri, fmt='%1.12e')
    print(np.poly1d(p_ri))

    print("Maximo valor de rapidez = {} cm/s".format(Rapidez[np.argmax(Rapidez)]))
    print("Minimo valor de rapidez = {} cm/s".format(Rapidez[np.argmin(Rapidez)]))
    print("Maximo valor de PWM rd = {}".format(PWMrd[np.argmax(PWMrd)]))
    print("Minimo valor de PWM rd = {}".format(PWMrd[np.argmin(PWMrd)]))
    print("Maximo valor de PWM ri = {}".format(PWMri[np.argmax(PWMri)]))
    print("Minimo valor de PWM ri = {}".format(PWMri[np.argmin(PWMri)]))

    # Graficas
    plt.figure()
    plt.subplot(3, 1, 1)
    time_cont = np.arange(0, 15, 0.01)
    plt.scatter(Rapidez, PWMrd,  c = 'r')
    plt.plot(time_cont, p_rd(time_cont) )
    plt.title("Polinomio orden {} ajustado al PWM rueda derecha".format(order_rd))
    plt.legend(["Polinomio", "Data tomada"])
    plt.xlabel("Rapidez(cm/s)")
    plt.ylabel("PWM")
    plt.xlim([0, 15])
    plt.ylim([0, 65000])
    plt.subplot(3, 1, 3)
    time_cont = np.arange(0, 15, 0.01)
    plt.scatter(Rapidez, PWMri, c = 'g')
    plt.plot(time_cont, p_ri(time_cont) )
    plt.title("Polinomio orden {} ajustado al PWM rueda izquierda".format(order_ri))
    plt.legend(["Polinomio", "Data tomada"])
    plt.xlabel("Rapidez(cm/s)")
    plt.ylabel("PWM")
    plt.xlim([0, 15])
    plt.ylim([0, 65000])
    plt.show()



#files = glob.glob('*.out')


if __name__ == "__main__": main()
