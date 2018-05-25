import numpy as np
import matplotlib.pyplot as plt


PWM_D = np.array([18000, 25000, 30000, 35000, 40000, 45000, 46000, 47000, 50000, 55000, 60000, 65000])
RPM_D = np.array([20, 38.48620911, 52.35602094, 58.47953216, 65.98240469, 70.75471698, 73.89162562, 74.25742574, 75.47169811, 76.04562738, 81.41112619, 85.22727273])

np.savetxt('PWM_D.out', PWM_D, fmt='%1.6e')
np.savetxt('RPM_D.out', RPM_D, fmt='%1.6e')

PWM_I = np.array([18000, 25000, 30000, 35000, 40000, 45000, 46000, 47000, 50000, 55000, 60000, 65000])
RPM_I = np.array([33.87916431, 47.20692368, 57.85920926, 62.63048017, 71.85628743, 73.71007371, 73.83100902, 80.35714286, 82.98755187, 85.14664144, 86.87258687, 87.97653959])

np.savetxt('PWM_I.out', PWM_I, fmt='%1.6e')
np.savetxt('RPM_I.out', RPM_I, fmt='%1.6e')

#-----------------------------------------------------------------------------------------------------------------------

# Ajuste polinomial a data
order = 9  # Orden del polinomio

poly_md = np.polyfit(RPM_D, PWM_D, order)
p_md = np.poly1d(poly_md)
np.savetxt('P_md.out', poly_md, fmt='%1.12e')

poly_mi = np.polyfit(PWM_I,RPM_I,  order) #Pendiente
p_mi = np.poly1d(poly_mi)
np.savetxt('P_mi.out', poly_mi, fmt='%1.12e')

# Graficas
Time = np.arange(15000, 65000, 1)

plt.figure()

plt.subplot(2, 1, 1)
#plt.scatter(Time, p_md(Time))

#plt.hold(True)

plt.scatter(PWM_D, RPM_D, color='g')

plt.title("Motor Derecho: Polinomio orden {} ajustado a la data".format(order))

plt.subplot(2, 1, 2)
#plt.scatter(Time, p_mi(Time))

#plt.hold(True)

plt.scatter(PWM_D, RPM_D, color='g')

plt.title("Motor izquierdo: Polinomio orden {} ajustado a la data".format(order))

plt.show()