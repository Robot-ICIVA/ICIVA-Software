import numpy as np
import matplotlib.pyplot as plt
import scipy
from scipy.stats import norm


n=100
a=np.arange(n)

#Centrar
sigma=0

#Escala
escala = 10
alpha = 4*escala
pqc=10*escala

#Pto a evaluar
pto=1000

x_1 = np.linspace(scipy.stats.norm(sigma, alpha).ppf(0.01), scipy.stats.norm(sigma, alpha).ppf(0.99), 100)
#x_1=np.linspace(0,0.01,100)
DesvEstan = 1
miu = 0
X = -np.power(x_1-miu, 2)/(2*np.power(DesvEstan, 2))
Prob = np.exp(X)/(np.sqrt(2*np.pi)*DesvEstan)
plt.plot(x_1, Prob, label='FDP nomal')
plt.title('Función de Densidad de Probabilidad')
plt.ylabel('probabilidad')
plt.xlabel('valores')
plt.show()

print(pqc*scipy.stats.norm(sigma, alpha).pdf(pto))

# Graficando Función de Densidad de Probibilidad con Python
FDP_normal = scipy.stats.norm(sigma, alpha).pdf(x_1) # FDP
plt.plot(x_1, FDP_normal, label='FDP nomal')
plt.title('Función de Densidad de Probabilidad')
plt.ylabel('probabilidad')
plt.xlabel('valores')
plt.show()