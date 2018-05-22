import serial
import matplotlib.pyplot as plt
import numpy as np
import time
import glob
import os

def main():
    directory = 'Data'
    a= os.listdir(directory)
    Distance_matrix = np.array([])
    Voltage_matrix = np.array([])
    for element in a:
        Data = np.loadtxt(directory+'/'+element)
        print(element)
        Valor_min = Data[np.argmin(Data, 0)]
        indices, = np.where(Data < (Valor_min + Valor_min * 0.1))
        Data_filtrada = np.array([])
        for i in indices:
            Data_filtrada = np.append(Data_filtrada, [Data[i]])
        np.savetxt("Data_Filtrada/"+element, Data_filtrada, fmt='%1.8e')



#files = glob.glob('*.out')


if __name__ == "__main__": main()