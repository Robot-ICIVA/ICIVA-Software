# EC3883-ColorBallTracker
Aplicación de visión por computadora que usa una cámara aérea para rastrear la posición de varias pelotas de color que están en el piso. Un marcador de realidad aumentada es usado como referencia para marcar la esquina inferior izquierda de la cancha, y medir la la posición en metros de las pelotas.

# Uso

Para iniciar el programa:

#### Linux

```sh
python main.py
```

#### Windows con WinPython

```sh
cd ../BallLocatingServer
python main.py
```


Si se tiene más de una cámara conectada a la computadora se puede usar la opción **-c Número** para seleccionar qué camara se va a usar. Ejemplo para usar la cámara **0** (default)

```sh
python main.py -c 0
```

Si se desea probar el programa con una imágen estática, se puede usar la opción **-i**. Ejemplo con las imágenes incluidas:

```sh
python main.py -i "figures/AR_and_balls_3.png"
```


# Instalación

Esta aplicación está escrita en Python3

#### Requerimientos:
- Numpy
- OpenCV + Contrib Modules.

#### Procedimiento:

Clona el repositorio a un sistema local con los requerimientos instalados.
```sh
git clone https://github.com/SaidAlvarado/EC3883-ColorBallTracker.git
```
