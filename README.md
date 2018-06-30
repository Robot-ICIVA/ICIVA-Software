# ICIVA-Software
Software utilizado en la realización del proyecto Robot-ICIVA

# Prerrequisitos
Este proyecto se realizó en un ambiente de Anaconda 4.3.30  nombrado como ICIVA

Para instalar Anaconda, diriijase a su  [página de descarga](https://www.anaconda.com/download/).

## Instalación del ambiente ICIVA para Windows y Linux
* Una vez instalado Anaconda, abrir el prompt de anaconda y ejecutar el siguiente comando:
```bash
conda create --name iciva python=3.6 pyserial matplotlib numpy opencv pyyaml requests
```

Lo cual instala python 3.6 y las librerias pyserial, matplotlib, numpy, yaml, requests y opencv

Luego  activar el ambiente en el prompt de anaconda e instalar la libreria de contribuciones de opencv:
```bash
pip install opencv-contrib-python
```


## IDE Python

En nuestro proyecto se empleo Pycharm como IDE de desarrollo:

Para instalar Pycharm, dirijase a su  [página de descarga](https://www.jetbrains.com/pycharm/download/)

Luego de instalar Pycharm, se procede a configurar el interprete para que corresponda al del ambiente ICIVA.
