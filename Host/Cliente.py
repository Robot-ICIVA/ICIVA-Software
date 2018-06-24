import time
import requests
import json
def Request(ip, port_server): # el servo que controla phi (plano xy)
    url_FREERUN = "http://"+ ip +":"+ port_server

    print(url_FREERUN)

    try:
        T_Inicio = time.time()

        req = requests.get(url_FREERUN)
        response = req.json()
        T_Final = time.time()
        Dif = T_Final - T_Inicio
        print(response[0][0])
    except:
        print("Error Sending Data")

    return


def main():
    while True:
        time.sleep(0.01)
        Request("127.0.0.1", "8000")


if __name__ == '__main__':
    main()
