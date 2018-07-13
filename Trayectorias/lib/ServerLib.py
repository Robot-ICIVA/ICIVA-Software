import requests
import numpy as np
import time


def circles_robot(response): # busca los circulos del robot en el paquete JSON
    x_cyan = None
    y_cyan = None
    radius_cyan = None
    x_yellow = None
    y_yellow = None
    radius_yellow = None
    for list in response:
        if 'CYAN' in list:
            x_cyan = list[0][0]*100
            y_cyan = list[0][1]*100
            radius_cyan = list[1]*100
        elif 'BLUE' in list:
            x_cyan = list[0][0]*100
            y_cyan = list[0][1]*100
            radius_cyan = list[1]*100
        elif 'YELLOW' in list:
            x_yellow = list[0][0]*100
            y_yellow = list[0][1]*100
            radius_yellow = list[1]*100


    return x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow

def get_balls(response): # busca los circulos del robot en el paquete JSON
    balls = np.zeros([0, 2])
    for list in response:
        if 'RED' in list:
            x_ball = list[0][0] * 100
            y_ball = list[0][1] * 100
            point = np.array([x_ball, y_ball])
            radius_ball = list[1] * 100
            balls = np.append(balls, [point], 0)
    return balls

def get_enemies(response): # busca los circulos del robot en el paquete JSON
    enemies = np.zeros([0, 2])
    for list in response:
        if 'GREEN' in list:
            x_ball = list[0][0]*100
            y_ball = list[0][1]*100
            point = np.array([x_ball, y_ball])
            radius_ball = list[1]*100
            enemies = np.append(enemies, [point],0)
    return enemies

def Request(ip, port_server): # el servo que controla phi (plano xy)
    url = "http://"+ ip +":"+ port_server
    try:

        req = requests.get(url)
        response = req.json()
    except:
        return None

    return response


def get_all():
    x_cyan = None
    y_cyan = None
    x_yellow = None
    robot_pos = None
    ball_pos = None
    head_vector = None
    while isinstance(x_cyan, type(None)) or isinstance(x_yellow, type(None)):
        response = Request("127.0.0.1", "8000")
        if isinstance(response, type(None)):
            print("Server error...Retrying")
            time.sleep(0.5)
        else:
            x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow = circles_robot(response)
            balls = get_balls(response)
            enemies = get_enemies(response)
            if isinstance(x_cyan, type(None)) or isinstance(x_yellow, type(None)):
                print("Error robot pos")
                time.sleep(0.5)
            else:
                # Vectors
                robot_pos = np.array(
                    [x_yellow + (x_cyan - x_yellow) / 2, y_yellow + (y_cyan - y_yellow) / 2])  # posicion del robot
                head_vector = np.array([x_cyan - robot_pos[0], y_cyan - robot_pos[1]])

    return robot_pos, head_vector, balls, enemies

def get_robot_pos():
    x_cyan = None
    y_cyan = None
    x_yellow = None
    robot_pos = None
    head_vector = None
    while isinstance(x_cyan, type(None)) or isinstance(x_yellow, type(None)):
        response = Request("127.0.0.1", "8000")
        if isinstance(response, type(None)):
            print("Server error...Retrying")
            time.sleep(0.5)
        else:
            x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow = circles_robot(response)
            if isinstance(x_cyan, type(None)) or isinstance(x_yellow, type(None)):
                print("Error robot pos")
                time.sleep(0.5)
            else:
                # Vectors
                robot_pos = np.array(
                    [x_yellow + (x_cyan - x_yellow) / 2, y_yellow + (y_cyan - y_yellow) / 2])  # posicion del robot
                head_vector = np.array([x_cyan - robot_pos[0], y_cyan - robot_pos[1]])

    return robot_pos, head_vector