from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label
from kivy.lang import Builder
from kivy.core.window import Window
from kivy.clock import Clock
from kivy.uix.button import Button

import socket
import errno
import math

import time
from kivy.uix.switch import Switch
from kivy.uix.popup import Popup
from kivy.uix.gridlayout import GridLayout
from kivy.properties import ObjectProperty

from robot import Robot
from map import Map
import os
from numba import jit

import requests
import wget

import os.path
from zipfile import ZipFile
import subprocess
import atexit
import warnings

#Build SimTwo ENV
response = requests.get("https://api.github.com/repos/P33a/SimTwo/releases/latest")
print(response.json()["assets"][0]["browser_download_url"])
filename = response.json()["assets"][0]['name']

if not os.path.exists(response.json()["assets"][0]['name']):
    url = response.json()["assets"][0]["browser_download_url"]
    filename = wget.download(url)

with ZipFile(filename, 'r') as zipObject:
    listOfFileNames = zipObject.namelist()
    for fileName in listOfFileNames:
        if fileName.endswith('.exe') or fileName.endswith('.dll'):
            zipObject.extract(fileName, 'SimTwo')

if not os.path.exists("SimTwo/Scene.cfg"):
    f = open("SimTwo/Scene.cfg", "w")
    f.write("MobileRobot")
    f.close()

while not os.path.exists("SimTwo\SimTwo2020.exe"):
    time.sleep(1)

if os.name == 'nt':
    p = subprocess.Popen(
        [os.path.abspath("SimTwo\SimTwo2020.exe")]
    )
else:
    warnings.warn("Sorry, unable to run SimTwo\SimTwo2020.exe please open it mannually.")

time.sleep(2)

my_port = 8421
simtwo_port = 9808

Builder.load_file('ui.kv')
Builder.load_file('MapEditor.kv')

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind(("127.0.0.1", my_port))
udp_socket.settimeout(0)

class UI(FloatLayout):
    _map_grid = ObjectProperty(GridLayout())
    _switch_post_processing = ObjectProperty(Switch())
    _switch_control = ObjectProperty(Switch())

    def __init__(self):
        super(UI, self).__init__()

        map_width = 16
        map_height = 16

        self.target_pos = (15, 0)

        self.map = Map(map_width, map_height)

        self.path = None
        self.waypoint_was_reached = False
        self.path_idx = 0

        self.map_buttons = None

        self.create_map_ui()
        self.robot = Robot()
        self.control = False

        Clock.schedule_interval(self.receive_udp_data, 0.001)

        data = "Manual\r\n"

        udp_socket.sendto(bytes(data.encode("ascii")), ("127.0.0.1", simtwo_port))

        self._switch_control.bind(active=self.control_mode_callback)
        self._switch_post_processing.bind(active=self.switch_post_processing_callback)


    def switch_post_processing_callback(self, *args):
        self.path = None

    def control_mode_callback(self, *args):
        if self._switch_control.active:
            self.control = True
            data = "Script\r\n"
            udp_socket.sendto(bytes(data.encode("ascii")), ("127.0.0.1", simtwo_port))
        else:
            self.control = False
            data = "Manual\r\n"
            udp_socket.sendto(bytes(data.encode("ascii")), ("127.0.0.1", simtwo_port))

    def receive_udp_data(self, dt):
        data = None
        try:
            data = udp_socket.recv(my_port)
        except Exception as e:
            if e.errno == 10054:
                self.simtwo_error_popup()

        if data:
            data = data.decode('ascii').split('\r\n')[:-1]

            if data[0] == "RX":
                start = time.time_ns()
                self.control_loop(data)
                #print((time.time_ns()-start)/(10**6))

    def control_loop(self, data):

        self.robot.update_robot(pos=(float(data[1]), float(data[3])),
                                theta_deg=float(data[5]),
                                lidar_dists=[float(data[11]), float(data[12]), float(data[13]), float(data[14]),
                                             float(data[15])])
        self.robot.add_tolerance_to_lidar_points(0.0075)
        points_for_map_update = [x for x in self.robot.lidar_points]
        points_for_map_update_are_occupied = [x for x in self.robot.lidar_point_is_wall]
        points_for_map_update.append(self.robot.pos)
        points_for_map_update_are_occupied.append(False)
        map_has_changed = self.map.update_map(points_for_map_update, points_for_map_update_are_occupied)
        simplified_path = None
        full_path = None

        if self._switch_post_processing.active or (not self._switch_post_processing.active and map_has_changed) or (self.path is None):
            full_path, simplified_path = self.map.shortest_path(self.robot.pos, self.map.map_to_world_coords(self.target_pos), self._switch_post_processing.active)
            self.path_idx = 0

            if simplified_path:
                self.path = simplified_path
            else:
                self.path = full_path
        else:
            full_path = self.path

        if self.path:
            waypoint = self.path[self.path_idx]

            pos_err = abs(math.sqrt(math.pow(abs(self.robot.pos[0]) - waypoint[0], 2) + math.pow(
                abs(self.robot.pos[1]) - waypoint[1], 2)))

            if pos_err < 0.01 and self.path_idx < (len(self.path) - 1):
                self.path_idx += 1

                waypoint = self.path[self.path_idx]
                pos_err = abs(math.sqrt(math.pow(abs(self.robot.pos[0]) - waypoint[0], 2) + math.pow(
                    abs(self.robot.pos[1]) - waypoint[1], 2)))

            if self.path_idx != (len(self.path) - 1) or (self.path_idx == (len(self.path) - 1) and pos_err > 0.1):
                theta_ref = math.atan2(waypoint[1] - self.robot.pos[1], waypoint[0] - self.robot.pos[0])
                theta_err = theta_ref - self.robot.theta

                while theta_err > math.radians(180):
                    theta_err -= math.radians(360)
                while theta_err < math.radians(-180):
                    theta_err += math.radians(360)

                if abs(theta_err) > math.radians(10):
                    self.send_robot_speeds(0, theta_err * 3)
                else:
                    self.send_robot_speeds(0.2, theta_err * 2)
            else:
                self.send_robot_speeds(0, 0)
        else:
            self.send_robot_speeds(0, 0)

        self.paint_map()
        if self.path:
            self.paint_path_on_map(full_path, simplified_path)
        if self.robot.pos:
            self.paint_robot_on_map()
        if self.target_pos:
            self.paint_target_on_map()

    def send_robot_speeds(self, linear_velocity, angular_velocity):
        if self.control:
            data = "V\r\n" + str(linear_velocity) + "\r\n" + "W\r\n" + str(angular_velocity) + "\r\n"
            udp_socket.sendto(bytes(data.encode("ascii")), ("127.0.0.1", simtwo_port))


    def clean_callback(self):
        self.map.clean_map()
        self.paint_map()

    def paint_path_on_map(self, full_path, simplified_path=None):
        if full_path:
            for idx in range(len(full_path)):
                col, row = self.map.world_to_map_coords(full_path[idx])
                self.map_buttons[row][col].background_color = [0, 0.5, 0, 1]
                self.map_buttons[row][col].text = str(idx)
        if simplified_path:
            for idx in range(len(simplified_path)):
                col, row = self.map.world_to_map_coords(simplified_path[idx])
                self.map_buttons[row][col].background_color = [1, 0.5, 0, 1]
                self.map_buttons[row][col].text = 'PP'

    def paint_robot_on_map(self):
        col, row = self.map.world_to_map_coords(self.robot.pos)
        self.map_buttons[row][col].background_color = [0.5, 0, 1]
        self.map_buttons[row][col].text = 'R'

    def paint_target_on_map(self):
        col, row = self.target_pos
        self.map_buttons[row][col].background_color = [0.5, 0.5, 1]
        self.map_buttons[row][col].text = 'T'

    def create_map_ui(self):
        self._map_grid.clear_widgets()
        self._map_grid.cols, self._map_grid.rows = self.map.get_map_size()

        self.map_buttons = [[Button() for _ in range(self._map_grid.cols)] for _ in range(self._map_grid.rows)]

        def create_callback(col, row):
            return lambda x: self.update_target_pos((col, row))

        for row in range(self._map_grid.rows):
            for col in range(self._map_grid.cols):
                self.map_buttons[row][col].bind(on_press=create_callback(col, row))
                self._map_grid.add_widget(self.map_buttons[row][col])

        self.paint_map()

    def update_target_pos(self, point):
        if self.map.is_map_coord_occupied(point):
            self.show_target_point_is_wall_error_popup()
        else:
            self.send_robot_speeds(0, 0)
            self.target_pos = point
            self.path = None


    def paint_map(self):
        for row in range(self._map_grid.rows):
            for col in range(self._map_grid.cols):
                self.map_buttons[row][col].disabled = False
                self.map_buttons[row][col].background_disabled_normal = ""
                self.map_buttons[row][col].background_normal = ""
                self.map_buttons[row][col].text = ""
                self.map_buttons[row][col].disabled_color = [1, 1, 1, 1]
                if self.map.is_map_coord_occupied((col, row)):
                    self.map_buttons[row][col].background_color = [204 / 255, 153 / 255, 0 / 255, 1]
                else:
                    self.map_buttons[row][col].background_color = [67 / 255, 67 / 255, 67 / 255, 1]

    def send_waypoint(self, path):
        data = "TC\r\n"
        for point in path:
            data += "TP\r\n" + str(point[0]) + "\r\n" + str(point[1]) + "\r\n"
        data += "GO\r\n"

        udp_socket.sendto(bytes(data.encode("ascii")), ("127.0.0.1", simtwo_port))

    def create_map_callback(self):
        if self.map:
            show = MapEditor()
            show.create(self.map.map_matrix)
            popup = Popup(title='Map Creation',
                          content=show,
                          size_hint=(None, None), size=(400, 400))
            popup.open()
        else:
            self.show_load_error_popup()

    @staticmethod
    def show_map_error_popup():
        popup = Popup(title='404: Map not found',
                      content=Label(text='Please load a map!'),
                      size_hint=(None, None), size=(200, 100))
        popup.open()

    @staticmethod
    def show_target_point_is_wall_error_popup():
        popup = Popup(title='Target Point Error',
                      content=Label(text='Target point cannot be a wall!\n Please choose another point.'),
                      size_hint=(None, None), size=(250, 100))
        popup.open()

    @staticmethod
    def simtwo_error_popup():
        popup = Popup(title='Unable to communnicate to SimTwo\SimTwo2020.exe',
                      content=Label(text='Please open SimTwo\SimTwo2020.exe manually'),
                      size_hint=(None, None), size=(350, 150))
        popup.open()


class MapEditor(FloatLayout):
    _create_map_grid = ObjectProperty(GridLayout())

    def __init__(self):
        super(MapEditor, self).__init__()

        self.map_buttons = [[Button() for _ in range(16)] for _ in range(16)]

    def save_map_callback(self):
        cur_path = os.path.dirname(__file__)

        new_path = os.path.relpath('SimTwo\MobileRobot\map.txt', cur_path)

        f = open(new_path, "w", newline='\r\n')

        for row in range(self._create_map_grid.rows):
            for col in range(self._create_map_grid.cols):
                if self.map_buttons[row][col].background_color == [67 / 255, 67 / 255, 67 / 255, 1]:
                    f.write("1")
                else:
                    f.write("2")
            f.write("\n")
        f.close()
        data = "map\r\n"
        udp_socket.sendto(bytes(data.encode("ascii")), ("127.0.0.1", simtwo_port))

    def paint_wall(self, col, row):

        if self.map_buttons[row][col].background_color == [67 / 255, 67 / 255, 67 / 255, 1]:
            self.map_buttons[row][col].background_color = [204 / 255, 153 / 255, 0 / 255, 1]
        else:
            self.map_buttons[row][col].background_color = [67 / 255, 67 / 255, 67 / 255, 1]

    def create(self, map_matrix):
        self._create_map_grid.clear_widgets()
        self._create_map_grid.cols, self._create_map_grid.rows = 16, 16 #self.map.get_map_size()

        cur_path = os.path.dirname(__file__)

        new_path = os.path.relpath('SimTwo\MobileRobot\map.txt', cur_path)
        f = open(new_path, "r").read().replace("\r","").split("\n")

        for row in range(self._create_map_grid.rows):
            for col in range(self._create_map_grid.cols):
                #map_matrix[row,col] = f[0]
                map_matrix[row, col] = int(f[row][col])-1

        def create_callback(col, row):
            return lambda x: self.paint_wall(col, row)

        for row in range(self._create_map_grid.rows):
            for col in range(self._create_map_grid.cols):
                self.map_buttons[row][col].bind(on_press=create_callback(col, row))
                self._create_map_grid.add_widget(self.map_buttons[row][col])

        for row in range(self._create_map_grid.rows):
            for col in range(self._create_map_grid.cols):
                self.map_buttons[row][col].disabled = False
                self.map_buttons[row][col].background_disabled_normal = ""
                self.map_buttons[row][col].background_normal = ""
                self.map_buttons[row][col].text = ""
                self.map_buttons[row][col].disabled_color = [1, 1, 1, 1]
                if map_matrix[row, col] == 0:
                    self.map_buttons[row][col].background_color = [67 / 255, 67 / 255, 67 / 255, 1]
                else:
                    self.map_buttons[row][col].background_color = [204 / 255, 153 / 255, 0 / 255, 1]


def my_exit_function():
    if 'p' in globals():
        p.terminate()
        p.wait()

class MyApp(App):
    atexit.register(my_exit_function)

    ui = UI()

    def build(self):
        self.title = "SLAM"
        return self.ui


if __name__ == '__main__':
    Window.size = (600, 500)
    Window.minimum_width, Window.minimum_height = (600, 500)
    MyApp().run()
