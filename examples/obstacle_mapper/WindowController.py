import tkinter
import tkinter.ttk
from PIL import Image, ImageTk
import threading

import time

from MapCoodinator import MapCoodinator
from DroneController import DroneController
import ObstacleDetector

class WindowController(tkinter.Frame):

    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.master.title('ObstacleMapper')
        self.pack()

        self.droneController = DroneController()

        time.sleep(4)

        home_lat = self.droneController.vehicle.location.global_frame.lat
        home_lon = self.droneController.vehicle.location.global_frame.lon
        self._mapCoordinator = MapCoodinator(home_lat, home_lon, 19)
        self.create_widgets()

        point_x, point_y = self._mapCoordinator.get_point_on_image(home_lat, home_lon)
        self.draw_home_point(point_x, point_y)
        print("drown points are {0}, {1}".format(point_x, point_y))

        # self.after(100, self.continuous_task)
        self.droneController.arm_and_takeoff(20)

        self.continuous_task_thread = threading.Thread(target=self.continuous_task)
        self.continuous_task_thread.start()


    def create_widgets(self):
        self.button_go_to_click_point = tkinter.ttk.Button(self, text=u'go to click point', command=self.go_to_click_point)
        self.button_guid_to_click_point = tkinter.ttk.Button(self, text=u'guid to click point', command=self.set_guid_to_click_point)

        self.label_description = tkinter.ttk.Label(self, text='Mouse position')
        self.label_description.grid(row=0, column=1)
        self.button_go_to_click_point.grid(row=1, column=1, columnspan=2)
        self.button_guid_to_click_point.grid(row=2, column=1,columnspan=2)

        self.test_canvas = tkinter.Canvas(self, bg='lightblue', width=256*3, height=256*3, highlightthickness=0)
        self.test_canvas.grid(row=0, column=0, rowspan=7)

        self.img = ImageTk.PhotoImage(self._mapCoordinator.generateImage())
        self.test_canvas.create_image(0, 0, image=self.img, anchor=tkinter.NW)
        self.test_canvas.bind('<ButtonPress-1>', self.start_pickup)

    def go_to_click_point(self):
        if self.droneController.click_point_lat != None and self.droneController.click_point_lon != None:
            print("let's go")
            self.droneController.go_to_click_point()

    def set_guid_to_click_point(self):
        self.guid_to_click_point_thread = threading.Thread(target=self.while_guid_to_task)
        self.guid_to_click_point_thread.start()

    def while_guid_to_task(self):
        if self.droneController.click_point_lat != None and self.droneController.click_point_lon != None:
            self.droneController.guid_to_click_point()

    def start_pickup(self, event):
        self.droneController.click_point_lat, self.droneController.click_point_lon = self._mapCoordinator.getGPS(event.x, event.y)
        self.reflesh_click_point(event.x, event.y)

    def draw_home_point(self, point_x, point_y):
        plot_size = 6
        self.test_canvas.create_oval(point_x - plot_size/2, point_y - plot_size/2, point_x + plot_size/2, point_y + plot_size, fill="red")


    def reflesh_click_point(self, point_x, point_y):
        plot_size = 4
        self.test_canvas.delete("click_point")
        self.test_canvas.create_oval(point_x - plot_size/2, point_y - plot_size/2, point_x + plot_size/2, point_y + plot_size, tag="click_point")

    def draw_obstacle(self, lat, lon):
        point_x, point_y = self._mapCoordinator.get_point_on_image(lat, lon)
        plot_size = 6
        self.test_canvas.create_oval(point_x - plot_size/2, point_y - plot_size/2, point_x + plot_size/2, point_y + plot_size, fill="yellow")

    def draw_vehicle_position(self, lat, lon):
        self.test_canvas.delete("vehicle_positin")
        point_x, point_y = self._mapCoordinator.get_point_on_image(lat, lon)
        plot_size = 6
        self.test_canvas.create_oval(point_x - plot_size/2, point_y - plot_size/2, point_x + plot_size/2, point_y + plot_size, fill="blue", tag="vehicle_positin")

    def continuous_task(self):
        while True:
            lat, lon = self.droneController.get_drone_position()
            self.draw_vehicle_position(lat, lon)
            self.get_obstacle()
            time.sleep(1)

    def get_obstacle(self):
        stock = self.droneController.get_draw_obstacle_stock()
        if not stock == []:
            for position in stock:
                self.draw_obstacle(position['lat'], position['lon'])

root = tkinter.Tk()
windowController = WindowController(master=root)
windowController.mainloop()