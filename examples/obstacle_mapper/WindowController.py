import tkinter
import tkinter.ttk
from PIL import Image, ImageTk

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

        home_lat = self.droneController.vehicle.location.global_frame.lat
        home_lon = self.droneController.vehicle.location.global_frame.lon
        self._mapCoordinator = MapCoodinator(home_lat, home_lon, 19)
        self.create_widgets()

        point_x, point_y = self._mapCoordinator.get_point_on_image(home_lat, home_lon)
        self.draw_home_point(point_x, point_y)
        print("drown points are {0}, {1}".format(point_x, point_y))

        self.after(100, self.continuous_task)
        # self.droneController.arm_and_takeoff(20)


    def create_widgets(self):
        self.button_go_to_click_point = tkinter.ttk.Button(self, text=u'go to click point', command=self.go_to_click_point)

        self.label_description = tkinter.ttk.Label(self, text='Mouse position')
        self.label_description.grid(row=0, column=1)
        self.button_go_to_click_point.grid(row=1, column=1, columnspan=2)

        self.test_canvas = tkinter.Canvas(self, bg='lightblue', width=256*3, height=256*3, highlightthickness=0)
        self.test_canvas.grid(row=0, column=0, rowspan=7)

        self.img = ImageTk.PhotoImage(self._mapCoordinator.generateImage())
        self.test_canvas.create_image(0, 0, image=self.img, anchor=tkinter.NW)
        self.test_canvas.bind('<ButtonPress-1>', self.start_pickup)

    def go_to_click_point(self):
        if self.droneController.click_point_lat != None and self.droneController.click_point_lon != None:
            print("let's go")
            self.droneController.go_to_click_point()

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

    def continuous_task(self):
        vehicle_state = self.droneController.get_vehicle_state()
        if not vehicle_state['obstacle_lat'] == None:
            self.draw_obstacle(vehicle_state['obstacle_lat'], vehicle_state['obstacle_lon'])
        print("get it")
        self.after(1000, self.continuous_task)

root = tkinter.Tk()
windowController = WindowController(master=root)
windowController.mainloop()