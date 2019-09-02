""" Tkinter demo gui for bezier fitting algorithm

     (c) Volker Poplawski 2014
"""
from __future__ import print_function
from numpy import array
from line_fitting_helpers.bezier import *
from line_fitting_helpers.fitCurves import *
from line_fitting_helpers.bspline_path import *
from Tkinter import *
import tkMessageBox as mb
import math


# center of bounding box
def cntr(x1, y1, x2, y2):
    return x1+(x2-x1)/2, y1+(y2-y1)/2


# tkinter Canvas plus some addons
class MyCanvas(Canvas):

    def __init__(self,root):
        Canvas.__init__(self, root, bg='white', width=1000, height=496)
        self.routes = []
        self.sampling_rates = 333.0

    def create_polyline(self, points, **kwargs):
        self.routes = points
        #print("xuyao:",points)
        for p1, p2 in zip(points, points[1:]):
            self.create_line(p1, p2, kwargs)


    def create_bezier(self, b, tag):
        self.create_polyline([bezier.q(b, t/float(self.sampling_rates)).tolist() for t in xrange(0, int(self.sampling_rates+1))], tag=tag, fill='blue', width='2') # there are better ways to draw a bezier
        #self.create_line(b[0].tolist(), b[1].tolist(), tag=tag)
        #self.create_point(b[1][0], b[1][1], 2, fill='black', tag=tag)
        #self.create_line(b[3].tolist(), b[2].tolist(), tag=tag)
        #self.create_point(b[2][0], b[2][1], 2, fill='black', tag=tag)


    def create_point(self, x, y, r, **kwargs):
        return self.create_oval(x-r, y-r, x+r, y+r, kwargs)


    def pos(self, idOrTag):
        return cntr(*self.coords(idOrTag))


    def itemsAtPos(self, x, y, tag):
        return [item for item in self.find_overlapping(x, y, x, y) if tag in self.gettags(item)]


class MainObject:
    def run(self, config_name="my_routes.csv"):
        
        self.root = Tk()
        

        self.canvas = MyCanvas(self.root)
        self.canvas.pack(side="bottom")
       
        self.map = StringVar(self.root)
        self.map.set("Town_control") #default value
        self.optionMenue1 = OptionMenu(self.root, self.map,"Town_control","Town01","Town02", \
            "Town03","Town04","Town05","Town06","Town07")
        self.optionMenue1.pack(side = "left")

        self.vehicle = StringVar(self.root)
        self.vehicle.set("vehicle.lincoln.mkz201") #default value
        self.optionMenue2 = OptionMenu(self.root, self.vehicle,"vehicle.lincoln.mkz201","vehicle.bmw.isetta","vehicle.tesla.model3")
        self.optionMenue2.pack(side = "left")


        self.mode = StringVar(self.root)
        self.mode.set("no_obstacle") #default value
        self.optionMenue3 = OptionMenu(self.root, self.mode,"no_obstacle","with_obstacle")
        self.optionMenue3.pack(side = "left")
        
        self.function = StringVar(self.root)
        self.function.set("B_spline")
        self.optionMenue4 = OptionMenu(self.root, self.function, "B_spline", "Bezier")
        self.optionMenue4.pack(side = "left")

        frame = Frame(self.root, relief=SUNKEN, borderwidth=1)
        frame.pack(side=LEFT)
        label = Label(frame, text='Max Error')
        label.pack(side = "left")
        self.spinbox = Spinbox(frame, width=8, from_=0.0, to=1000000.0, command=self.onSpinBoxValueChange)
        self.spinbox.insert(0, 10.0)
        self.spinbox.pack()

        sampling_interval = Frame(self.root, relief=SUNKEN, borderwidth=1)
        sampling_interval.pack(side=LEFT)
        label = Label(sampling_interval, text='set_sampling_interval')
        label.pack(side = "left")
        self.spinbox_sam = Spinbox(sampling_interval, width=8, from_=0.0, to=100.0,increment=0.2, command=self.onSpinBoxsamValueChange)
        self.spinbox_sam.delete(0)
        self.spinbox_sam.delete(1)
       
        self.spinbox_sam.insert(0, 2)
        self.spinbox_sam.insert(END, 0)
        self.spinbox_sam.pack()

        self.config_file = config_name
        self.points = []
        self.vehicle_points = []
        self.sampling_rates = self.canvas.sampling_rates
        self.draggingPoint = None
        self.dragging_vehicle = None
        self.total_distance = 0
        self.ifsave = 0 
        self.vehicle_mode = 0
        self.optionMenue2.bind('<ButtonPress-1>', self.add_vehicle)
     
        self.optionMenue4.bind('<ButtonPress-1>',self.change_function)
        self.canvas.bind('<ButtonPress-1>', self.onButton1Press)
        self.canvas.bind('<ButtonPress-2>', self.onButton2Press)
        self.canvas.bind('<B1-Motion>', self.onMouseMove)
        self.canvas.bind('<ButtonRelease-1>', self.onButton1Release)
        self.button = Button(self.root,text='save',font=('KaiTi',12,'bold'),bg='green',bd=1,width=5,command=self.record_routes)
        #self.button.insert(0,10.0)
        self.button.pack(side="right")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def on_closing(self):
        if self.ifsave == 0:
            if mb.askokcancel("WARNING", "current routes are not saved, do you want to quit? it will load your pre_configuration to your scenario"):
                self.root.destroy()
            
    def record_routes(self):
         #print("self.canvas.routes=", self.canvas.routes)
         #print(len(self.canvas.routes))
         map_ = self.map.get()
         vehicle_ = self.vehicle.get()
         mode_ = self.mode.get()
         #print("mao:", map_)
         #print("vehicle:", vehicle_)
         #print("mode:", mode_)
         with open(self.config_file, "w") as my_file:
            lines = []
            init_text = "x,y,z,yaw\n"
            my_file.write(init_text)
            for i in range(len(self.canvas.routes)):
                x = 397 - self.canvas.routes[i][1] 
                y = -290 + self.canvas.routes[i][0]
                z = 0
                if i < len(self.canvas.routes)-1:
                    delta_x = -self.canvas.routes[i+1][0] + self.canvas.routes[i][0]
                    delta_y = self.canvas.routes[i+1][1] - self.canvas.routes[i][1]
                    yaw = (math.atan2(delta_x, delta_y))*180/math.pi + 180
                else:
                    delta_x = -self.canvas.routes[i][0] + self.canvas.routes[i-1][0]
                    delta_y = self.canvas.routes[i][1] - self.canvas.routes[i-1][1]
                    yaw = (math.atan2(delta_x, delta_y))*180/math.pi + 180
                text = "{},{},{},{}\n".format(
                    str(x), str(y), str(z),  str(yaw) 
                )
                lines.append(text)
            my_file.writelines(lines)
            end_text = "{},{},{}".format(
                 str(map_),str(vehicle_),str(mode_)
                )
            my_file.write(end_text)
            self.ifsave = 1
            print("save successfully into {}!,please close design window".format(self.config_file))
            if mb.askokcancel("QUIT", "routes are saved, do you want to quit to load you routes to scenario?"):
                self.root.destroy()

    def add_vehicle(self,event):
        if mb.askokcancel("vehicle", "press your mouse to put the vehicle"):
            self.vehicle_mode = 1

    
        

    def change_function(self, event):
        self.redraw()
        
        print("change your function to {}".format(self.function.get()))
    def onButton1Press(self, event):
        items = self.canvas.itemsAtPos(event.x, event.y, 'point')
        items_vehicle = self.canvas.itemsAtPos(event.x, event.y, 'line')
        items_rear = self.canvas.itemsAtPos(event.x, event.y, 'rear')
        #print("x:",event.x)
        #print("y:",event.yif self.vehicle_mode = 1:
        if self.vehicle_mode == 2:
            if items:
                self.draggingPoint = items[0]
            elif items_vehicle:
                self.dragging_vehicle = items_vehicle[0]
                self.dragging_rear = self.vehicle_points[1]
            
            else:
                self.points.append(self.canvas.create_point(event.x, event.y, 4, fill='red', tag='point'))
                self.redraw()
        else:
            if self.vehicle_mode != 1:
                mb.showwarning("warning","please put your vehicle fisrt, just press the vehicle button")

        if self.vehicle_mode == 1:
                self.vehicle_points.append( self.canvas.create_line((event.x - 14,event.y - 14), (event.x , event.y ),fill ='green', width=8,tag='line') )
                self.vehicle_points.append( self.canvas.create_point(event.x, event.y, 6, fill='blue', tag='rear')  )
                self.vehicle_mode = 2


    def onButton2Press(self, event):
        self.canvas.delete(self.points.pop())
        self.redraw()


    def onMouseMove(self, event):
        if self.draggingPoint:
            self.canvas.coords(self.draggingPoint, event.x-4, event.y-4, event.x+4, event.y+4)
            self.redraw()
        if self.dragging_vehicle:
            self.canvas.coords(self.dragging_vehicle, event.x-14, event.y-14, event.x, event.y)
            self.canvas.coords(self.dragging_rear, event.x-6, event.y-6, event.x+6, event.y+6)

    def onButton1Release(self, event):
        self.draggingPoint = None


    def onSpinBoxValueChange(self):
        self.redraw()

    def onSpinBoxsamValueChange(self):
        #print(self.spinbox_sam.get())
        sampling_rates = float(self.spinbox_sam.get())
        self.canvas.sampling_rates = int(self.total_distance/sampling_rates)
        self.redraw()

    def redraw(self):
        #set the ifsave
        self.ifsave = 0
        # redraw polyline
        self.canvas.delete('polyline')
        self.canvas.create_polyline([self.canvas.pos(pId) for pId in self.points], fill='grey', tag='polyline')
        self.canvas.tag_lower('polyline')

        # redraw bezier
        if len(self.points) < 2:
            return

        self.canvas.delete('bezier')
        self.canvas.delete('B_spline')
        points = array([self.canvas.pos(p) for p in self.points])
        #compute the total distance here
        self.total_distance = 0
        for i in range(len(points)-1):
            delta_y = points[i+1][1] - points[i][1]
            delta_x = points[i+1][0] - points[i][0]
            self.total_distance += math.sqrt(math.pow(delta_x, 2) + math.pow(delta_y, 2))
        #print("total_distance", self.total_distance)
        #print(points) is the points you draw 
        sampling_rates = float(self.spinbox_sam.get())
        self.canvas.sampling_rates = int(self.total_distance/sampling_rates)

        print("you sample {} points:".format(self.canvas.sampling_rates))
        '''
        add your own fit algorithem here
        '''
        if self.function.get() == "Bezier":
            beziers = fitCurve(points, float(self.spinbox.get())**2)
            for bezier in beziers:
                self.canvas.create_bezier(bezier, tag='bezier')
        elif self.function.get() == "B_spline":
            B_splines = bspline_planning(points, self.canvas.sampling_rates)
            self.canvas.create_polyline(B_splines, tag='B_spline', fill='blue', width='2')
if __name__ == '__main__':
    o = MainObject()
    import sys
    if len(sys.argv) > 1:
        print(sys.argv[1])
        o.run(sys.argv[1])
    else:
        o.run()


