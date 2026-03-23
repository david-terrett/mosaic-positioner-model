import ezdxf
import matplotlib.pyplot as plt
from math import cos
from math import radians
from math import sin
import os

class inspect_drawing(object):

    def __init__(self, filename, ax, delay=0.0):
        try:
            paths = os.environ['PYTHONPATH'].split(os.pathsep)
        except KeyError:
            paths = []
        doc = None
        for path in paths:
            file = os.path.join(path, 'positioner_model', filename)
            if os.path.isfile(file):
                doc = ezdxf.readfile(file)
        self.msp = doc.modelspace()
        self.ax = ax
        self.delay = delay


    def item(self, i):
        e = self.msp[i]
        if e.dxftype() == 'LINE':
            print(i, 'LINE', e.dxf.start[0], e.dxf.end[0],
                  e.dxf.start[1], e.dxf.end[1])
            self.ax.plot([e.dxf.start[0], e.dxf.start[1]],
                         [e.dxf.end[0], e.dxf.end[1]],
                         color='black')
            if self.delay > 0.0:
                plt.pause(self.delay)

        elif e.dxftype() == 'POLYLINE':
            print(i, 'POLYLINE')
            x = []
            y = []
            for p in e.points():
                print(p[0], p[1])
                x.append(p[0])
                y.append(p[1])
            self.ax.plot(x, y, color='red')
            if self.delay > 0.0:
                plt.pause(self.delay)
        elif e.dxftype() == 'ARC':
            print(i, 'ARC', e.dxf.center[0], e.dxf.center[1], e.dxf.radius)
            x = []
            y = []
            for a in e.angles(10):
                x.append(e.dxf.center[0] + e.dxf.radius * cos(radians(a)))
                y.append(e.dxf.center[1] + e.dxf.radius * sin(radians(a)))
            self.ax.plot(x, y, color='blue')
            if self.delay > 0.0:
                plt.pause(self.delay)
