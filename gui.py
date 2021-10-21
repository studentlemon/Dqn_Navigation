"""
Created on Mon May 7 20:13:21 2018

@author: nader
"""

import numpy as np
from random import random, randint
import matplotlib.pyplot as plt
#import time

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.button import Button
from kivy.graphics import Color, Ellipse, Line
from kivy.config import Config
from kivy.properties import NumericProperty, ReferenceListProperty, ObjectProperty
from kivy.vector import Vector
from kivy.clock import Clock

# Importing the AI
from ai import Dqn

# We use the code below to stop adding red cirles by right clicking
Config.set('input', 'mouse', 'mouse,multitouch_on_demand')

# we use last_x and last_y to keep track of last point we draw on canvas
last_x = 0
last_y = 0
n_points = 0
length = 0
last_action = 1
# Initialization of Brain with 8 sensors and 8 actions and 0.9 discount factor
brain = Dqn(4,9,0.9)
action2rotation = [0,1,2,3,4,5,6,7]
Action_set = [[-1,1],[0,1],[1,1],[-1,0],[0,0],[1,0],[-1,-1],[0,-1],[1,-1]]
last_reward = 0
scores = []
# map initialization
first_update = True
resetstate = True
goal_x = 500
goal_y = 500
last_distance = 0

def init():
    #sand is an array with size of screen pixel size
    global sand
    global goal_x
    global goal_y
    global first_update
    global resetstate
    #longueu,largeu
    #sand = np.zeros((length,width))
    # initializing sand array with 0
    sand=np.zeros((length,width))
    first_update = False



class Car(Widget):
    
    # initializing the angle of the car
    angle = NumericProperty(0)
    # initializing rotation of the car
    rotation = NumericProperty(0)
    # initializing speed in x-vector
    velocity_x = NumericProperty(0)
    # initializing speed in y-vector
    velocity_y = NumericProperty(0)
    # speed vector
    velocity = ReferenceListProperty(velocity_x, velocity_y)
    # initializing x of forward sensor
    sensor1_x = NumericProperty(0)
    # initializing y of forward sensor
    sensor1_y = NumericProperty(0)
    # forward sensor vector
    sensor1 = ReferenceListProperty(sensor1_x, sensor1_y)
    # initializing x of left sensor
    sensor2_x = NumericProperty(0)
    # initializing y of left sensor
    sensor2_y = NumericProperty(0)
    # left sensor vector
    sensor2 = ReferenceListProperty(sensor2_x, sensor2_y)

    # initializing x of right sensor
    sensor3_x = NumericProperty(0)
    # initializing y of right sensor
    sensor3_y = NumericProperty(0)
    # right sensor vector
    sensor3 = ReferenceListProperty(sensor3_x, sensor3_y)

    # initializing x of right sensor
    sensor4_x = NumericProperty(0)
    # initializing y of right sensor
    sensor4_y = NumericProperty(0)
    # right sensor vector
    sensor4 = ReferenceListProperty(sensor4_x, sensor4_y)

    # initializing x of right sensor
    sensor5_x = NumericProperty(0)
    # initializing y of right sensor
    sensor5_y = NumericProperty(0)
    # right sensor vector
    sensor5 = ReferenceListProperty(sensor5_x, sensor5_y)

    # initializing x of right sensor
    sensor6_x = NumericProperty(0)
    # initializing y of right sensor
    sensor6_y = NumericProperty(0)
    # right sensor vector
    sensor6 = ReferenceListProperty(sensor6_x, sensor6_y)

    # initializing x of right sensor
    sensor7_x = NumericProperty(0)
    # initializing y of right sensor
    sensor7_y = NumericProperty(0)
    # right sensor vector
    sensor7 = ReferenceListProperty(sensor7_x, sensor7_y)

    # initializing x of right sensor
    sensor8_x = NumericProperty(0)
    # initializing y of right sensor
    sensor8_y = NumericProperty(0)
    # right sensor vector
    sensor8 = ReferenceListProperty(sensor8_x, sensor8_y)

    # initializing signal recieved from f-sensor
    signal1 = NumericProperty(0)
    # initializing signal recieved from l-sensor
    signal2 = NumericProperty(0)
    # initializing signal recieved from r-sensor
    signal3 = NumericProperty(0)
    signal4 = NumericProperty(0)
    # initializing signal recieved from l-sensor
    signal5 = NumericProperty(0)
    # initializing signal recieved from r-sensor
    signal6 = NumericProperty(0)
    # initializing signal recieved from l-sensor
    signal7 = NumericProperty(0)
    # initializing signal recieved from r-sensor
    signal8 = NumericProperty(0)

    def move(self, action):
        # update position of car according to its last position and speed
        #action_temp = Action_set[action]
        self.action = Action_set[action]
        self.pos = Vector(self.action) + self.pos
        # updating position of sensors
        self.sensor1 = Vector([-1, 1])+ self.pos
        self.sensor2 = Vector([0, 1])+ self.pos
        self.sensor3 = Vector([1, 1]) + self.pos
        self.sensor4 = Vector([-1, 0])+ self.pos
        self.sensor5 = Vector([1, 0])+ self.pos
        self.sensor6 = Vector([-1, -1]) + self.pos
        self.sensor7 = Vector([0, -1]) + self.pos
        self.sensor8 = Vector([1, -1]) + self.pos

        # getting signals recieved from sensors => density of wall or sand aside it
        #self.signal1 = int(np.sum(sand[int(self.sensor1_x)-2:int(self.sensor1_x)+2, int(self.sensor1_y)-2:int(self.sensor1_y)+2]))/16.
        #self.signal2 = int(np.sum(sand[int(self.sensor2_x)-2:int(self.sensor2_x)+2, int(self.sensor2_y)-2:int(self.sensor2_y)+2]))/16.
        #self.signal3 = int(np.sum(sand[int(self.sensor3_x)-2:int(self.sensor3_x)+2, int(self.sensor3_y)-2:int(self.sensor3_y)+2]))/16.

        # checking if any sensor has detected full density of wall or sand
        if self.sensor1_x>length-5 or self.sensor1_x<5 or self.sensor1_y>width-5 or self.sensor1_y<5:
            self.signal1 = 1.
        if self.sensor2_x>length-5 or self.sensor2_x<5 or self.sensor2_y>width-5 or self.sensor2_y<5:
            self.signal2 = 1.
        if self.sensor3_x>length-5 or self.sensor3_x<5 or self.sensor3_y>width-5 or self.sensor3_y<5:
            self.signal3 = 1.
        if self.sensor4_x>length-5 or self.sensor4_x<5 or self.sensor4_y>width-5 or self.sensor4_y<5:
            self.signal4 = 1.
        if self.sensor5_x>length-5 or self.sensor5_x<5 or self.sensor5_y>width-5 or self.sensor5_y<5:
            self.signal5 = 1.
        if self.sensor6_x>length-5 or self.sensor6_x<5 or self.sensor6_y>width-5 or self.sensor6_y<5:
            self.signal6 = 1.
        if self.sensor7_x>length-5 or self.sensor7_x<5 or self.sensor7_y>width-5 or self.sensor7_y<5:
            self.signal7 = 1.
        if self.sensor8_x>length-5 or self.sensor8_x<5 or self.sensor8_y>width-5 or self.sensor8_y<5:
            self.signal8 = 1.

# sensors
class Ball1(Widget):
    pass
class Ball2(Widget):
    pass
class Ball3(Widget):
    pass
class Ball4(Widget):
    pass
class Ball5(Widget):
    pass
class Ball6(Widget):
    pass
class Ball7(Widget):
    pass
class Ball8(Widget):
    pass

# the main class
class Game(Widget):

    # getting objects from kivy file
    car = ObjectProperty(None)
    ball1 = ObjectProperty(None)
    ball2 = ObjectProperty(None)
    ball3 = ObjectProperty(None)
    ball4 = ObjectProperty(None)
    ball5 = ObjectProperty(None)
    ball6 = ObjectProperty(None)
    ball7 = ObjectProperty(None)
    ball8 = ObjectProperty(None)

    def serve_car(self):
        # car starts in the center of screen going right with speed of 6
        self.car.center = self.center

    def update(self, dt):# update function for updating everything in new

        global brain
        global last_reward
        global scores
        global last_distance
        global goal_x
        global goal_y
        global length
        global width
        global last_action
        global resetstate
        global first_update
        length = self.width
        width = self.height

        # to initialize map only once
        if first_update:
            init()

        dx = self.car.x - goal_x
        dy = self.car.y - goal_y
        ddx = Action_set[last_action][0]
        ddy = Action_set[last_action][1]

        # our input state according to sensors and orientation
        last_signal = [dx,dy,ddx,ddy]
        # getting action from ai
        action = brain.update(last_reward, last_signal)
        # appending new score to **score window**
        scores.append(brain.score())
        #act_action = Action_set[action]
        # moving car according to rotation
        self.car.move(action)
        # setting new distance
        # updating sensors new position
        self.ball1.pos = self.car.sensor1
        self.ball2.pos = self.car.sensor2
        self.ball3.pos = self.car.sensor3
        self.ball4.pos = self.car.sensor4
        self.ball5.pos = self.car.sensor5
        self.ball6.pos = self.car.sensor6
        self.ball7.pos = self.car.sensor7
        self.ball8.pos = self.car.sensor8
        distance = np.sqrt((self.car.x - goal_x)**2 + (self.car.y - goal_y)**2)

        if 0:# changing speed when going into walls or sand
            # getting a very bad reward => -1
            last_reward = -10.0
        else: 
            # normal move . reward => -0.2
            last_reward = -np.sqrt(ddx*ddx+ddy*ddy)

            if ddx != 0 and (ddx + Action_set[action][0] == 0) and resetstate != True:
                last_reward = -100.0
                print("distance: ", distance)
                print("action: ", action)
                #print("loop x")

            if ddy != 0 and (ddy + Action_set[action][1] == 0) and resetstate != True:
                last_reward = -100.0
                print("distance: ", distance)
                print("action: ", action)
                #print("loop y")

            if action == 4 and resetstate != True:
                last_reward = -10.0

            if distance < last_distance:
                # if in correct direction get a little positive reward
                last_reward += (last_distance - distance)*1.2
            elif resetstate != True:
                last_reward += (last_distance - distance)*1.2

        if self.car.x < 0:# if car goes to left border of screen
            self.car.x = 0
            last_reward = -100
        if self.car.x > self.width:# if car goes to right side border of screen
            self.car.x = self.width
            last_reward = -100
        if self.car.y < 0:# if car goes to top border of screen
            self.car.y = 0
            last_reward = -100
        if self.car.y > self.height:# if car goes to bottom border of screen
            self.car.y = self.height
            last_reward = -100

        if distance < 10:
            last_reward += 1000
            self.car.x = int(random()*length*0.8)
            self.car.y = int(random()*width*0.8)
            goal_x = int(random()*length*0.8)
            goal_y = int(random()*width*0.8)
            print("goal x, goal y: ",goal_x, goal_y)
            resetstate = True
            first_update = False
            last_action = 4
            last_distance = np.sqrt((self.car.x - goal_x)**2 + (self.car.y - goal_y)**2)
        else:
            resetstate = False
            last_distance = distance
            last_action = action

class MyPaintWidget(Widget):

    def on_touch_down(self, touch):
        global length, n_points, last_x, last_y
        with self.canvas:
            Color(0.6,0.5,0.1)
            d = 10.
            touch.ud['line'] = Line(points = (touch.x, touch.y), width = 10)
            last_x = int(touch.x)
            last_y = int(touch.y)
            n_points = 0
            length = 0
            sand[int(touch.x),int(touch.y)] = 1

    def on_touch_move(self, touch):
        global length, n_points, last_x, last_y
        if touch.button == 'left':
            touch.ud['line'].points += [touch.x, touch.y]
            x = int(touch.x)
            y = int(touch.y)
            length += np.sqrt(max((x - last_x)**2 + (y - last_y)**2, 2))
            n_points += 1.
            density = n_points/(length)
            touch.ud['line'].width = int(20 * density + 1)
            sand[int(touch.x) - 10 : int(touch.x) + 10, int(touch.y) - 10 : int(touch.y) + 10] = 1
            last_x = x
            last_y = y


class CarApp(App):

    def build(self):
        parent = Game()
        parent.serve_car()
        Clock.schedule_interval(parent.update, 1.0/60.0)
        self.painter = MyPaintWidget()
        clearbtn = Button(text = 'clear')
        savebtn = Button(text = 'save', pos = (parent.width, 0))
        loadbtn = Button(text = 'load', pos = (2 * parent.width, 0))
        clearbtn.bind(on_release = self.clear_canvas)
        savebtn.bind(on_release = self.save)
        loadbtn.bind(on_release = self.load)
        parent.add_widget(self.painter)
        parent.add_widget(clearbtn)
        parent.add_widget(savebtn)
        parent.add_widget(loadbtn)
        return parent

    def clear_canvas(self, obj):
        global sand
        self.painter.canvas.clear()
        sand = np.zeros((length,width))

    def save(self, obj):
        print("saving brain...")
        brain.save()
        plt.plot(scores)
        plt.show()

    def load(self, obj):
        print("loading last saved brain...")
        brain.load()

if __name__ == '__main__':
    CarApp().run()
