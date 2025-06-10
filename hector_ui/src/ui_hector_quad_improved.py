#!/usr/bin/env python3
#  -*- coding: utf-8 -*-
import math
import rospy
import threading
import tkinter as tk
from tkinter import ttk
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

root = tk.Tk(className="Drone Controller")
root.title("Drone Controller")

# Imagen de icono
iconImg = """iVBORw0KGgoAAAANSUhEUgAAADAAAAAwCAYAAABXAvmHAAAABmJLR0QA/wD/AP+gvaeTAAAE4klEQVRoge2YW2wUVRjHf+fMXqHGECVB5Vq8EJW
        LAsG0knBpJZCQlmJIKKWSWEuMMSwPvvggMfpiTGTRNECiwbRbYxq6pfGhGisRuQQKNOXyYDS0QA0gGMQIvezuzOfDttDt7nZnZxtRsv992DPf9f+d
        b86cmQN55JFHHnnk8aCjtLKmo3TT68f/i/lcBw4elUxGdZ/vA1HYsS1fUazG0o93Pp3J4IFAaWWNlFbWZJyN+5FvzHYDBEPhDmBx/EpOBKrWv5QTu3HOZ
        +cWWnxvqJY4p2YbWeVL24EDB4+sAbUXmDpaV7ajGIDW9486IZiEDPF+E5HadStfbkulHKMDqcnfB0xVSu1Np3Q5ifjnE8+Czrh8/pV4ab1afjiyeqjyaU
        6JjRN6lVK1ZcuLvk2ltFV207Fjfne/PC5aHlOWnqaRhShKBOaPB0MFZxDaLdRp0VavstTVqF9d2VBU1G/D1zkOHDy8BPSXwByHIX4Ga0v5iqUnnHLI+UZ
        ubj9caGh9wYmvaVmz15cs7c4lf06vEh990fpQ7+9/VDr1v3ztxsa6pqaCXDg4LiDYuH+V12v+gsgHTmMo+DAacf0abNy/ymkMRwUEG8PbEd0GTAF+ROR6
        tjHE4roIh4ApiG7b1dCyzQmXrAsIhsJvIHwCmCi2BaoqlgtsAXqzCNOrDfXa9s0Vy5SoAGCKkuDOhuaabPlktYg/bQjPtxTHAY8oqrdvqmjMNmEqBEMtV
        SD1wCDaWhKofPWsXd+sOiCKzwAfqJ3jRR4gULUuhBAEfFhqt4jYnljbBexsbC4RWArcjFoTdjghOhbc3th7wE1QRcGvwivt+tl+F1Ki3gRQsPud6lV30t
        mVP//1K0rRAkwYpfpLi5Q0n994KpXfWxs23A42hvcgvDuUq90OL1sd+Lj+u4nAGsCyROrS2a1d+M0ErdiTgjzAw5ZSDVtm7vOl8ze0UQcIsHooZ0bY6oB
        X9xVb4FNwLrB5/dVhedWi1lKt2AvMikss8PnHCjXH8vn7qye3Dl/3gNTWnyxvB3h7Y9mVXaHweYG5bnWnCPg+EzdbHRDkRQBRcijBOYH8PcQsk77IILcH
        Brg9MEBfZJCYaaYKPUtIetf/CUAp9YIdbrY6IDAv/q9PjyYw2nYwGiVixhJkpiX0WxE84sLrcifoFBSOvLaUOqVEEGSBHW72FrFSTyGClrE3q5hlEjFjG
        IbBogXzKJwxHUHouXSZU13niMRiGErjMoy0MbTQO3Qc8eT4FSDyKIAg7cFQ+K64c1eiWSQan/lFC+ZRvOxpAG5dg+fmPIMIdHR2ETFjSQUEQ+G7RyjC8D
        CeMxPs7gOP2DGyJJ68cMZ0UDDyYGf2rBkAmJZlM6WyldPuPuAH8N9xT9y6dW3fsLB6cWvC4ZOMoHzrKgmQoeKUSt5kA1UVd4V1TU0F0YjrbyDt43Yk7Hb
        AAJg0qX9wTCMdD3fh4qUkXXdPXGakKGAkbkyePDA0tDW5OX2Rje5AzDTpj0YwDIOF8+dSOHM6AN0XL3P6zFlM08Lv8eDSiWug/mSZYx6OjlXSBjMMPOIi
        EovR0dlFR2dXgt5juJLI54pcT6d7Rgu8Ljd+tweX1qihn0tr/B4PXrc7KYDA/fsmBqlNRcBlGPg9Xgp8Pgp8Pvweb8qZF+jW6NrcOOSRRx7/a/wDNeu3t
        iRlt6UAAAAASUVORK5CYII="""
img = tk.PhotoImage(data=iconImg)
root.tk.call('wm','iconphoto',root._w,img)

# UI variables
x_p = tk.StringVar()
y_p = tk.StringVar()
z_p = tk.StringVar()
z_o = tk.StringVar()

current_altitude = 0.0
desired_altitude = 0.0

valor_z = tk.StringVar()

# ROS Callbacks
def pose_callback(data):
    global current_altitude
    current_altitude = data.pose.pose.position.z
    x_p.set("{0:.2f}".format(data.pose.pose.position.x))
    y_p.set("{0:.2f}".format(data.pose.pose.position.y))
    z_p.set("{0:.2f}".format(data.pose.pose.position.z))

def rot_callback(data):
    def quaternion_to_yaw(data):
        x = data.pose.orientation.x
        y = data.pose.orientation.y
        z = data.pose.orientation.z
        w = data.pose.orientation.w
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return 2 * math.pi + yaw if yaw < 0 else yaw
    z_o.set("{0:.2f}".format(math.degrees(quaternion_to_yaw(data))))

# ROS init
rospy.init_node('HectorQ_GUI', anonymous=False)

# Publishers
takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Subscribers
rospy.Subscriber("/ground_truth/state", Odometry, pose_callback)
rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, rot_callback)

mainframe = ttk.Frame(root, padding="3 3 12 12")
mainframe.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
mainframe.columnconfigure(0, weight=1) 

ttk.Separator(mainframe, orient='horizontal').grid(row=8, column=1, columnspan=4, sticky="e", pady=10)
ttk.Separator(mainframe, orient='vertical').grid(row=3, column=2, rowspan=7, sticky="s")

ttk.Label(mainframe, text="Set Altitude (m)").grid(column=1, row=3)
ttk.Entry(mainframe, width=5, textvariable=valor_z).grid(column=1, row=4)

def usar_valor_z():
    global desired_altitude
    try:
        desired_altitude = float(valor_z.get())

    except ValueError:
        print("Valor inválido.")

ttk.Button(mainframe, text="Ok", command=usar_valor_z).grid(column=1, row=5)

ttk.Label(mainframe, text="Status").grid(column=3, row=3)
status_label = ttk.Label(mainframe, text="Drone armed")
status_label.grid(column=3, row=4)

def setText(text):
    status_label.config(text=text)

def enviar_velocidad(vx, vy, vz, vaz):
    vel = Twist()
    vel.linear.x = float(vx)
    vel.linear.y = float(vy)
    vel.linear.z = float(vz)
    vel.angular.z = float(vaz)
    vel.angular.x = 0.0
    vel.angular.y = 0.0
    vel_pub.publish(vel)

def hover():
    setText("Hovering")
    enviar_velocidad(0.0, 0.0, 0.0, 0.0)

def up_fun():
    def run():
        setText("Going up")
        takeoff_pub.publish(Empty())
        rospy.sleep(1.0)
        while not rospy.is_shutdown() and current_altitude < (desired_altitude - 0.15): #Take off until to reach the desired altitude
            enviar_velocidad(0.0, 0.0, 0.5, 0.0)
            rospy.sleep(0.1)
        hover()
    threading.Thread(target=run).start()

def down_fun():
    def run():
        setText("Going down")
        land_pub.publish(Empty())
        rospy.sleep(1.0)
        while not rospy.is_shutdown() and current_altitude > (desired_altitude + 0.15): #Land until to reach the ground (base height = 0.27m)
            enviar_velocidad(0.0, 0.0, -0.5, 0.0)
            rospy.sleep(0.1)
        hover()
    threading.Thread(target=run).start()

# Movement commands with status update
def forward_fun():
    setText("Going forward")
    enviar_velocidad(1.0, 0.0, 0.0, 0.0)

def backward_fun():
    setText("Going backward")
    enviar_velocidad(-1.0, 0.0, 0.0, 0.0)

def left_fun():
    setText("Going left")
    enviar_velocidad(0.0, 1.0, 0.0, 0.0)

def right_fun():
    setText("Going right")
    enviar_velocidad(0.0, -1.0, 0.0, 0.0)

def takeoff_fun():
    setText("Taking off")
    enviar_velocidad(0.0, 0.0, 1.0, 0.0)

def land_fun():
    setText("Landing")
    enviar_velocidad(0.0, 0.0, -1.0, 0.0)

def turn_left_fun():
    setText("Turning left")
    enviar_velocidad(0.0, 0.0, 0.0, 1.0)

def turn_right_fun():
    setText("Turning right")
    enviar_velocidad(0.0, 0.0, 0.0, -1.0)

# Labels for data
for i, (label, var) in enumerate(zip(["X (m)", "Y (m)", "Alt (m)", "Yaw (°)"], [x_p, y_p, z_p, z_o]), start=1):
    ttk.Label(mainframe, textvariable=var).grid(column=i, row=2)
    ttk.Label(mainframe, text=label).grid(column=i, row=1)

# Control buttons
buttons = [
    ("Up", up_fun, 1, 6),
    ("Down", down_fun, 1, 7),
    ("Hover", hover, 3, 6),
    ("Forward", forward_fun, 3, 5),
    ("Backward", backward_fun, 3, 7),
    ("Left", left_fun, 2, 6),
    ("Right", right_fun, 4, 6),
    ("Take off", takeoff_fun, 2, 7),
    ("Land", land_fun, 4, 7),
    ("Turn left", turn_left_fun, 2, 5),
    ("Turn right", turn_right_fun, 4, 5)
]



for text, cmd, col, row in buttons:
    ttk.Button(mainframe, text=text, command=cmd).grid(column=col, row=row)

for child in mainframe.winfo_children():
    child.grid_configure(padx=5, pady=5)

root.mainloop()

