#!/usr/bin/env python
# -*- coding: utf-8 -*-

# commands:Array
commands_extra = [
    # manually added
    ['set %m.var to %s', ' ', 9, 'setVar:to:'],
    ['change %m.var by %n', ' ', 9, 'changeVar:by:'],
]

# extension:Array
extras = []

# robotics:ScratchExtension
extras += [
    # add extensions code if not auto-generated
    [' ', 'stop robot-drone', 'Scratch2JdeRobot/stop'],
    [' ', 'move robot %m.robotDirections', 'Scratch2JdeRobot/robot/move', 'forward'],
    [' ', 'move drone %m.droneDirections', 'Scratch2JdeRobot/drone/move', 'forward'],
    [' ', 'move drone %m.direction speed %n', 'Scratch2JdeRobot/drone/move/speed', 'forward', 1],
    [' ', 'move robot %m.direction speed %n', 'Scratch2JdeRobot/robot/move/speed', 'forward', 1],
    [' ', 'turn drone %m.turnDirections speed %n', 'Scratch2JdeRobot/turn/speed', 'left', 1],
    [' ', 'turn robot %m.turnDirections speed %n', 'Scratch2JdeRobot/turn/speed', 'left', 1],
    [' ', 'take off drone', 'Scratch2JdeRobot/drone/takeoff'],
    [' ', 'land drone', 'Scratch2JdeRobot/drone/land'],
    ['r', 'frontal laser distance', 'Scratch2JdeRobot/laser/frontal'],
    ['r', 'color detection %m.color', 'Scratch2JdeRobot/camera/all', 'red'],
    ['r', 'get pose3D', 'Scratch2JdeRobot/camera/pose3D'],
]
