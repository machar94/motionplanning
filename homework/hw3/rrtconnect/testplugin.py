#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/rrtconnect')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    RRTConnect = RaveCreateModule(env,'RRTConnect')
    print RRTConnect.SendCommand('help')
finally:
    RaveDestroy()
