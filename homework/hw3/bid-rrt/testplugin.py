#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/bid-rrt')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    bidirect-rrtconnect = RaveCreateModule(env,'bidirect-rrtconnect')
    print bidirect-rrtconnect.SendCommand('help')
finally:
    RaveDestroy()
