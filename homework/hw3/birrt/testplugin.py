#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/birrt')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    BiRRT = RaveCreateModule(env,'BiRRT')
    print BiRRT.SendCommand('help')
finally:
    RaveDestroy()
