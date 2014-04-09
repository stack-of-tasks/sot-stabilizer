# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
import matplotlib.pyplot as pl
import dynamic_graph as dg
import dynamic_graph.signal_base as dgsb

from math import sin

from dynamic_graph.sot.application.stabilizer.compensater import *

appli = HandCompensater(robot)
appli.withTraces()

if 'usingRobotViewer' in locals() and usingRobotViewer:    refreshList.append(lambda: appli.updateDisplay());    appli.initDisplay(); go()

# --- SHORTCUTS
rm           = appli.rm
push         = appli.push
dyn          = appli.robot.dynamic
sot          = appli.sot
taskCom      = appli.taskCom
taskChest    = appli.taskChest
taskPosture  = appli.taskPosture
taskRH       = appli.taskRH
tr           = appli.robot.tracer

t = optionalparentheses(appli.dumpTracer)
a = appli
s = optionalparentheses(appli.nextStep)

#### Flexibility Estimator ##
est= sotso.DGIMUFlexibilityEstimation('flextimator')
est.setSamplingPeriod(0.005)

meas = est.signal('measurement')
inputs = est.signal('input')
contactNbr = est.signal('contactNbr')
contactNbr.value = 1

contact1 = est.signal('contact1')
contact1.value = (0,0,0);

flex=est.signal('flexMatrixInverse')
flexdot = est.signal('flexInverseVelocityVector')

plug(flex,appli.ccMc)
plug(flexdot,appli.ccVc)

#appli.robot.addTrace( est.name,'flexInverseVelocityVector' )
#appli.robot.addTrace( est.name,'flexibility'  )
#appli.robot.addTrace( est.name , 'simulatedSensors' )

meas.value = (0.0 , 0.0,  9.81 , 0.0 , 0.0 , 0.0)
inputs.value = (0.0, 0.0, 1.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

#stop()
