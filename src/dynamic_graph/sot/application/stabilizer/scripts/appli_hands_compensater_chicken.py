# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
import dynamic_graph as dg
import dynamic_graph.signal_base as dgsb

from math import sin
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier
import dynamic_graph.sot.application.sot_state_observation as sotso

from dynamic_graph.sot.application.stabilizer.compensater_chicken import *

appli = HandCompensaterChicken(robot)
appli.withTraces()

#### Flexibility Estimator ##
est= sotso.DGIMUFlexibilityEstimation('flextimator')
est.setSamplingPeriod(0.005)

meas = est.signal('measurement')
inputs = est.signal('input')
contactNbr = est.signal('contactNbr')
contactNbr.value = 2

contact1 = est.signal('contact1')

contact2 = est.signal('contact2')

rFootPos = MatrixHomoToPose('rFootFrame')
lFootPos = MatrixHomoToPose('lFootFrame')

plug(robot.frames['rightFootForceSensor'].position,rFootPos.sin)
plug(robot.frames['leftFootForceSensor'].position,lFootPos.sin)

plug(rFootPos.sout,contact1)
plug(lFootPos.sout,contact2)

sensorStack = Stack_of_vector ('sv1')
plug(robot.device.accelerometer,sensorStack.sin1)
plug(robot.device.gyrometer,sensorStack.sin2)
sensorStack.selec1 (0, 3)
sensorStack.selec2 (0, 3)

plug(sensorStack.sout,meas);


inputPos = MatrixHomoToPoseUTheta('inputPosition')

plug(robot.frames['accelerometer'].position,inputPos.sin)
inputVector = sotso.PositionStateReconstructor ('estimatorInput')


robot.dynamic.createJacobian('jchest','chest')
imu = OpPointModifier('IMU_oppoint')
imu.setEndEffector(True)
imu.setTransformation(matrixToTuple(matrix(robot.frames['accelerometer'].position.value)*np.linalg.inv(matrix(robot.dynamic.chest.value))))

plug(inputPos.sout,inputVector.sin)
inputVector.inputFormat.value  = '000011'
inputVector.outputFormat.value = '011111'

plug(inputVector.sout,inputs)

flex=est.signal('flexMatrixInverse')
flexdot = est.signal('flexInverseVelocityVector')



rhMcc = Inverse_of_matrixHomo('hMcc')
plug(appli.robot.dynamic.signal('right-wrist'),rhMcc.sin)
hMrhref = Multiply_of_matrixHomo('hMhref')
plug(rhMcc.sout    ,hMrhref.sin1)
plug(appli.ccMrhref,hMrhref.sin2)
hMrhrefVector = MatrixHomoToPoseUTheta('hMhrefVector')
plug(hMrhref.sout,hMrhrefVector.sin)

appli.robot.addTrace( est.name,'flexibility' )
appli.robot.addTrace( est.name,'flexInverseVelocityVector' )
appli.robot.addTrace( est.name,'flexMatrixInverse' )
appli.robot.addTrace( est.name,'input')
appli.robot.addTrace( est.name,'measurement')
appli.robot.addTrace( est.name,'simulatedSensors' )

appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name,  'gyrometer')

appli.robot.addTrace( hMrhrefVector.name,'sout')

appli.robot.addTrace( appli.tasks['right-wrist'].name,'error')

appli.robot.addTrace( appli.robot.dynamic.name,'chest')

appli.startTracer()

plug(flex,appli.ccMc)
plug(flexdot,appli.ccVc)

est.setMeasurementNoiseCovariance(matrixToTuple(diag((1e-2,)*6)))

#appli.rm(appli.taskPosture)

appli.nextStep()
