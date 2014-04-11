# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb

from math import sin
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier

from dynamic_graph.sot.application.state_observation.initializations.hrp2_flexibility_estimator import HRP2FlexibilityEstimator 
from dynamic_graph.sot.application.stabilizer.scenarii.hand_compensater import *


appli = HandCompensater(robot, True, True)
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

sensorStack = Stack_of_vector ('sensors')
plug(robot.device.accelerometer,sensorStack.sin1)
plug(robot.device.gyrometer,sensorStack.sin2)
sensorStack.selec1 (0, 3)
sensorStack.selec2 (0, 3)

plug(sensorStack.sout,meas);


inputPos = MatrixHomoToPoseUTheta('inputPosition')

plug(robot.frames['accelerometer'].position,inputPos.sin)


robot.dynamic.createJacobian('chestJ','chest')
imu = OpPointModifier('IMU_oppoint')
imu.setEndEffector(False)
imu.setTransformation(matrixToTuple(matrix(robot.frames['accelerometer'].position.value)*np.linalg.inv(matrix(robot.dynamic.chest.value))))

plug (robot.dynamic.chest,imu.positionIN)
plug (robot.dynamic.chestJ,imu.jacobianIN)

inputVel = Multiply_matrix_vector('inputVelocity')
plug(imu.jacobian,inputVel.sin1)
plug(robot.device.velocity,inputVel.sin2)

inputPosVel = Stack_of_vector ('inputPosVel')
plug(inputPos.sout,inputPosVel.sin1)
plug(inputVel.sout,inputPosVel.sin2)
inputPosVel.selec1 (0, 6)
inputPosVel.selec2 (0, 6)

inputVector = sotso.PositionStateReconstructor ('estimatorInput')
plug(inputPosVel.sout,inputVector.sin)
inputVector.inputFormat.value  = '001111'
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

appli.robot.addTrace( inputVel.name, 'sout')

appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name,  'gyrometer')

appli.robot.addTrace( hMrhrefVector.name,'sout')

appli.robot.addTrace( appli.tasks['right-wrist'].name,'error')

appli.robot.addTrace( appli.robot.dynamic.name,'chest')

appli.robot.addTrace( appli.solver.sot.name,'control')
appli.robot.addTrace( robot.device.name,'state')

appli.startTracer()

plug(flex,appli.ccMc)
plug(flexdot,appli.ccVc)

est.setMeasurementNoiseCovariance(matrixToTuple(diag((1e-2,)*6)))

inputVector.setFiniteDifferencesInterval(2)

appli.gains['trunk'].setConstant(2)

appli.nextStep()
