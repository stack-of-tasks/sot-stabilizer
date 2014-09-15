# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose, MatrixHomoToPoseRollPitchYaw
from dynamic_graph.sot.application.state_observation.initializations.hrp2_flexibility_estimator import HRP2FlexibilityEstimator 
from dynamic_graph.sot.application.stabilizer.scenarii.hand_compensater import HandCompensater
from dynamic_graph.sot.core.matrix_util import matrixToTuple
import time

appli = HandCompensater(robot, True, True)
appli.withTraces()

est = HRP2FlexibilityEstimator(robot)

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

flex=est.signal('flexMatrixInverse')
flexdot = est.signal('flexInverseVelocityVector')

appli.robot.addTrace( est.name,'flexibility' )
appli.robot.addTrace( est.name,'flexInverseVelocityVector' )
appli.robot.addTrace( est.name,'flexMatrixInverse' )
appli.robot.addTrace( est.name,'input')
appli.robot.addTrace( est.name,'measurement')
appli.robot.addTrace( est.name,'simulatedSensors' )
appli.robot.addTrace( est.name,'predictedSensors' )

appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name,  'gyrometer')

appli.robot.addTrace( est.name,'inovation' )
# appli.robot.addTrace( est.name,'prediction' )

# Position main droite
RHand = MatrixHomoToPoseRollPitchYaw('HomotoPoseRH')
plug(robot.dynamic.signal('right-wrist'),RHand.sin)
appli.robot.addTrace( RHand.name,  'sout')

# Position main gauche
LHand = MatrixHomoToPoseRollPitchYaw('HomotoPoseLH')
plug(robot.dynamic.signal('left-wrist'),LHand.sin)
appli.robot.addTrace( LHand.name,  'sout')

# Position ref main droite
RHandref = MatrixHomoToPoseRollPitchYaw('HomotoPoseRHR')
plug(appli.features['right-wrist'].reference,RHandref.sin)
appli.robot.addTrace( RHandref.name,  'sout')

# Position ref main gauche
LHandref = MatrixHomoToPoseRollPitchYaw('HomotoPoseLHR')
plug(appli.features['left-wrist'].reference,LHandref.sin)
appli.robot.addTrace( LHandref.name,  'sout')

# Position ref main droite
appli.robot.addTrace( 'tranformation_right',  'gV0')

# Position ref main droite
appli.robot.addTrace( 'tranformation_left' ,  'gV0')


appli.startTracer()

plug(flex,appli.ccMc)
plug(flexdot,appli.ccVc)

est.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-2,)*6)))
est.setVirtualMeasurementsCovariance(1e-4)

appli.gains['trunk'].setConstant(2)

appli.nextStep()

