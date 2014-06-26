# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose 
from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator import HRP2ModelBaseFlexEstimator 
from dynamic_graph.sot.application.stabilizer.scenarii.hand_compensater import HandCompensater
from dynamic_graph.sot.core.matrix_util import matrixToTuple

appli = HandCompensater(robot, True, True)
appli.withTraces()

est1 = HRP2ModelBaseFlexEstimator(robot)

meas = est1.signal('measurement')
inputs = est1.signal('input')
contactNbr = est1.signal('contactNbr')

# Inutilise mais l'entite attend les signaux
contactNbr.value = 2

flex=est1.signal('flexMatrixInverse')
flexdot = est1.signal('flexInverseVelocityVector')

appli.robot.addTrace( est1.name,'flexibility' )
appli.robot.addTrace( est1.name,'flexInverseVelocityVector' )
appli.robot.addTrace( est1.name,'flexMatrixInverse' )
appli.robot.addTrace( est1.name,'input')
appli.robot.addTrace( est1.name,'measurement')
appli.robot.addTrace( est1.name,'simulatedSensors' )
appli.robot.addTrace( est1.name,'inovation' )

appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name,  'gyrometer')

appli.startTracer()

plug(flex,appli.ccMc)
plug(flexdot,appli.ccVc)

est1.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-2,)*6)))

appli.gains['trunk'].setConstant(2)

appli.nextStep()
