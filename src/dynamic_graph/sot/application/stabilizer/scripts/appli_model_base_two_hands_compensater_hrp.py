# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose 
from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator import HRP2ModelBaseFlexEstimator, PositionStateReconstructor 
from dynamic_graph.sot.application.stabilizer.scenarii.hand_compensater import HandCompensater
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.application.state_observation import InputReconstructor
import time

appli = HandCompensater(robot, True, True)
appli.withTraces()

est1 = HRP2ModelBaseFlexEstimator(robot)

meas = est1.signal('measurement')
inputs = est1.signal('input')
contactNbr = est1.signal('contactNbr')

# Definition des contacts
contactNbr.value = 2

rFootPos = MatrixHomoToPose('rFootFrame')
lFootPos = MatrixHomoToPose('lFootFrame')
plug(robot.frames['rightFootForceSensor'].position,rFootPos.sin)
plug(robot.frames['leftFootForceSensor'].position,lFootPos.sin)
est1.contacts = Stack_of_vector ('contacts')
plug(rFootPos.sout,est1.contacts.sin1)
plug(lFootPos.sout,est1.contacts.sin2)
est1.contacts.selec1 (0, 3)
est1.contacts.selec2 (0, 3)
plug(est1.contacts.sout,est1.inputVector.contactsPosition)

flexVect=est1.signal('flexibility')
flex=est1.signal('flexMatrixInverse')
flexdot = est1.signal('flexInverseVelocityVector')

appli.robot.addTrace( est1.name,'flexibility' )
appli.robot.addTrace( est1.name,'flexInverseVelocityVector' )
appli.robot.addTrace( est1.name,'flexMatrixInverse' )
appli.robot.addTrace( est1.name,'input')
appli.robot.addTrace( est1.name,'measurement')
appli.robot.addTrace( est1.name,'simulatedSensors' )
appli.robot.addTrace( est1.name,'inovation' )
appli.robot.addTrace( est1.name,'prediction' )
appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name,  'gyrometer')

deriv=PositionStateReconstructor('deriv')
deriv.inputFormat.value='000010'
deriv.outputFormat.value='001000'
deriv.setSampligPeriod(0.005)
plug(est1.flexThetaU,deriv.sin)


appli.robot.addTrace( est1.name,'flexThetaU' )
appli.robot.addTrace( est1.name,'flexOmega' )
appli.robot.addTrace( deriv.name,'sout')




appli.startTracer()

plug(flex,appli.ccMc)
plug(flexdot,appli.ccVc)

est1.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-6,)*6)))

appli.gains['trunk'].setConstant(2)

#appli.nextStep()

time.sleep(120)
appli.comRef.value=(0.033566999999999999, 0.001536, 0.80771000000000004)
time.sleep(120)
appli.comRef.value=(0.003566999999999999, 0.001536, 0.80771000000000004)
time.sleep(120)
appli.comRef.value=(-0.01, 0.001536, 0.80771000000000004)
time.sleep(120)
appli.comRef.value=(0.003566999999999999, 0.001536, 0.80771000000000004)
time.sleep(120)
appli.comRef.value=(0.003566999999999999, 0.031536, 0.80771000000000004)
time.sleep(120)
appli.comRef.value=(0.003566999999999999, 0.001536, 0.80771000000000004)
time.sleep(120)
appli.comRef.value=(0.003566999999999999, -0.031536, 0.80771000000000004)
time.sleep(120)
appli.comRef.value=(0.003566999999999999, 0.001536, 0.80771000000000004)