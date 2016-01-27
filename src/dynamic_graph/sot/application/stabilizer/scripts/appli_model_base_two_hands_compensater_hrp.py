# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose, MatrixHomoToPoseUTheta,MatrixHomoToPoseRollPitchYaw, Selec_of_vector
from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator import HRP2ModelBaseFlexEstimator, PositionStateReconstructor 
from dynamic_graph.sot.application.stabilizer.scenarii.hand_compensater import HandCompensater
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.application.state_observation import InputReconstructor
from dynamic_graph.sot.dynamics.zmp_from_forces import ZmpFromForces
import time


# Initialisation de l'appli
appli = HandCompensater(robot, True, False)
appli.withTraces()

est1 = HRP2ModelBaseFlexEstimator(robot)

meas = est1.signal('measurement')
inputs = est1.signal('input')
contactNbr = est1.signal('contactNbr')

# Definition des contacts
contactNbr.value = 2
est1.setContactModel(1)
rFootPos = MatrixHomoToPoseUTheta('rFootFramePos')
lFootPos = MatrixHomoToPoseUTheta('lFootFramePos')
plug(robot.frames['rightFootForceSensor'].position,rFootPos.sin)
plug(robot.frames['leftFootForceSensor'].position,lFootPos.sin)

est1.contacts = Stack_of_vector ('contacts')
plug(rFootPos.sout,est1.contacts.sin1)
plug(lFootPos.sout,est1.contacts.sin2)
est1.contacts.selec1 (0, 6)
est1.contacts.selec2 (0, 6)
plug(est1.contacts.sout,est1.inputVector.contactsPosition)


# Simulation: Stifness and damping
kfe=40000
kfv=600
kte=600
ktv=60
est1.setKfe(matrixToTuple(np.diag((kfe,kfe,kfe))))
est1.setKfv(matrixToTuple(np.diag((kfv,kfv,kfv))))
est1.setKte(matrixToTuple(np.diag((kte,kte,kte))))
est1.setKtv(matrixToTuple(np.diag((ktv,ktv,ktv))))

flexVect=est1.signal('flexibility')
flex=est1.signal('flexMatrixInverse')
flexdot = est1.signal('flexInverseVelocityVector')

appli.robot.addTrace( est1.name,'flexibility' )
appli.robot.addTrace( est1.name,'input' )
appli.robot.addTrace( est1.name,'flexInverseVelocityVector' )
appli.robot.addTrace( est1.name,'flexMatrixInverse' )
appli.robot.addTrace( est1.name,'input')
appli.robot.addTrace( est1.name,'measurement')
appli.robot.addTrace( est1.name,'simulatedSensors' )
appli.robot.addTrace( est1.name,'predictedSensors' )
appli.robot.addTrace( est1.name,'inovation' )
appli.robot.addTrace( est1.name,'prediction' )
appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( est1.name,  'forcesAndMoments')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name,  'gyrometer')

appli.robot.addTrace( est1.name,  'flexibilityComputationTime')

# Position main droite
RHand = MatrixHomoToPoseRollPitchYaw('HomotoPoseRH')
plug(robot.dynamic.signal('right-wrist'),RHand.sin)
appli.robot.addTrace( RHand.name,  'sout')

# Positions mains gauche et droite
LHand = MatrixHomoToPoseRollPitchYaw('HomotoPoseLH')
plug(robot.dynamic.signal('left-wrist'),LHand.sin)
appli.robot.addTrace( LHand.name,  'sout')
RHandref = MatrixHomoToPoseRollPitchYaw('HomotoPoseRHR')
plug(appli.features['right-wrist'].reference,RHandref.sin)
appli.robot.addTrace( RHandref.name,  'sout')

# Positions references mains gauche et droite
LHandref = MatrixHomoToPoseRollPitchYaw('HomotoPoseLHR')
plug(appli.features['left-wrist'].reference,LHandref.sin)
appli.robot.addTrace( LHandref.name,  'sout')
appli.robot.addTrace( 'tranformation_right',  'gV0')

# Position ref main gauche
appli.robot.addTrace( 'symvel' ,  'sout')

deriv=PositionStateReconstructor('deriv')
deriv.inputFormat.value='000010'
deriv.outputFormat.value='001000'
deriv.setSampligPeriod(0.005)
plug(est1.flexThetaU,deriv.sin)

integr=PositionStateReconstructor('integr')
integr.inputFormat.value='000100'
integr.outputFormat.value='000001'
integr.setSampligPeriod(0.005)
plug(est1.flexibility,integr.sin)

appli.robot.addTrace( est1.name,'flexThetaU' )
appli.robot.addTrace( est1.name,'flexOmega' )
appli.robot.addTrace( deriv.name,'sout')
appli.robot.addTrace( integr.name,'sout')

zmp = ZmpFromForces('zmp')
plug (robot.device.forceRLEG , zmp.force_0)
plug (robot.device.forceLLEG, zmp.force_1)
plug (robot.frames['rightFootForceSensor'].position , zmp.sensorPosition_0)
plug (robot.frames['leftFootForceSensor'].position, zmp.sensorPosition_1)
appli.robot.addTrace( zmp.name, 'zmp')


zmpestimated = ZmpFromForces('zmpestimated')
firstContactForceEst = Selec_of_vector('firstContactForceEst')
secondContactForceEst = Selec_of_vector('secondContactForceEst')
firstContactForceEst.selec(0,6)
secondContactForceEst.selec(6,12)
plug ( est1.forcesAndMoments, firstContactForceEst.sin)
plug ( est1.forcesAndMoments, secondContactForceEst.sin)
plug (firstContactForceEst.sout , zmpestimated.force_0)
plug (secondContactForceEst.sout, zmpestimated.force_1)
plug (robot.frames['rightFootForceSensor'].position , zmpestimated.sensorPosition_0)
plug (robot.frames['leftFootForceSensor'].position, zmpestimated.sensorPosition_1)
appli.robot.addTrace( zmpestimated.name, 'zmp')

plug(flex,appli.ccMc)
plug(flexdot,appli.ccVc)

appli.startTracer()





est1.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-2,)*3+(1e-6,)*3)))
appli.gains['trunk'].setConstant(2)

