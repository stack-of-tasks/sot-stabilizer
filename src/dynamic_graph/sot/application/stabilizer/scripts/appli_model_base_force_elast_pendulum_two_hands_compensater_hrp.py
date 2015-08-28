# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose, MatrixHomoToPoseUTheta,MatrixHomoToPoseRollPitchYaw, Selec_of_vector
from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import HRP2ModelBaseFlexEstimatorIMUForce, PositionStateReconstructor 
from dynamic_graph.sot.application.stabilizer.scenarii.hand_compensater import HandCompensater
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.application.state_observation import InputReconstructor
from dynamic_graph.sot.dynamics.zmp_from_forces import ZmpFromForces
from dynamic_graph.sot.application.stabilizer import VectorPerturbationsGenerator
import time


# Initialisation de l'appli
appli = HandCompensater(robot, True, False)
appli.withTraces()

est= est2 = HRP2ModelBaseFlexEstimatorIMUForce(robot,'forceflextimator')

perturbation = 10;
perturbator = VectorPerturbationsGenerator('perturbator')
perturbator.setMode(2)
perturbator.activate(True)
perturbator.perturbation.value = (perturbation,)*12
perturbator.selec.value = '1'*12
plug(est.sensorStackforce.sout,perturbator.sin)
plug(perturbator.sout,est.sensorStack.sin2)


meas = est2.signal('measurement')
inputs = est2.signal('input')
contactNbr = est2.signal('contactNbr')

# Definition des contacts
contactNbr.value = 2
est2.setContactModel(1)
rFootPos = MatrixHomoToPoseUTheta('rFootFramePos')
lFootPos = MatrixHomoToPoseUTheta('lFootFramePos')
plug(robot.frames['rightFootForceSensor'].position,rFootPos.sin)
plug(robot.frames['leftFootForceSensor'].position,lFootPos.sin)

est2.contacts = Stack_of_vector ('contacts')
plug(lFootPos.sout,est2.contacts.sin1)
plug(rFootPos.sout,est2.contacts.sin2)
est2.contacts.selec1 (0, 6)
est2.contacts.selec2 (0, 6)
plug(est2.contacts.sout,est2.inputVector.contactsPosition)



# Simulation: Stifness and damping
kfe=50000
kfv=600
kte=600
ktv=60
est2.setKfe(matrixToTuple(np.diag((kfe,kfe,kfe))))
est2.setKfv(matrixToTuple(np.diag((kfv,kfv,kfv))))
est2.setKte(matrixToTuple(np.diag((kte,kte,kte))))
est2.setKtv(matrixToTuple(np.diag((ktv,ktv,ktv))))

# Robot: Stifness and damping
#est2.setKfe(matrixToTuple(np.diag((kfe,kfe,kfe))))
#est2.setKfv(matrixToTuple(np.diag((kfv,kfv,kfv))))
#est2.setKte(matrixToTuple(np.diag((320,475,400))) # random z
#est2.setKtv(matrixToTuple(np.diag((40,3,40)))) # random z


flexVect=est2.signal('flexibility')
flex=est2.signal('flexMatrixInverse')
flexdot = est2.signal('flexInverseVelocityVector')

appli.robot.addTrace( est2.name,'flexibility' )
appli.robot.addTrace( est2.name,'input' )
#appli.robot.addTrace( est2.name,'flexInverseVelocityVector' )
#appli.robot.addTrace( est2.name,'flexMatrixInverse' )
appli.robot.addTrace( est2.name,'measurement')
appli.robot.addTrace( est2.name,'simulatedSensors' )
appli.robot.addTrace( est2.name,'predictedSensors' )
appli.robot.addTrace( est2.name,'inovation' )
appli.robot.addTrace( est2.name,'prediction' )
appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
#appli.robot.addTrace( est2.name,  'forcesAndMoments')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name,  'gyrometer')


appli.robot.addTrace( est2.name,  'flexibilityComputationTime')

# Position main droite
RHand = MatrixHomoToPoseRollPitchYaw('HomotoPoseRH')
plug(robot.dynamic.signal('right-wrist'),RHand.sin)
#appli.robot.addTrace( RHand.name,  'sout')

# Position main gauche
LHand = MatrixHomoToPoseRollPitchYaw('HomotoPoseLH')
plug(robot.dynamic.signal('left-wrist'),LHand.sin)
#appli.robot.addTrace( LHand.name,  'sout')

# Position ref main droite
RHandref = MatrixHomoToPoseRollPitchYaw('HomotoPoseRHR')
plug(appli.features['right-wrist'].reference,RHandref.sin)
#appli.robot.addTrace( RHandref.name,  'sout')

# Position ref main gauche
LHandref = MatrixHomoToPoseRollPitchYaw('HomotoPoseLHR')
plug(appli.features['left-wrist'].reference,LHandref.sin)
#appli.robot.addTrace( LHandref.name,  'sout')

# Position ref main droite
#appli.robot.addTrace( 'tranformation_right',  'gV0')

# Position ref main gauche
#appli.robot.addTrace( 'symvel' ,  'sout')

deriv=PositionStateReconstructor('deriv')
deriv.inputFormat.value='000010'
deriv.outputFormat.value='001000'
deriv.setSampligPeriod(0.005)
plug(est2.flexThetaU,deriv.sin)

integr=PositionStateReconstructor('integr')
integr.inputFormat.value='000100'
integr.outputFormat.value='000001'
integr.setSampligPeriod(0.005)
plug(est2.flexibility,integr.sin)

#appli.robot.addTrace( est2.name,'flexThetaU' )
#appli.robot.addTrace( est2.name,'flexOmega' )
#appli.robot.addTrace( deriv.name,'sout')
#appli.robot.addTrace( integr.name,'sout')

zmp = ZmpFromForces('zmp')
plug (robot.device.forceRLEG , zmp.force_0)
plug (robot.device.forceLLEG, zmp.force_1)
plug (robot.frames['rightFootForceSensor'].position , zmp.sensorPosition_0)
plug (robot.frames['leftFootForceSensor'].position, zmp.sensorPosition_1)
appli.robot.addTrace( zmp.name, 'zmp')

zmpnoisy = ZmpFromForces('zmpnoisy')
firstContactForceEst = Selec_of_vector('firstContactForcenoise')
secondContactForceEst = Selec_of_vector('secondContactForcenoise')
firstContactForceEst.selec(0,6)
secondContactForceEst.selec(6,12)
plug ( perturbator.sout, firstContactForceEst.sin)
plug ( perturbator.sout, secondContactForceEst.sin)
plug (firstContactForceEst.sout , zmpnoisy.force_0)
plug (secondContactForceEst.sout, zmpnoisy.force_1)
plug (robot.frames['rightFootForceSensor'].position , zmpnoisy.sensorPosition_1)
plug (robot.frames['leftFootForceSensor'].position, zmpnoisy.sensorPosition_0)
appli.robot.addTrace( zmpnoisy.name, 'zmp')


zmpestimated0 = ZmpFromForces('zmpestimated')
firstContactForceEst = Selec_of_vector('firstContactForceEst')
secondContactForceEst = Selec_of_vector('secondContactForceEst')
firstContactForceEst.selec(0,6)
secondContactForceEst.selec(6,12)
plug ( est2.forcesAndMoments, firstContactForceEst.sin)
plug ( est2.forcesAndMoments, secondContactForceEst.sin)
plug (firstContactForceEst.sout , zmpestimated0.force_0)
plug (secondContactForceEst.sout, zmpestimated0.force_1)
plug (robot.frames['rightFootForceSensor'].position , zmpestimated0.sensorPosition_1)
plug (robot.frames['leftFootForceSensor'].position, zmpestimated0.sensorPosition_0)
appli.robot.addTrace( zmpestimated0.name, 'zmp')

plug(flex,appli.ccMc)
plug(flexdot,appli.ccVc)




est2.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-2,)*3+(1e-6,)*3)))
est2.setForceVariance(10*perturbation)

est2.setWithForceSensors(True)

################################

from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator import HRP2ModelBaseFlexEstimator, PositionStateReconstructor 


est1 = HRP2ModelBaseFlexEstimator(robot)

meas = est1.signal('measurement')
inputs = est1.signal('input')
contactNbr = est1.signal('contactNbr')

# Definition des contacts
contactNbr.value = 2
est1.setContactModel(1)

est1.contacts = Stack_of_vector ('contacts2')
plug(lFootPos.sout,est1.contacts.sin1)
plug(rFootPos.sout,est1.contacts.sin2)
est1.contacts.selec1 (0, 6)
est1.contacts.selec2 (0, 6)
plug(est1.contacts.sout,est1.inputVector.contactsPosition)


est1.setKfe(matrixToTuple(np.diag((kfe,kfe,kfe))))
est1.setKfv(matrixToTuple(np.diag((kfv,kfv,kfv))))
est1.setKte(matrixToTuple(np.diag((kte,kte,kte))))
est1.setKtv(matrixToTuple(np.diag((ktv,ktv,ktv))))



flexVect=est1.signal('flexibility')
flex=est1.signal('flexMatrixInverse')
flexdot = est1.signal('flexInverseVelocityVector')

appli.robot.addTrace( est1.name,'flexibility' )
appli.robot.addTrace( est1.name,'input' )
#appli.robot.addTrace( est1.name,'flexInverseVelocityVector' )
#appli.robot.addTrace( est1.name,'flexMatrixInverse' )
appli.robot.addTrace( est1.name,'input')
appli.robot.addTrace( est1.name,'measurement')
appli.robot.addTrace( est1.name,'simulatedSensors' )
appli.robot.addTrace( est1.name,'predictedSensors' )
appli.robot.addTrace( est1.name,'inovation' )
appli.robot.addTrace( est1.name,'prediction' )
#appli.robot.addTrace( est1.name,  'forcesAndMoments')

appli.robot.addTrace( est1.name,  'flexibilityComputationTime')

#appli.robot.addTrace( est1.name,'flexThetaU' )
#appli.robot.addTrace( est1.name,'flexOmega' )

zmpestimated2 = ZmpFromForces('zmpestimated2')
firstContactForceEst = Selec_of_vector('firstContactForceEst2')
secondContactForceEst = Selec_of_vector('secondContactForceEst2')
firstContactForceEst.selec(0,6)
secondContactForceEst.selec(6,12)
plug ( est1.forcesAndMoments, firstContactForceEst.sin)
plug ( est1.forcesAndMoments, secondContactForceEst.sin)
plug (firstContactForceEst.sout , zmpestimated2.force_0)
plug (secondContactForceEst.sout, zmpestimated2.force_1)
plug (robot.frames['rightFootForceSensor'].position , zmpestimated2.sensorPosition_1)
plug (robot.frames['leftFootForceSensor'].position, zmpestimated2.sensorPosition_0)
appli.robot.addTrace( zmpestimated2.name, 'zmp')
est1.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-2,)*3+(1e-6,)*3)))





############################


est3 = HRP2ModelBaseFlexEstimatorIMUForce(robot,"ForceFlextimator2")

meas = est3.signal('measurement')
inputs = est3.signal('input')
contactNbr = est3.signal('contactNbr')

# Definition des contacts
contactNbr.value = 2
est3.setContactModel(1)

est3.contacts = Stack_of_vector ('contacts3')
plug(lFootPos.sout,est3.contacts.sin1)
plug(rFootPos.sout,est3.contacts.sin2)
est3.contacts.selec1 (0, 6)
est3.contacts.selec2 (0, 6)
plug(est3.contacts.sout,est3.inputVector.contactsPosition)
plug(perturbator.sout,est3.sensorStack.sin2)
est3.sensorStack.sin1.value = (0,)*6


est3.setKfe(matrixToTuple(np.diag((kfe,kfe,kfe))))
est3.setKfv(matrixToTuple(np.diag((kfv,kfv,kfv))))
est3.setKte(matrixToTuple(np.diag((kte,kte,kte))))
est3.setKtv(matrixToTuple(np.diag((ktv,ktv,ktv))))


flexVect=est3.signal('flexibility')
flex=est3.signal('flexMatrixInverse')
flexdot = est3.signal('flexInverseVelocityVector')

appli.robot.addTrace( est3.name,'flexibility' )
appli.robot.addTrace( est3.name,'input' )
appli.robot.addTrace( est3.name,'flexInverseVelocityVector' )
appli.robot.addTrace( est3.name,'flexMatrixInverse' )
appli.robot.addTrace( est3.name,'input')
appli.robot.addTrace( est3.name,'measurement')
appli.robot.addTrace( est3.name,'simulatedSensors' )
appli.robot.addTrace( est3.name,'predictedSensors' )
appli.robot.addTrace( est3.name,'inovation' )
appli.robot.addTrace( est3.name,'prediction' )
#appli.robot.addTrace( est3.name,  'forcesAndMoments')

appli.robot.addTrace( est3.name,  'flexibilityComputationTime')

#appli.robot.addTrace( est3.name,'flexThetaU' )
#appli.robot.addTrace( est3.name,'flexOmega' )

zmpestimated3 = ZmpFromForces('zmpestimated3')
firstContactForceEst = Selec_of_vector('firstContactForceEst3')
secondContactForceEst = Selec_of_vector('secondContactForceEst3')
firstContactForceEst.selec(0,6)
secondContactForceEst.selec(6,12)
plug ( est3.forcesAndMoments, firstContactForceEst.sin)
plug ( est3.forcesAndMoments, secondContactForceEst.sin)
plug (firstContactForceEst.sout , zmpestimated3.force_0)
plug (secondContactForceEst.sout, zmpestimated3.force_1)
plug (robot.frames['rightFootForceSensor'].position , zmpestimated3.sensorPosition_1)
plug (robot.frames['leftFootForceSensor'].position, zmpestimated3.sensorPosition_0)
appli.robot.addTrace( zmpestimated3.name, 'zmp')
est3.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e8,)*3+(1e8,)*3)))
est3.setForceVariance(perturbation)
est3.setWithForceSensors(True)

appli.startTracer()

appli.comRef.value = (0.01, 0.0, 0.80)


