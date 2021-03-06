# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose, MatrixHomoToPoseRollPitchYaw 
from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator import HRP2ModelBaseFlexEstimator, PositionStateReconstructor 
from dynamic_graph.sot.application.stabilizer.scenarii.hand_compensater import HandCompensater
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.application.state_observation import InputReconstructor
from dynamic_graph.sot.application.state_observation import MovingFrameTransformation
import time
  

# Initialisation de l'appli
appli = HandCompensater(robot, True, False)
appli.withTraces()

est1 = HRP2ModelBaseFlexEstimator(robot)

meas = est1.signal('measurement')
inputs = est1.signal('input')
contactNbr = est1.signal('contactNbr')
flexVect=est1.signal('flexibility')
flex=est1.signal('flexMatrixInverse')
flexdot = est1.signal('flexInverseVelocityVector')

# contacts definition
contactNbr.value = 2
est1.setContactModel(2)
	# Position of the anchorage in the global frame
Peg = (0,0,4.60)
	# Positions of the contacts on the robot (in the local frame) with respect to the chest
Prl1 = np.matrix([[1,0,0,0-3.19997004e-02],[0,1,0,0.15-0],[0,0,1,1.28-1],[0,0,0,1]])
Prl2 = np.matrix([[1,0,0,0-3.19997004e-02],[0,1,0,-0.15-0],[0,0,1,1.28-1],[0,0,0,1]])

	# contact 1
contact1OpPoint = OpPointModifier('contact1_oppoint')
contact1OpPoint.setEndEffector(False)
contact1OpPoint.setTransformation(matrixToTuple(Prl1))
plug (robot.dynamic.chest,contact1OpPoint.positionIN)
contact1Pos = MatrixHomoToPose('contact1Pos')
plug(contact1OpPoint.position, contact1Pos.sin)
contact1 = Stack_of_vector ('contact1')
contact1.sin1.value = Peg
plug(contact1Pos.sout,contact1.sin2)
#contact1.sin2.value = (0,-0.15,1.28)
contact1.selec1 (0, 3)
contact1.selec2 (0, 3)


	# contact 2
contact2OpPoint = OpPointModifier('contact2_oppoint')
contact2OpPoint.setEndEffector(False)
contact2OpPoint.setTransformation(matrixToTuple(Prl2))
plug (robot.dynamic.chest,contact2OpPoint.positionIN)
contact2Pos = MatrixHomoToPose('contact2Pos')
plug(contact2OpPoint.position, contact2Pos.sin)
contact2 = Stack_of_vector ('contact2')
contact2.sin1.value = Peg
plug(contact2Pos.sout,contact2.sin2)
#contact2.sin2.value = (0,0.15,1.28)
contact2.selec1 (0, 3)
contact2.selec2 (0, 3)

	# concatenation
est1.contacts = Stack_of_vector ('contacts')
plug(contact1.sout,est1.contacts.sin1)
plug(contact2.sout,est1.contacts.sin2)
est1.contacts.selec1 (0, 6)
est1.contacts.selec2 (0, 6)
plug(est1.contacts.sout,est1.inputVector.contactsPosition)


# Simulation: Stifness and damping
kfe=40000
kfv=600 #600
kte=600
ktv=60
est1.setKfe(matrixToTuple(np.diag((kfe,kfe,kfe))))
est1.setKfv(matrixToTuple(np.diag((kfv,kfv,kfv))))
est1.setKte(matrixToTuple(np.diag((kte,kte,kte))))
est1.setKtv(matrixToTuple(np.diag((ktv,ktv,ktv))))

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
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name,  'gyrometer')
appli.robot.addTrace( est1.name,  'forcesAndMoments')
appli.robot.addTrace( est1.name,  'flexibilityComputationTime')

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
appli.startTracer()

plug(flex,appli.ccMc)
plug(flexdot,appli.ccVc)

est1.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-2,)*3+(1e-6,)*3)))
#est1.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e0,)*3+(1e0,)*3)))
appli.gains['trunk'].setConstant(2)


