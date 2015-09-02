# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 

from dynamic_graph.sot.application.stabilizer import HRP2LQRTwoDofCoupledStabilizer
from dynamic_graph import plug
import numpy as np
import dynamic_graph.signal_base as dgsb
    
from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose, MatrixHomoToPoseRollPitchYaw, MatrixToUTheta, HomoToMatrix, HomoToRotation
from dynamic_graph.sot.core.matrix_util import matrixToTuple



class HRP2LqrTwoDofCoupledStabilizer(HRP2LQRTwoDofCoupledStabilizer):
    
    
    def __init__(self,robot,taskname = 'com-stabilized'):      

	from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import HRP2ModelBaseFlexEstimatorIMUForce  
        HRP2LQRTwoDofCoupledStabilizer.__init__(self,taskname)
        robot.dynamic.com.recompute(0)
        robot.dynamic.Jcom.recompute(0)
        robot.dynamic.waist.recompute(0)
        robot.dynamic.Jwaist.recompute(0)
	robot.dynamic.inertia.recompute(0)

        self.DCom = Multiply_matrix_vector('DCom') # Com velocity: self.DCom.sout
        plug(robot.dynamic.Jcom,self.DCom.sin1)
        plug(robot.device.velocity,self.DCom.sin2)

        self.DWaist = Multiply_matrix_vector('DWaist') # Waist angulat velocity: self.DWaist.sout
        plug(robot.dynamic.Jwaist,self.DWaist.sin1)
        plug(robot.device.velocity,self.DWaist.sin2)

	self.waistHomoToMatrix = HomoToRotation ('waistHomoToMatrix') # Waist orientation: self.waistMatrixToUTheta.sout
	plug(robot.dynamic.waist,self.waistHomoToMatrix.sin)
	self.waistMatrixToUTheta = MatrixToUTheta('waistMatrixToUTheta')
	plug(self.waistHomoToMatrix.sout,self.waistMatrixToUTheta.sin)

	# For determining nbSupport
        plug (robot.device.forceLLEG,self.force_lf)
        plug (robot.device.forceRLEG,self.force_rf)
        plug (robot.frames['rightFootForceSensor'].position,self.rightFootPosition)
        plug (robot.frames['leftFootForceSensor'].position,self.leftFootPosition)

	# Estimator of the flexibility state
        self.estimator = HRP2ModelBaseFlexEstimatorIMUForce (robot, taskname+"Estimator")
	self.estimator.setContactModel(1)
        plug (self.nbSupport,self.estimator.contactNbr) # In
	
		# Contacts definition
	self.estimator.contacts = Stack_of_vector ('contacts')
        plug(self.supportPos1,self.estimator.contacts.sin1)
        plug(self.supportPos2,self.estimator.contacts.sin2)
	self.estimator.contacts.selec1 (0, 6)
	self.estimator.contacts.selec2 (0, 6)
	plug(self.nbSupport,self.estimator.calibration.contactsNbr)
	plug(self.estimator.contacts.sout,self.estimator.calibration.contactsPositionIn)
	plug(self.estimator.calibration.contactsPositionOut,self.estimator.inputVector.contactsPosition)


		# Simulation: Stifness and damping
	kfe=40000
	kfv=600
	kte=600
	ktv=60
	self.estimator.setKfe(matrixToTuple(np.diag((kfe,kfe,kfe))))
	self.estimator.setKfv(matrixToTuple(np.diag((kfv,kfv,kfv))))
	self.estimator.setKte(matrixToTuple(np.diag((kte,kte,kte))))
	self.estimator.setKtv(matrixToTuple(np.diag((ktv,ktv,ktv))))

        plug(self.estimator.flexThetaU, self.flexOriVect ) # Out
        plug(self.estimator.flexOmega, self.flexAngVelVect )

	plug(self.estimator.flexPosition, self.tflex)
	plug(self.estimator.flexVelocity, self.dtflex)
	plug(self.estimator.flexAcceleration, self.ddtflex)

	# Control state
	plug(robot.dynamic.waist,self.waistHomo)

	#plug (robot.dynamic.com, self.com)
	plug(robot.dynamic.com, self.estimator.calibration.comIn)
	plug(robot.dynamic.com, self.com)
        plug (robot.dynamic.Jcom, self.Jcom)
	plug(self.DCom.sout,self.comDot)

	plug ( robot.dynamic.Jwaist, self.Jwaist) 
	plug ( robot.dynamic.inertia, self.inertia)
	plug(self.DWaist.sout,self.waistAngVel)

	plug ( robot.dynamic.Jchest, self.Jchest)

	# Angular momentum
        plug(robot.dynamic.angularmomentum,self.angularmomentum)



