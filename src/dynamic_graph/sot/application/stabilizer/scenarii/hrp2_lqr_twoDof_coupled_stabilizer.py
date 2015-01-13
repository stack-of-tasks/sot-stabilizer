# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 

from dynamic_graph.sot.application.stabilizer import HRP2LQRTwoDofCoupledStabilizer
from dynamic_graph import plug
import numpy as np
import dynamic_graph.signal_base as dgsb
    
from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose, MatrixHomoToPoseRollPitchYaw
from dynamic_graph.sot.core.matrix_util import matrixToTuple



class HRP2LqrTwoDofCoupledStabilizer(HRP2LQRTwoDofCoupledStabilizer):
    
    
    def __init__(self,robot,taskname = 'com-stabilized'):      

	from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator import HRP2ModelBaseFlexEstimator  
        HRP2LQRTwoDofCoupledStabilizer.__init__(self,taskname)
        robot.dynamic.com.recompute(0)
        robot.dynamic.Jcom.recompute(0)
        robot.dynamic.waist.recompute(0)
        robot.dynamic.Jwaist.recompute(0)
        
        plug (robot.dynamic.com, self.com)
        plug (robot.dynamic.Jcom, self.Jcom)
	plug (robot.dynamic.waist, self.waist)
	plug ( robot.dynamic.Jwaist, self.Jwaist) # /!\ There is certainly not only the jacobian of the orientation

	# For determining nbSupport
        plug (robot.device.forceLLEG,self.force_lf)
        plug (robot.device.forceRLEG,self.force_rf)
        plug (robot.frames['rightFootForceSensor'].position,self.rightFootPosition)
        plug (robot.frames['leftFootForceSensor'].position,self.leftFootPosition)

	# Estimator of the flexibility state
        self.estimator = HRP2ModelBaseFlexEstimator(robot, taskname+"Estimator")
        plug (self.nbSupport,self.estimator.contactNbr) # In
	
		# Contacts definition
	rFootPos = MatrixHomoToPoseRollPitchYaw('rFootFramePos')
	lFootPos = MatrixHomoToPoseRollPitchYaw('lFootFramePos')
	plug(robot.frames['rightFootForceSensor'].position,rFootPos.sin)
	plug(robot.frames['leftFootForceSensor'].position,lFootPos.sin)
	self.estimator.contacts = Stack_of_vector ('contacts')
	plug(rFootPos.sout,self.estimator.contacts.sin1)
	plug(lFootPos.sout,self.estimator.contacts.sin2)
	self.estimator.contacts.selec1 (0, 6)
	self.estimator.contacts.selec2 (0, 6)
	plug(self.estimator.contacts.sout,self.estimator.inputVector.contactsPosition)

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


