# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 

from dynamic_graph.sot.application.stabilizer import HRP2LQRTwoDofCoupledStabilizer
from dynamic_graph import plug
import numpy as np
import dynamic_graph.signal_base as dgsb
    
from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose, MatrixHomoToPoseRollPitchYaw, MatrixToUTheta, HomoToMatrix, HomoToRotation
from dynamic_graph.sot.core.matrix_util import matrixToTuple

class HRP2LqrTwoDofCoupledStabilizerEncoders(HRP2LQRTwoDofCoupledStabilizer):
    
    def __init__(self,robot,taskname = 'com-stabilized'):      

	from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force_encoders import HRP2ModelBaseFlexEstimatorIMUForceEncoders
	from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import HRP2ModelBaseFlexEstimatorIMUForce
        HRP2LQRTwoDofCoupledStabilizer.__init__(self,taskname)
        robot.dynamic.com.recompute(0)
        robot.dynamic.Jcom.recompute(0)
        robot.dynamic.waist.recompute(0)
        robot.dynamic.Jwaist.recompute(0)
	robot.dynamic.inertia.recompute(0)

	# Reconstruction of the control state

		# DCoM
        self.DCom = Multiply_matrix_vector('DCom') 
        plug(robot.dynamic.Jcom,self.DCom.sin1)
        plug(robot.device.velocity,self.DCom.sin2)

		# DWaist
        self.DWaist = Multiply_matrix_vector('DWaist') 
        plug(robot.dynamic.Jwaist,self.DWaist.sin1)
        plug(robot.device.velocity,self.DWaist.sin2)

		# Estimator of the flexibility state
        self.estimatorEnc = HRP2ModelBaseFlexEstimatorIMUForceEncoders (robot, taskname+"EstimatorEncoders")
	self.estimator = HRP2ModelBaseFlexEstimatorIMUForce (robot, taskname+"Estimator")
	plug (self.estimatorEnc.interface.supportContactsNbr,self.nbSupport)
	plug(self.estimatorEnc.flexPosition, self.tflex)
	plug(self.estimatorEnc.flexVelocity, self.dtflex)

	# Control state
	plug(robot.dynamic.com, self.com)
	plug(robot.dynamic.waist,self.waistHomo)
        plug(self.estimatorEnc.flexThetaU, self.flexOriVect ) 
	plug(self.DCom.sout,self.comDot)
	plug(self.DWaist.sout,self.waistAngVel)
        plug(self.estimatorEnc.flexOmega, self.flexAngVelVect )

	# Jacobians
        plug (robot.dynamic.Jcom, self.Jcom)
	plug (robot.dynamic.Jchest, self.Jchest)
	plug ( robot.dynamic.Jwaist, self.Jwaist) 

	# Inertia
	plug ( robot.dynamic.inertia, self.inertia)
        plug(robot.dynamic.angularmomentum,self.angularmomentum)



