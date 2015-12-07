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

	# DCoM
        self.DCom = Multiply_matrix_vector('DCom') # Com velocity: self.DCom.sout
        plug(robot.dynamic.Jcom,self.DCom.sin1)
        plug(robot.device.velocity,self.DCom.sin2)

	# DWaist
        self.DWaist = Multiply_matrix_vector('DWaist') # Waist angulat velocity: self.DWaist.sout
        plug(robot.dynamic.Jwaist,self.DWaist.sin1)
        plug(robot.device.velocity,self.DWaist.sin2)

	# Estimator of the flexibility state
        self.estimator = HRP2ModelBaseFlexEstimatorIMUForce (robot, taskname+"Estimator")
	self.estimator.setContactModel(1)
        plug (self.estimator.stackOfContacts.nbSupport,self.nbSupport)
	plug(self.estimator.flexPosition, self.tflex)
	plug(self.estimator.flexVelocity, self.dtflex)
	plug(self.estimator.flexAcceleration, self.ddtflex)

	# Control state
	plug(robot.dynamic.com, self.com)
	plug(robot.dynamic.waist,self.waistHomo)
        plug(self.estimator.flexThetaU, self.flexOriVect )
	plug(self.DCom.sout,self.comDot)
	plug(self.DWaist.sout,self.waistAngVel)
        plug(self.estimator.flexOmega, self.flexAngVelVect )

	# Jacobians
        plug ( robot.dynamic.Jcom  , self.Jcom  )
	plug ( robot.dynamic.Jwaist, self.Jwaist) 
	plug ( robot.dynamic.Jchest, self.Jchest)

	# Inertial values
	plug ( robot.dynamic.inertia, self.inertia)
        plug(robot.dynamic.angularmomentum,self.angularmomentum)



