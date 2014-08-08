# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 

from dynamic_graph.sot.application.stabilizer import HRP2LQRDecoupledStabilizer
from dynamic_graph import plug
import numpy as np
import dynamic_graph.signal_base as dgsb
    
from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector
from dynamic_graph.sot.core.matrix_util import matrixToTuple



class HRP2Stabilizer(HRP2LQRDecoupledStabilizer):
    
    
    def __init__(self,robot,taskname = 'com-stabilized'):
        
        from dynamic_graph.sot.application.state_observation.initializations.hrp2_flexibility_estimator import HRP2FlexibilityEstimator

        HRP2LQRDecoupledStabilizer.__init__(self,taskname)
        robot.dynamic.com.recompute(0)
        robot.dynamic.Jcom.recompute(0)
        
        plug (robot.dynamic.com, self.com)
        plug (robot.dynamic.Jcom, self.Jcom)
        plug (robot.device.forceLLEG,self.force_lf)
        plug (robot.device.forceRLEG,self.force_rf)
        plug (robot.frames['rightFootForceSensor'].position,self.rightFootPosition)
        plug (robot.frames['leftFootForceSensor'].position,self.leftFootPosition)
        #TODO ///// SELEC ??

        self.estimator = HRP2FlexibilityEstimator(robot, taskname+"Estimator")
        plug (self.nbSupport,self.estimator.contactNbr)
        
        plug(self.supportPos1,self.estimator.contact1)
        plug(self.supportPos2,self.estimator.contact2)
        
        plug(self.estimator.flexMatrixInverse, self.stateFlex )        
        plug(self.estimator.flexInverseOmega, self.stateFlexDot )
