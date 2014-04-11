# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 

from dynamic_graph.sot.application.stabilizer import HRP2DecoupledStabilizer
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
    
from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector


class HRP2Stabilizer(HRP2DecoupledStabilizer):
    
    
    def __init__(robot,taskname = 'com-stabilized'):
        
        from dynamic_graph.sot.application.state_observation.initializations.hrp2_flexibility_estimator import HRP2FlexibilityEstimator

        HRP2DecoupledStabilizer.__init__(taskname)
        robot.dynamic.com.recompute(0)
        robot.dynamic.Jcom.recompute(0)
        plug (robot.dynamic.com, self.com)
        plug (robot.dynamic.Jcom, self.Jcom)
        plug (robot.device.forceLLEG,self.force_lf)
        plug (robot.device.forceRLEG,self.force_rf)
        plug (robot.frames['rightFootForceSensor'].position,self.rightFootPosition)
        plug (robot.frames['leftFootForceSensor'].position,self.leftFootPosition)
        #TODO ///// SELEC ??

        est = HRP2FlexibilityEstimator(robot, taskname+"estimator")
        plug (self.nbSupport,est.contactNbr)
        
        plug(self.supportPos1,est.contact1)
        plug(self.supportPos2,est.contact2)
