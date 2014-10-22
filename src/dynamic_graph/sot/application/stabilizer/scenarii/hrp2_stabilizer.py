# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 

from dynamic_graph.sot.application.stabilizer import HRP2LQRDecoupledStabilizer
from dynamic_graph import plug
import numpy as np
import dynamic_graph.signal_base as dgsb
    
from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector
from dynamic_graph.sot.core.matrix_util import matrixToTuple



class HRP2Stabilizer(HRP2LQRDecoupledStabilizer):
    
    
    def __init__(self,robot,taskname = 'com-stabilized'):
        
        from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator import HRP2ModelBaseFlexEstimator

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

        self.estimator = HRP2ModelBaseFlexEstimator(robot, taskname+"Estimator")
        plug (self.nbSupport,self.estimator.contactNbr)


        self.contacts = Stack_of_vector (taskname+'contacts')

        plug(self.supportPos1,self.contacts.sin1)
        plug(self.supportPos2,self.contacts.sin2)
        
        self.contacts.selec1 (0, 6)
        self.contacts.selec2 (0, 6)
        plug(self.contacts.sout, self.estimator.inputVector.contactsPosition)
        
        plug(self.estimator.flexTransformationMatrix, self.stateFlex )        
        plug(self.estimator.flexOmega, self.stateFlexDot )
        plug(self.estimator.flexOmegaDot, self.stateFlexDDot )

        self.comVectorPV = Stack_of_vector (taskname+'ComVectorPV')
        self.comVector = Stack_of_vector (taskname+'ComVector')
        
        self.comVectorPV.selec1 (0, 3)
        self.comVectorPV.selec2 (0, 3)        
        self.comVector.selec1 (0, 6)
        self.comVector.selec2 (0, 3)        
        

        plug(robot.dynamic.com,     self.comVectorPV.sin1)
        plug(self.comdot,           self.comVectorPV.sin2)
        plug(self.comVectorPV.sout, self.comVector.sin1)
        plug(self.comddot,          self.comVector.sin2)
        


    def start(self):
        plug(self.comVector.sout,self.inputVector.comVector)
        HRP2LQRDecoupledStabilizer.start()
        

    def stop(self):
        for i in range(1,self.inputVector.getFiniteDifferencesInterval()):
            self.inputVector.comVector.recompute(self.inputVector.comVector.time + 1)
        plug(self.estimator.comVector.sout,self.inputVector.comVector)
        HRP2LQRDecoupledStabilizer.stop()
        
