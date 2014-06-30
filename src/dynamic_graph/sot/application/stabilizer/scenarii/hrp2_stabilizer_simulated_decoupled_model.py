# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 

from dynamic_graph.sot.application.stabilizer import HRP2DecoupledStabilizer
from dynamic_graph import plug
import numpy as np
import dynamic_graph.signal_base as dgsb
    
from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector
from dynamic_graph.sot.core.matrix_util import matrixToTuple



class HRP2StabilizerSimulatedDecoupled(HRP2DecoupledStabilizer):
    
    
    def __init__(self,name,robot):
        
        from dynamic_graph.sot.application.stabilizer import DecoupledElasticInvPendulum

        HRP2DecoupledStabilizer.__init__(self,name)
        robot.dynamic.com.recompute(0)
        robot.dynamic.Jcom.recompute(0)
        
        plug (robot.dynamic.com, self.com)
        plug (robot.dynamic.Jcom, self.Jcom)
        plug (robot.device.forceLLEG,self.force_lf)
        plug (robot.device.forceRLEG,self.force_rf)
        plug (robot.frames['rightFootForceSensor'].position,self.rightFootPosition)
        plug (robot.frames['leftFootForceSensor'].position,self.leftFootPosition)
        #TODO ///// SELEC ??

        self.simulator = DecoupledElasticInvPendulum(name+"Simulator")
        plug (self.nbSupport,self.simulator.contactNbr)
        
        plug(self.supportPos1,self.simulator.contact1)
        plug(self.supportPos2,self.simulator.contact2)
        
        plug(self.simulator.flex, self.stateFlex )        
        plug(self.simulator.flexdot, self.stateFlexDot )

        plug (robot.dynamic.com, self.simulator.comIn)
        plug (self.d2com, self.simulator.comddot)
        
    def init(self,comref,comdotref,gain):
        plug(comref, self.comRef)
        plug(comdotref, self.comDotRef)
        self.simulator.resetCoM()
        self.gain.value = gain
        
        
