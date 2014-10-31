from dynamic_graph.sot.application.stabilizer.scenarii.ds_stabilizer import DSStabilizer
from dynamic_graph.sot.application.stabilizer.scenarii.hrp2_linear_stabilizer import HRP2LinearStabilizer
from dynamic_graph.sot.core.meta_tasks import GainAdaptive
from dynamic_graph import plug
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from numpy import diag


class DSLinearStabilizerHRP2(DSStabilizer):
    def __init__(self,robot,trunkStabilize = True, hands = True, posture =True):
        DSStabilizer.__init__(self,robot,trunkStabilize,hands,posture)

    def createStabilizedCoMTask (self):
        task = HRP2LinearStabilizer(self.robot)
        gain = GainAdaptive('gain'+task.name)
        plug(self.comRef,task.comRef)
        task.comdot.value = (0.0,)*3
        plug(gain.gain, task.controlGain)
        plug(task.error, gain.error) 
        if self.trunkStabilize:
            self.ccMc = task.estimator.flexMatrixInverse
            self.ccVc = task.estimator.flexInverseVelocityVector
        return (task, gain)
    
    def initTaskPosture(self):
        # --- LEAST NORM
        weight_ff        = 0
        weight_leg       = 3
        weight_knee      = 5
        weight_chest     = 1
        weight_chesttilt = 10
        weight_head      = 0.3
        weight_arm       = 1

        #weight = diag( (weight_ff,)*6 + (weight_leg,)*12 + (weight_chest,)*2 + (weight_head,)*2 + (weight_arm,)*14)
        #weight[9,9] = weight_knee
        #weight[15,15] = weight_knee
        #weight[19,19] = weight_chesttilt

        weight = diag( (0,)*6+(1,)*30)
        #weight = weight[6:,:]

        self.featurePosture.jacobianIN.value = matrixToTuple(weight)
        self.featurePostureDes.errorIN.value = self.robot.halfSitting
        #mask = '1'*36
        #mask = '1'*14+'0'*22
        #self.tasks['posture'].controlSelec.value = mask
