from dynamic_graph.sot.application.stabilizer.scenarii.ds_stabilizer import DSStabilizer
from dynamic_graph.sot.application.stabilizer.scenarii.hrp2_stabilizer import HRP2Stabilizer
from dynamic_graph.sot.core.meta_tasks import GainAdaptive
from dynamic_graph import plug


class DSStabilizerHRP2(DSStabilizer):
    def __init__(self,robot,trunkStabilize = True):
        DSStabilizer.__init__(self,robot,trunkStabilize)

    def createStabilizedCoMTask (self):
        task = HRP2Stabilizer(self.robot)
        gain = GainAdaptive('gain'+task.name)
        plug(self.comRef,task.comRef)
        task.comdot.value = (0.0,)*3
        plug(gain.gain, task.controlGain)
        plug(task.error, gain.error) 
        if self.trunkStabilize:
            plug(self.estimator.flexMatrixInverse, self.ccMc)
            plug(self.estimator.flexInverseVelocityVector, self.ccVc)
        return (task, gain)


