from dynamic_graph.sot.application.stabilizer.scenarii.ds_stabilizer import DSStabilizer
from dynamic_graph.sot.application.stabilizer.scenarii.hrp2_lqr_twoDof_coupled_stabilizer import HRP2LqrTwoDofCoupledStabilizer
from dynamic_graph.sot.core.meta_tasks import GainAdaptive
from dynamic_graph import plug
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core import  MatrixToUTheta, HomoToMatrix, HomoToRotation, Multiply_matrix_vector
from numpy import diag


class DSLqrTwoDofCoupledStabilizerHRP2(DSStabilizer):
    def __init__(self,robot,trunkStabilize = False, hands = False, posture =False):
        DSStabilizer.__init__(self,robot,trunkStabilize,hands,posture)

    def createStabilizedCoMTask (self):
        task = HRP2LqrTwoDofCoupledStabilizer(self.robot)
        gain = GainAdaptive('gain'+task.name)

        self.DCom = Multiply_matrix_vector('DCom') # Com velocity: self.DCom.sout
        plug(self.robot.dynamic.Jcom,self.DCom.sin1)
        plug(self.robot.device.velocity,self.DCom.sin2)

	waistHomoToMatrix = HomoToRotation ('waistHomoToMatrix') # Waist orientation: self.waistMatrixToUTheta.sout
	plug(self.robot.dynamic.waist,waistHomoToMatrix.sin)
	self.waistMatrixToUTheta = MatrixToUTheta('waistMatrixToUTheta')
	plug(waistHomoToMatrix.sout,self.waistMatrixToUTheta.sin)

        self.DWaist = Multiply_matrix_vector('DWaist') # Waist angulat velocity: self.DWaist.sout
        plug(self.robot.dynamic.Jwaist,self.DWaist.sin1)
        plug(self.robot.device.velocity,self.DWaist.sin2)
	
	# References
	task.waistOriRefg.value=self.waistMatrixToUTheta.sout.value

	task.comRefg.value[0] = 0.5*(stabilizer.supportPos1[0]+stabilizer.supportPos2[0])
	task.comRefg.value[1] = 0.5*(stabilizer.supportPos1[1]+stabilizer.supportPos2[1])
	task.comRefg.value[2] = self.comRef.value[2]

	# Control state
	plug(self.waistMatrixToUTheta.sout,task.waistOri)
	plug(self.DCom.sout,task.comDot)
	plug(self.DWaist.sout,task.waistVel)

        plug(gain.gain, task.controlGain)
        plug(task.error, gain.error) 
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
