import numpy as np
from numpy import linalg as LA
import matplotlib.pyplot as plt
from dynamic_graph import plug
from dynamic_graph.sot.application.stabilizer import NonLinearRotationalTableCartDevice, HRP2LQRTwoDofCoupledStabilizer
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks import GainAdaptive
import math

model = NonLinearRotationalTableCartDevice("model")
stab = HRP2LQRTwoDofCoupledStabilizer("stabilizer")

I=((8.15831,-0.00380455,0.236677,),(-0.00380453,6.94757,-0.0465754),(0.236677,-0.0465754,1.73429))

# Model

model.setRobotMass(59.8)
model.setMomentOfInertia(I)
model.setKfe(matrixToTuple(1*np.diag((53200.0*0.2,53200.0*0.2,53200.0*0.2))))
model.setKfv(matrixToTuple(1*np.diag((100,100,100))))
model.setKte(matrixToTuple(1*np.diag((53200.0*0.2,53200.0*0.2,53200.0*0.2))))
model.setKtv(matrixToTuple(1*np.diag((100,100,100))))

# Stabilizer

stab.comRef.value=(0.00965, 0.0, 0.80777668336283626)
stab.waistOriRef.value=(0,)*3
stab.flexOriRef.value=(0,)*3
stab.comDotRef.value=(0,)*3
stab.waistVelRef.value=(0,)*3
stab.flexAngVelRef.value=(0,)*3
gain = GainAdaptive('gain'+stab.name)
plug(gain.gain, stab.controlGain)
plug(stab.error, gain.error) 

stab.setStateCost(matrixToTuple(10*np.diag((1,1,1,100,100,0.1,0.1,1,1,1,1,1,0.1,0.1))))
stab.setInputCost(matrixToTuple(1*np.diag((1,1,1,10,10))))
stab.setFixedGains(True)
stab.setHorizon(200)
stab.constantInertia(True)

plug(stab.control,model.control)
plug(model.com,stab.com)
plug(model.waistHomo,stab.waistHomo)
plug(model.flexOriVect,stab.flexOriVect)
plug(model.comDot,stab.comDot)
plug(model.waistVel,stab.waistVel)
plug(model.flexAngVelVect,stab.flexAngVelVect)

stab.start()

stepTime = 0
simuTime =9
dt = 0.005

for i in range(int(stepTime/dt),int(simuTime/dt)):
   stab.task.recompute(i)
   model.incr(dt)



