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

model.setRobotMass(59.8)
#model.setMomentOfInertia()

# Model

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

stab.start()

stab.setStateCost(matrixToTuple(10*np.diag((1,1,1,100,100,0.1,0.1,1,1,1,1,1,0.1,0.1))))
stab.setInputCost(matrixToTuple(1*np.diag((1,1,1,10,10))))




    plug(cart.comHeight,controller.comHeight)
    plug(cart.state,controller.comIn)
    controller.comddotIN.value = (0.0,)
    plug(controller.comddot,cart.control)

    plug(cart.comHeight,controller.comHeight)
    plug(cart.comreal,controller.comIn)
    plug(cart.flexcomddot,controller.comddotIN)
    plug(controller.comddot,cart.control)

stepTime = 0.05
simuTime = 9

for i in range(int(stepTime/dt),int(simuTime/dt)):
    print(i)
    model.incr(dt)


