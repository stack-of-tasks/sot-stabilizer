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

kfe=40000
kfv=600
kte=600
ktv=60

model.setKfe(matrixToTuple(np.diag((kfe,kfe,kfe))))
model.setKfv(matrixToTuple(np.diag((kfv,kfv,kfv))))
model.setKte(matrixToTuple(np.diag((kte,kte,kte))))
model.setKtv(matrixToTuple(np.diag((ktv,ktv,ktv))))

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

#stab.start()

stepTime = 0
simuTime =9
dt = 0.005

logState = np.array([])
logState.resize(simuTime/dt,25)

logControl = np.array([])
logControl.resize(simuTime/dt,6)

for i in range(int(stepTime/dt),int(simuTime/dt)):
   stab.task.recompute(i)
   model.incr(dt)
   logState[i,0] = i
   logState[i,1:] = model.state.value
   logControl[i,0] = i
   logControl[i,1:] = model.control.value

fig = plt.figure(); 

axfig = fig.add_subplot(241)
axfig.plot(logState[:,0], logState[:,1], label='com X')
axfig.plot(logState[:,0], logState[:,2], label='com Y')
axfig.plot(logState[:,0], logState[:,3], label='com Z')

axfig = fig.add_subplot(242)
axfig.plot(logState[:,0], logState[:,4], label='OmegaCH X')
axfig.plot(logState[:,0], logState[:,5], label='OmegaCH Y')
axfig.plot(logState[:,0], logState[:,6], label='OmegaCH Z')

axfig = fig.add_subplot(243)
axfig.plot(logState[:,0], logState[:,7], label='Omega X')
axfig.plot(logState[:,0], logState[:,8], label='Omega Y')
axfig.plot(logState[:,0], logState[:,9], label='Omega Z')

axfig = fig.add_subplot(244)
axfig.plot(logState[:,0], logState[:,10], label='t X')
axfig.plot(logState[:,0], logState[:,11], label='t Y')
axfig.plot(logState[:,0], logState[:,12], label='t Z')

axfig = fig.add_subplot(245)
axfig.plot(logState[:,0], logState[:,13], label='dcom X')
axfig.plot(logState[:,0], logState[:,14], label='dcom Y')
axfig.plot(logState[:,0], logState[:,15], label='dcom Z')

axfig = fig.add_subplot(246)
axfig.plot(logState[:,0], logState[:,16], label='dOmegaCH X')
axfig.plot(logState[:,0], logState[:,17], label='dOmegaCH Y')
axfig.plot(logState[:,0], logState[:,18], label='dOmegaCH Z')

axfig = fig.add_subplot(247)
axfig.plot(logState[:,0], logState[:,19], label='dOmega X')
axfig.plot(logState[:,0], logState[:,20], label='dOmega Y')
axfig.plot(logState[:,0], logState[:,21], label='dOmega Z')

axfig = fig.add_subplot(248)
axfig.plot(logState[:,0], logState[:,22], label='dt X')
axfig.plot(logState[:,0], logState[:,23], label='dt Y')
axfig.plot(logState[:,0], logState[:,24], label='dt Z')

handles, labels = axfig.get_legend_handles_labels()
axfig.legend(handles, labels)

fig = plt.figure(); 

axfig = fig.add_subplot(121)
axfig.plot(logControl[:,0], logControl[:,1], label='ddcom X')
axfig.plot(logControl[:,0], logControl[:,2], label='ddcom Y')
axfig.plot(logControl[:,0], logControl[:,3], label='ddcom Z')

axfig = fig.add_subplot(122)
axfig.plot(logControl[:,0], logControl[:,4], label='ddOmegaCH X')
axfig.plot(logControl[:,0], logControl[:,5], label='ddOmegaCH Y')

handles, labels = axfig.get_legend_handles_labels()
axfig.legend(handles, labels)

plt.show()

