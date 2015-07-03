import numpy as np
from numpy import linalg as LA
import matplotlib.pyplot as plt
from dynamic_graph import plug
from dynamic_graph.sot.application.stabilizer import NonLinearRotationalTableCartDevice, HRP2LQRTwoDofCoupledStabilizer
from dynamic_graph.sot.application.stabilizer import VectorPerturbationsGenerator
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks import GainAdaptive
import math
from dynamic_graph.sot.dynamics.zmp_from_forces import ZmpFromForces

model = NonLinearRotationalTableCartDevice("model")
stab = HRP2LQRTwoDofCoupledStabilizer("stabilizer")

I=((8.15831,-0.00380455,0.236677),(-0.00380453,6.94757,-0.0465754),(0.236677,-0.0465754,1.73429))

a=1
I1=((a*8.15831,a*-0.00380455,a*0.236677),(a*-0.00380453,a*6.94757,a*-0.0465754),(a*0.236677,a*-0.0465754,a*1.73429))

kfe=40000
kfv=600
kte=600
ktv=60

b=1
kfe1=b*40000
kfv1=b*600
kte1=b*600
ktv1=b*60


# Model

model.setRobotMass(59.8)
model.setMomentOfInertia(I)

model.setKfe(matrixToTuple(np.diag((kfe,kfe,kfe))))
model.setKfv(matrixToTuple(np.diag((kfv,kfv,kfv))))
model.setKte(matrixToTuple(np.diag((kte,kte,kte))))
model.setKtv(matrixToTuple(np.diag((ktv,ktv,ktv))))

model.setState((0.0, 0.0, 0.80771, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

# Stabilizer

stab.setInertia(I1)

stab.setKth(matrixToTuple(np.diag((kte1,kte1,kte1))))
stab.setKdth(matrixToTuple(np.diag((ktv1,ktv1,ktv1))))

stab.comRef.value=(0.00949, 0.0, 0.80771)
stab.comRef.value=(0.0, 0.0, 0.80771)
stab.waistOriRef.value=(0,)*3
stab.flexOriRef.value=(0,)*3
stab.comDotRef.value=(0,)*3
stab.waistVelRef.value=(0,)*3
stab.flexAngVelRef.value=(0,)*3

gain = GainAdaptive('gain'+stab.name)
plug(gain.gain, stab.controlGain)
plug(stab.error, gain.error) 

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
plug(model.contact1Pos,stab.leftFootPosition)
plug(model.contact2Pos,stab.rightFootPosition)
plug(model.contact1Forces,stab.force_lf)
plug(model.contact2Forces,stab.force_rf)

perturbator = VectorPerturbationsGenerator('perturbation')
perturbator.setSinLessMode(True)
vect = perturbator.sin
vect.value = stab.comRef.value
plug (perturbator.sout,stab.perturbationAcc)
perturbator.perturbation.value=(5,0,0)
perturbator.selec.value = '111'
perturbator.setMode(0)
perturbator.setPeriod(1000)
perturbator.activate(True)

stab.start()

stepTime = 5
step2Time=10
step3Time=15
simuTime =20
dt = 0.005

logState = np.array([])
logState.resize(simuTime/dt,25)
logControl = np.array([])
logControl.resize(simuTime/dt,6)
logFlexAcc = np.array([])
logFlexAcc.resize(simuTime/dt,7)
logZmp1 = np.array([])
logZmp1.resize(simuTime/dt,4)
logZmp2 = np.array([])
logZmp2.resize(simuTime/dt,4)
logZmp3 = np.array([])
logZmp3.resize(simuTime/dt,4)
logZmp4 = np.array([])
logZmp4.resize(simuTime/dt,4)
logForces1 = np.array([])
logForces1.resize(simuTime/dt,7)
logForces2 = np.array([])
logForces2.resize(simuTime/dt,7)
logStabSimulatedState = np.array([])
logStabSimulatedState.resize(simuTime/dt,15)

zmp = ZmpFromForces('zmp')
plug (model.contact1Forces, zmp.force_0)
plug (model.contact2Forces, zmp.force_1)
plug (model.contact1Pos, zmp.sensorPosition_0)
plug (model.contact2Pos, zmp.sensorPosition_1)

stab.setStateCost(matrixToTuple(1*np.diag((2000,2000,2000,2000,2000,1,1,1,1,1,1,1,1,1))))
stab.setInputCost(matrixToTuple(1*np.diag((1,1,1,1,1))))

for i in range(int(0/dt),int(stepTime/dt)):
   stab.task.recompute(i)
   zmp.zmp.recompute(i)
   model.incr(dt)
   print 'boucle 1:'+str(i)
   logState[i,0] = i
   logState[i,1:] = model.state.value
   logControl[i,0] = i
   logControl[i,1:] = model.control.value
   logFlexAcc[i,0]=i
   logFlexAcc[i,1:4]=model.flexAngAccVect.value
   logFlexAcc[i,4:7]=model.flexLinAcc.value
   logZmp1[i,0]=i
   logZmp1[i,1:]=zmp.zmp.value
   logForces1[i,0]=i
   logForces1[i,1:]=model.contact1Forces.value
   logForces2[i,0]=i
   logForces2[i,1:]=model.contact1Forces.value
   logStabSimulatedState[i,0]=i
   logStabSimulatedState[i,1:]=stab.stateSimulation.value

stab.setStateCost(matrixToTuple(100*np.diag((1,1,1,1,1,1,1,1,1,1,1,1,1,1))))
stab.setInputCost(matrixToTuple(1*np.diag((1,1,1,10,10))))

for i in range(int(stepTime/dt),int(step2Time/dt)):
   stab.task.recompute(i)
   zmp.zmp.recompute(i)
   model.incr(dt)
   print 'boucle 2:'+str(i)
   logState[i,0] = i
   logState[i,1:] = model.state.value
   logControl[i,0] = i
   logControl[i,1:] = model.control.value
   logFlexAcc[i,0]=i
   logFlexAcc[i,1:4]=model.flexAngAccVect.value
   logFlexAcc[i,4:7]=model.flexLinAcc.value
   logZmp2[i,0]=i
   logZmp2[i,1:]=zmp.zmp.value
   logForces1[i,0]=i
   logForces1[i,1:]=model.contact1Forces.value
   logForces2[i,0]=i
   logForces2[i,1:]=model.contact1Forces.value
   logStabSimulatedState[i,0]=i
   logStabSimulatedState[i,1:]=stab.stateSimulation.value

stab.setStateCost(matrixToTuple(100*np.diag((1,1,1,1,1,1,1,1,1,1,1,1,1,1))))
stab.setInputCost(matrixToTuple(1*np.diag((1,1,1,1,1))))


for i in range(int(step2Time/dt),int(step3Time/dt)):
   stab.task.recompute(i)
   zmp.zmp.recompute(i)
   model.incr(dt)
   print 'boucle 2:'+str(i)
   logState[i,0] = i
   logState[i,1:] = model.state.value
   logControl[i,0] = i
   logControl[i,1:] = model.control.value
   logFlexAcc[i,0]=i
   logFlexAcc[i,1:4]=model.flexAngAccVect.value
   logFlexAcc[i,4:7]=model.flexLinAcc.value
   logZmp3[i,0]=i
   logZmp3[i,1:]=zmp.zmp.value
   logForces1[i,0]=i
   logForces1[i,1:]=model.contact1Forces.value
   logForces2[i,0]=i
   logForces2[i,1:]=model.contact1Forces.value
   logStabSimulatedState[i,0]=i
   logStabSimulatedState[i,1:]=stab.stateSimulation.value

stab.setStateCost(matrixToTuple(10*np.diag((1,1,1,1,1,1,1,1,1,1,1,1,1,1))))
stab.setInputCost(matrixToTuple(1*np.diag((1,1,1,1,1))))


for i in range(int(step3Time/dt),int(simuTime/dt)):
   stab.task.recompute(i)
   zmp.zmp.recompute(i)
   model.incr(dt)
   print 'boucle 3:'+str(i)
   logState[i,0] = i
   logState[i,1:] = model.state.value
   logControl[i,0] = i
   logControl[i,1:] = model.control.value
   logFlexAcc[i,0]=i
   logFlexAcc[i,1:4]=model.flexAngAccVect.value
   logFlexAcc[i,4:7]=model.flexLinAcc.value
   logZmp4[i,0]=i
   logZmp4[i,1:]=zmp.zmp.value
   logForces1[i,0]=i
   logForces1[i,1:]=model.contact1Forces.value
   logForces2[i,0]=i
   logForces2[i,1:]=model.contact1Forces.value
   logStabSimulatedState[i,0]=i
   logStabSimulatedState[i,1:]=stab.stateSimulation.value

# Plot state
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

# Plot control
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

# Flex acc
#fig = plt.figure(); 

#axfig = fig.add_subplot(121)
#axfig.plot(logFlexAcc[:,0], logFlexAcc[:,1], label='ddOmega X')
#axfig.plot(logFlexAcc[:,0], logFlexAcc[:,2], label='ddOmega Y')
#axfig.plot(logFlexAcc[:,0], logFlexAcc[:,3], label='ddOmega Z')

#axfig = fig.add_subplot(122)
#axfig.plot(logFlexAcc[:,0], logFlexAcc[:,4], label='ddt X')
#axfig.plot(logFlexAcc[:,0], logFlexAcc[:,5], label='ddt Y')
#axfig.plot(logFlexAcc[:,0], logFlexAcc[:,6], label='ddt Z')

#handles, labels = axfig.get_legend_handles_labels()
#axfig.legend(handles, labels)

# ZMP
fig = plt.figure(); 

axfig = fig.add_subplot(141)
axfig.plot(logZmp1[:,2], logZmp1[:,1], label='ZMP')
axfig = fig.add_subplot(142)
axfig.plot(logZmp2[:,2], logZmp2[:,1], label='ZMP')
axfig = fig.add_subplot(143)
axfig.plot(logZmp3[:,2], logZmp3[:,1], label='ZMP')
axfig = fig.add_subplot(144)
axfig.plot(logZmp4[:,2], logZmp4[:,1], label='ZMP')
#axfig.plot(logZmp[:,0], logZmp[:,2], label='ZMP Y')
#axfig.plot(logZmp[:,0], logZmp[:,3], label='ZMP Z')

handles, labels = axfig.get_legend_handles_labels()
axfig.legend(handles, labels)

# contact 1 forces
#fig = plt.figure(); 

#axfig = fig.add_subplot(121)
#axfig.plot(logForces1[:,0], logForces1[:,1], label='force X')
#axfig.plot(logForces1[:,0], logForces1[:,2], label='force Y')
#axfig.plot(logForces1[:,0], logForces1[:,3], label='force Z')

#axfig = fig.add_subplot(122)
#axfig.plot(logForces1[:,0], logForces1[:,4], label='moment X')
#axfig.plot(logForces1[:,0], logForces1[:,5], label='moment Y')
#axfig.plot(logForces1[:,0], logForces1[:,6], label='moment Z')

#handles, labels = axfig.get_legend_handles_labels()
#axfig.legend(handles, labels)

# contact 2 forces
#fig = plt.figure(); 

#axfig = fig.add_subplot(121)
#axfig.plot(logForces2[:,0], logForces2[:,1], label='force X')
#axfig.plot(logForces2[:,0], logForces2[:,2], label='force Y')
#axfig.plot(logForces2[:,0], logForces2[:,3], label='force Z')

#axfig = fig.add_subplot(122)
#axfig.plot(logForces2[:,0], logForces2[:,4], label='moment X')
#axfig.plot(logForces2[:,0], logForces2[:,5], label='moment Y')
#axfig.plot(logForces2[:,0], logForces2[:,6], label='moment Z')

#handles, labels = axfig.get_legend_handles_labels()
#axfig.legend(handles, labels)

# models comparaison
fig = plt.figure();

i=1
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,1], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')
i=2
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,2], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')
i=3
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,3], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')

i=4
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,4], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')
i=5
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,5], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')

i=6
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,7], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')
i=7
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,8], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')

i=8
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,13], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')
i=9
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,14], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')
i=10
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,15], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')

i=11
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,16], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')
i=12
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,17], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')

i=13
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,19], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')
i=14
axfig = fig.add_subplot(2,7,i)
axfig.plot(logState[:,0], logState[:,20], label='model state')
axfig.plot(logStabSimulatedState[:,0], logStabSimulatedState[:,i], label='stabilizer state')

handles, labels = axfig.get_legend_handles_labels()
axfig.legend(handles, labels)

plt.show()

