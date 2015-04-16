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

stab.comRef.value=(0.00965, 0.0, 0.80771)
stab.waistOriRef.value=(0,)*3
stab.flexOriRef.value=(0,)*3
stab.comDotRef.value=(0,)*3
stab.waistVelRef.value=(0,)*3
stab.flexAngVelRef.value=(0,)*3

gain = GainAdaptive('gain'+stab.name)
plug(gain.gain, stab.controlGain)
plug(stab.error, gain.error) 

Qdiag=20000*np.diag((1,1,1,1,1,1,1,1,1,1,1,1,1,1))
Rdiag=1*np.diag((1,1,1,1,1))

Q11=np.zeros((3,3))
Q12=np.zeros((3,2)) # Cl Omegach
Q13=np.mat('[0,0;0,0;0,0]') #np.zeros((3,2)) ### Cl Omega
Q14=np.zeros((3,3)) # Cl dCl
Q15=np.zeros((3,2)) # Cl dOmegach
Q16=np.zeros((3,2)) ## Cl dOmega
Q22=np.zeros((2,2))
Q23=np.mat('[0,0;0,0]') #np.zeros((2,2)) ### Omegach Omega
Q24=np.zeros((2,3)) # Omegach dCl
Q25=np.zeros((2,2)) # Omegach dOmegach
Q26=np.zeros((2,2)) ## Omegach dOmega
Q33=np.zeros((2,2))
Q34=np.zeros((2,3)) ### Omega dCl
Q35=np.zeros((2,2)) ### Omega dOmegach
Q36=np.zeros((2,2)) ### Omega dOmega
Q44=np.zeros((3,3))
Q45=np.zeros((3,2)) # dCl dOmegach
Q46=np.zeros((3,2)) ## dCl dOmega
Q55=np.zeros((2,2))
Q56=np.mat('[0,0;0,0]') #np.zeros((2,2)) ## dOmegach dOmega
Q66=np.zeros((2,2))
Qcoupl=np.bmat(([[Q11,Q12,Q13,Q14,Q15,Q16],[np.transpose(Q12),Q22,Q23,Q24,Q25,Q26],[np.transpose(Q13),np.transpose(Q23),Q33,Q34,Q35,Q36],[np.transpose(Q14),np.transpose(Q24),np.transpose(Q34),Q44,Q45,Q46],[np.transpose(Q15),np.transpose(Q25),np.transpose(Q35),np.transpose(Q45),Q55,Q56],[np.transpose(Q16),np.transpose(Q26),np.transpose(Q36),np.transpose(Q46),np.transpose(Q56),Q66]]))

Q=Qdiag+Qcoupl
R=Rdiag
stab.setStateCost(matrixToTuple(Q))
stab.setInputCost(matrixToTuple(R))

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

stepTime = 5
simuTime =20
dt = 0.005

logState = np.array([])
logState.resize(simuTime/dt,25)
logControl = np.array([])
logControl.resize(simuTime/dt,6)
logFlexAcc = np.array([])
logFlexAcc.resize(simuTime/dt,7)

for i in range(int(0/dt),int(stepTime/dt)):
   stab.task.recompute(i)
   model.incr(dt)
   print 'boucle 1:'+str(i)
   logState[i,0] = i
   logState[i,1:] = model.state.value
   logControl[i,0] = i
   logControl[i,1:] = model.control.value
   logFlexAcc[i,0]=i
   logFlexAcc[i,1:4]=model.flexAngAccVect.value
   logFlexAcc[i,4:7]=model.flexLinAcc.value

stab.comRef.value=(0.05, 0.0, 0.80771)

for i in range(int(stepTime/dt),int(simuTime/dt)):
   stab.task.recompute(i)
   model.incr(dt)
   print 'boucle 2:'+str(i)
   logState[i,0] = i
   logState[i,1:] = model.state.value
   logControl[i,0] = i
   logControl[i,1:] = model.control.value
   logFlexAcc[i,0]=i
   logFlexAcc[i,1:4]=model.flexAngAccVect.value
   logFlexAcc[i,4:7]=model.flexLinAcc.value


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

fig = plt.figure(); 

axfig = fig.add_subplot(121)
axfig.plot(logFlexAcc[:,0], logFlexAcc[:,1], label='ddOmega X')
axfig.plot(logFlexAcc[:,0], logFlexAcc[:,2], label='ddOmega Y')
axfig.plot(logFlexAcc[:,0], logFlexAcc[:,3], label='ddOmega Z')

axfig = fig.add_subplot(122)
axfig.plot(logFlexAcc[:,0], logFlexAcc[:,4], label='ddt X')
axfig.plot(logFlexAcc[:,0], logFlexAcc[:,5], label='ddt Y')
axfig.plot(logFlexAcc[:,0], logFlexAcc[:,6], label='ddt Z')

handles, labels = axfig.get_legend_handles_labels()
axfig.legend(handles, labels)

plt.show()

