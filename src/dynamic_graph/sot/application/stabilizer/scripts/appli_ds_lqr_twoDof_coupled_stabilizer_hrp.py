###
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose, Multiply_matrixHomo_vector
from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator import HRP2ModelBaseFlexEstimator 
from dynamic_graph.sot.application.stabilizer.scenarii.ds_lqr_twoDof_coupled_stabilizer_hrp2 import DSLqrTwoDofCoupledStabilizerHRP2
from dynamic_graph.sot.application.stabilizer import VectorPerturbationsGenerator
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.dynamics.zmp_from_forces import ZmpFromForces
import time


appli =  DSLqrTwoDofCoupledStabilizerHRP2(robot, False, False, False)
appli.withTraces()

est = appli.taskCoMStabilized.estimator
stabilizer = appli.taskCoMStabilized

appli.comRef.value=(0.0111, 0.0, 0.80771)
# Flexibility = 0
#stabilizer.stateRef.value=(0.01111, 0.0, 0.80777699999999997, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

perturbator = VectorPerturbationsGenerator('comref')
comRef = perturbator.sin
comRef.value = appli.comRef.value
plug (perturbator.sout,appli.comRef)

realcom = Multiply_matrixHomo_vector('real-com')
plug(est.flexTransformationMatrix, realcom.sin1)
plug(appli.com, realcom.sin2)

zmp = ZmpFromForces('zmp')
plug (robot.device.forceLLEG , zmp.force_0)
plug (robot.device.forceRLEG, zmp.force_1)
plug (robot.frames['leftFootForceSensor'].position , zmp.sensorPosition_0)
plug (robot.frames['rightFootForceSensor'].position, zmp.sensorPosition_1)

appli.robot.addTrace( est.name,'flexibility' )
appli.robot.addTrace( est.name,'flexThetaU' )
appli.robot.addTrace( est.name,'flexInversePoseThetaU' )
appli.robot.addTrace( est.name,'flexInverseOmega')
appli.robot.addTrace( est.name,'flexInverseVelocityVector' )
appli.robot.addTrace( est.name,'flexMatrixInverse' )
appli.robot.addTrace( est.name,'input')
appli.robot.addTrace( est.name,'measurement')
appli.robot.addTrace( est.name,'simulatedSensors' )
a1*np.diag((20000,20000,20000,1,10,20000,20000,1,1,1,1,1,1,1))ppli.robot.addTrace( stabilizer.name,'task' )
appli.robot.addTrace( stabilizer.name,'nbSupport' )
appli.robot.addTrace( stabilizer.name,'error' )
appli.robot.addTrace( perturbator.name, 'sout')
appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name, 'gyrometer')
appli.robot.addTrace( realcom.name, 'sout')
appli.robot.addTrace ( stabilizer.name,'state')
appli.robot.addTrace ( stabilizer.name,'stateRef')
appli.robot.addTrace ( stabilizer.name,'stateSimulation')
appli.robot.addTrace ( stabilizer.name,'stateModelError')
appli.robot.addTrace ( stabilizer.name,'stateError')
appli.robot.addTrace ( stabilizer.name,'stateExtended')
appli.robot.addTrace ( stabilizer.name,'control')
appli.robot.addTrace (robot.device.name,'velocity')
appli.robot.addTrace (robot.dynamic.name,'waist')
appli.robot.addTrace (stabilizer.name,'inertiaOut')
appli.robot.addTrace (stabilizer.name,'gain')
appli.robot.addTrace (stabilizer.name,'Amatrix')
appli.robot.addTrace (stabilizer.name,'Bmatrix')
appli.robot.addTrace (stabilizer.name,'supportPos1')
appli.robot.addTrace (stabilizer.name,'supportPos2')
appli.robot.addTrace( zmp.name, 'zmp')

appli.startTracer()

appli.gains['trunk'].setConstant(2)

est.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-6,)*6)))

stabilizer.setFixedGains(True)
stabilizer.setHorizon(200)

est.setKfe(matrixToTuple(np.diag((40000,40000,40000))))
est.setKfv(matrixToTuple(np.diag((600,600,600))))
est.setKte(matrixToTuple(np.diag((600,600,600))))
est.setKtv(matrixToTuple(np.diag((60,60,60))))

Q=1*np.diag((20000,20000,20000,1,10,20000,20000,1,1,1,1,1,1,1))
R=1*np.diag((1,1,1,1,1))

Q11=np.zeros((3,3))
Q12=np.zeros((3,2)) # Cl Omegach
Q13=np.mat('[0,10000;10000,0;0,0]') #np.zeros((3,2)) ### Cl Omega
Q14=np.zeros((3,3)) # Cl dCl
Q15=np.zeros((3,2)) # Cl dOmegach
Q16=np.mat('[0,0;0,0;0,0]') #np.zeros((3,2)) ## Cl dOmega

Q22=np.zeros((2,2))
Q23=np.mat('[10000,0;0,10000]') #np.zeros((2,2)) ### Omegach Omega
Q24=np.zeros((2,3)) # Omegach dCl
Q25=np.zeros((2,2)) # Omegach dOmegach
Q26=np.mat('[1,0;0,1]') #np.zeros((2,2)) ## Omegach dOmega

Q33=np.zeros((2,2))
Q34=np.mat('[0,0,0;0,0,0]') #np.mat('[0,1,0;1,0,0]') #np.zeros((2,3)) ### Omega dCl
Q35=np.zeros((2,2)) ### Omega dOmegach
Q36=np.mat('[0,0;0,0]') #np.mat('[1,0;0,1]') #np.zeros((2,2)) ### Omega dOmega

Q44=np.zeros((3,3))
Q45=np.zeros((3,2)) # dCl dOmegach
Q46=np.mat('[0,0;0,0;0,0]') #np.mat('[0,1;1,0;0,0]') #np.zeros((3,2)) ## dCl dOmega

Q55=np.zeros((2,2))
Q56=np.mat('[0,0;0,0]') #np.mat('[1,0;0,1]') #np.zeros((2,2)) ## dOmegach dOmega

Q66=np.zeros((2,2))

Qcoupl=np.bmat(([[Q11,Q12,Q13,Q14,Q15,Q16],[np.transpose(Q12),Q22,Q23,Q24,Q25,Q26],[np.transpose(Q13),np.transpose(Q23),Q33,Q34,Q35,Q36],[np.transpose(Q14),np.transpose(Q24),np.transpose(Q34),Q44,Q45,Q46],[np.transpose(Q15),np.transpose(Q25),np.transpose(Q35),np.transpose(Q45),Q55,Q56],[np.transpose(Q16),np.transpose(Q26),np.transpose(Q36),np.transpose(Q46),np.transpose(Q56),Q66]]))

#Q=Qdiag+Qcoupl
#R=Rdiag
stabilizer.setStateCost(matrixToTuple(1*np.diag((20000,20000,20000,1,10,20000,20000,1,1,1,1,1,1,1))))
stabilizer.setInputCost(matrixToTuple(R))

#stabilizer.start()

perturbator.perturbation.value=(0,0.2,0)
perturbator.selec.value = '111'
perturbator.setMode(0)
perturbator.setPeriod(500)
perturbator.activate(False)

stabilizer.stop()
appli.nextStep()

perturbator.activate(True)
perturbator.perturbation.value=(1,0,0)
time.sleep(50)
perturbator.perturbation.value=(0,0,0)
time.sleep(10)
perturbator.perturbation.value=(-1,0,0)
time.sleep(50)
perturbator.perturbation.value=(0,0,0)
time.sleep(10)
perturbator.perturbation.value=(0,0.2,0)
time.sleep(50)
perturbator.perturbation.value=(0,0,0)
time.sleep(10)
perturbator.perturbation.value=(0,-0.2,0)
time.sleep(50)
perturbator.perturbation.value=(0,0,0)
time.sleep(10)
