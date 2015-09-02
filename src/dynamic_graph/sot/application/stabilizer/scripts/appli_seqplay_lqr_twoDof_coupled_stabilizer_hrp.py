###
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose, Multiply_matrixHomo_vector
from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import HRP2ModelBaseFlexEstimatorIMUForce
from dynamic_graph.sot.application.stabilizer.scenarii.seqplay_lqr_twoDof_coupled_stabilizer_hrp2 import SeqPlayLqrTwoDofCoupledStabilizerHRP2
from dynamic_graph.sot.application.stabilizer import VectorPerturbationsGenerator
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.dynamics.zmp_from_forces import ZmpFromForces

forceSeqplay = True
#traj = '/home/alexis/devel/ros/install/resources/seqplay/stand-on-left-foot'
traj = '/home/alexis/devel/ros/install/resources/seqplay/stand-on-right-foot'

appli =  SeqPlayLqrTwoDofCoupledStabilizerHRP2(robot, traj, False, False, False,forceSeqplay)
appli.withTraces()

est = appli.taskCoMStabilized.estimator
stabilizer = appli.taskCoMStabilized
seq = appli.seq

# Changer raideurs

# Simulation
kfe=40000
kfv=600
kte=600
ktv=60

# Robot
#kfe=40000
#kfv=600
#kte=400
#ktv=10

stabilizer.setkts(kte)
stabilizer.setktd(ktv)
stabilizer.setkfs(kfe)
stabilizer.setkfd(kfv)

plug(robot.device.velocity,robot.dynamic.velocity)

# Simulation
appli.comRef.value=(0.01068, 0.00014, 0.80771000000000004) # two feet Mx=My=0
#appli.comRef.value=(0.00182, 0.09582, 0.80769999999999997) # one foot Mx=My=0

# Robot


appli.robot.addTrace( est.name,'flexibility' )
appli.robot.addTrace( est.name,'flexThetaU' )
appli.robot.addTrace( est.name,'flexInversePoseThetaU' )
appli.robot.addTrace( est.name,'flexInverseOmega')
appli.robot.addTrace( est.name,'flexInverseVelocityVector' )
appli.robot.addTrace( est.name,'flexMatrixInverse' )
appli.robot.addTrace( est.name,'input')
appli.robot.addTrace( est.name,'measurement')
appli.robot.addTrace( est.name,'simulatedSensors' )
appli.robot.addTrace( est.name,  'forcesAndMoments')
appli.robot.addTrace( est.name,  'forcesSupport1')
appli.robot.addTrace( est.name,  'forcesSupport2')
appli.robot.addTrace( stabilizer.name,'task' )
appli.robot.addTrace( stabilizer.name,'nbSupport' )
appli.robot.addTrace( stabilizer.name,'error' )
appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name, 'gyrometer')
appli.robot.addTrace ( stabilizer.name,'state')
appli.robot.addTrace ( stabilizer.name,'stateWorld')
appli.robot.addTrace ( stabilizer.name,'stateRef')
appli.robot.addTrace ( stabilizer.name,'stateError')
appli.robot.addTrace ( stabilizer.name,'stateExtended')
appli.robot.addTrace ( stabilizer.name,'control')
appli.robot.addTrace (robot.device.name,'velocity')
appli.robot.addTrace (robot.dynamic.name,'angularmomentum')
appli.robot.addTrace (robot.dynamic.name,'waist')
appli.robot.addTrace (stabilizer.name,'inertiaOut')
appli.robot.addTrace (stabilizer.name,'gain')
appli.robot.addTrace (stabilizer.name,'Amatrix')
appli.robot.addTrace (stabilizer.name,'Bmatrix')
appli.robot.addTrace (stabilizer.name,'supportPos1')
appli.robot.addTrace (stabilizer.name,'supportPos2')
appli.robot.addTrace (stabilizer.name,'energy')

zmp = ZmpFromForces('zmpReal')
plug (robot.device.forceLLEG , zmp.force_0)
plug (robot.device.forceRLEG, zmp.force_1)
plug (robot.frames['leftFootForceSensor'].position , zmp.sensorPosition_0)
plug (robot.frames['rightFootForceSensor'].position, zmp.sensorPosition_1)
appli.robot.addTrace( zmp.name, 'zmp')
appli.robot.addTrace( appli.zmpRef.name, 'zmp')

#zmpEst = ZmpFromForces('zmpEstimated')
#plug (est.forcesSupport1 , zmpEst.force_0)
#plug (est.forcesSupport2, zmpEst.force_1)
#plug (stabilizer.supportPos1 , zmpEst.sensorPosition_0)
#plug (stabilizer.supportPos2, zmpEst.sensorPosition_1)
#appli.robot.addTrace( zmpEst.name, 'zmp')

appli.startTracer()

# Perturbation Generator on task
perturbatorTask = VectorPerturbationsGenerator('perturbatedTask')
perturbatorTask.setSinLessMode(True)
vect = perturbatorTask.sin
vect.value = appli.comRef.value
plug (perturbatorTask.sout,stabilizer.perturbationVel)
appli.robot.addTrace( perturbatorTask.name, 'sout')
perturbatorTask.perturbation.value=(10,0,0)
perturbatorTask.selec.value = '111'
perturbatorTask.setMode(4)
perturbatorTask.setPeriod(0)
perturbatorTask.activate(False)

appli.gains['trunk'].setConstant(2)
est.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-6,)*6)))
est.setForceVariance(10)

stabilizer.setFixedGains(True)
stabilizer.setHorizon(100)
est.setWithForceSensors(False)

# Trouver Mx=My=0

est.calibration.start(500)
est.flexThetaU.value

#appli.comRef.value=(0.0, 0.0, 0.80771000000000004)
appli.nextStep(1)
stabilizer.setStateCost(matrixToTuple(1*np.diag((20000,20000,20000,20000,20000,1,1,1,1,1,1,1,1,1))))
stabilizer.setInputCost(matrixToTuple(1*np.diag((1,1,1,1,1))))

stabilizer.start()


stabilizer.setStateCost(matrixToTuple(1*np.diag((1000,1,10000,1,1000,1,1000,10,1,1000,1,10,1,10))))

stabilizer.setStateCost(matrixToTuple(1*np.diag((100,100,100,100,100,1000,1000,100,100,100,100,100,1000,1000))))
stabilizer.setInputCost(matrixToTuple(1*np.diag((1,1,1,1,1))))

stabilizer.setStateCost(matrixToTuple(1*np.diag((20000,20000,20000,1,10,20000,20000,1,1,1,1,1,1,1))))
stabilizer.setInputCost(matrixToTuple(1*np.diag((1,20000,1,1,1))))

# Perturbation Generator on task
perturbatorTask = VectorPerturbationsGenerator('perturbatedTask')
perturbatorTask.setSinLessMode(True)
vect = perturbatorTask.sin
vect.value = appli.comRef.value
plug (perturbatorTask.sout,stabilizer.perturbationVel)
appli.robot.addTrace( perturbatorTask.name, 'sout')
perturbatorTask.perturbation.value=(1,0,0)
perturbatorTask.selec.value = '111'
perturbatorTask.setMode(4)
perturbatorTask.setPeriod(0)
perturbatorTask.activate(False)








# Perturbation Generator on comRef
perturbator = VectorPerturbationsGenerator('comref')
comRef = perturbator.sin
comRef.value = appli.comRef.value
plug (perturbator.sout,appli.comRef)
appli.robot.addTrace( perturbator.name, 'sout')
perturbator.perturbation.value=(0,1,0)
perturbator.selec.value = '111'
perturbator.setMode(0)
perturbator.setPeriod(0)
perturbator.activate(False)

stab.setStateCost(matrixToTuple(1*np.diag((100,100,100,100,100,1000,1000,100,100,100,100,100,1000,1000))))

stabilizer.setStateCost(matrixToTuple(1*np.diag((5000,5000,5000,1,1,5000,5000,1,1,1,1,1,1,1))))
stabilizer.setInputCost(matrixToTuple(1*np.diag((1,5000,1,1,1))))

stabilizer.setStateCost(matrixToTuple(1*np.diag((20000,20000,20000,100,100,1000,1000,1,1,1,1,1,1,1))))

stabilizer.setStateCost(matrixToTuple(1*np.diag((10000,10000,10000,100,100,10000,10000,1,1,1,1,1,1,1))))

stabilizer.setStateCost(matrixToTuple(1*np.diag((100,1000,1000,1000,100,1,1000,1,1,1,1,1,1,100))))

stabilizer.setStateCost(matrixToTuple(1*np.diag((100,100,100,100,100,10000,10000,1000,1000,1000,100,100,1000,1000))))



