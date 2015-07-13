###
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose, Multiply_matrixHomo_vector
from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator import HRP2ModelBaseFlexEstimator 
from dynamic_graph.sot.application.stabilizer.scenarii.seqplay_lqr_twoDof_coupled_stabilizer_hrp2 import SeqPlayLqrTwoDofCoupledStabilizerHRP2
from dynamic_graph.sot.application.stabilizer import VectorPerturbationsGenerator
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.dynamics.zmp_from_forces import ZmpFromForces

forceSeqplay = True
#traj = '/home/alexis/devel/ros/install/resources/seqplay/walkfwd-resampled'
#traj = '/home/alexis/devel/ros/install/resources/seqplay/walkfwd-resampled-30'
traj = '/home/alexis/devel/ros/install/resources/seqplay/stand-on-left-foot'
#traj = '/home/alexis/devel/ros/install/resources/seqplay/walkfwd-shifted'
#traj = '/home/alexis/devel/ros/install/resources/seqplay/stand-on-left-foot-shifted' 
#traj = '/home/alexis/devel/ros/install/resources/seqplay/onspot16s'; forceSeqplay = False
#traj = '/home/alexis/devel/ros/install/resources/seqplay/onspot'; forceSeqplay = False

appli =  SeqPlayLqrTwoDofCoupledStabilizerHRP2(robot, traj, False, False, False,forceSeqplay)
appli.withTraces()

est = appli.taskCoMStabilized.estimator
stabilizer = appli.taskCoMStabilized
seq = appli.seq

kfe=40000
kfv=600
kte=600
ktv=60

kfe=600;
kfv=65;
kte=60;
ktv=6.5;

stabilizer.setkts(kte)
stabilizer.setktd(ktv)
stabilizer.setkfs(kfe)
stabilizer.setkfd(kfv)

plug(robot.device.velocity,robot.dynamic.velocity)

#appli.comRef.value=(0.0105, 0.0002, 0.80771000000000004) # two feet Mx=My=0
appli.comRef.value=(0.01068, 0.00014, 0.80771000000000004) # two feet Mx=My=0
# appli.comRef.value=(0.0111, 0.0, 0.80771) # two feet AngFlex=0 without re-calibration
#appli.comRef.value=(0.00202, 0.096025, 0.80769999999999997) # one foot (include in the seqplay)
#appli.comRef.value=(0.00182, 0.09582, 0.80769999999999997) # one foot Mx=My=0

realcom = Multiply_matrixHomo_vector('real-com')
plug(est.flexTransformationMatrix, realcom.sin1)
plug(appli.com, realcom.sin2)

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
appli.robot.addTrace( stabilizer.name,'task' )
appli.robot.addTrace( stabilizer.name,'nbSupport' )
appli.robot.addTrace( stabilizer.name,'error' )
appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name, 'gyrometer')
appli.robot.addTrace( realcom.name, 'sout')
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

# Perturbation Generator on task
perturbatorTask = VectorPerturbationsGenerator('perturbatedTask')
perturbatorTask.setSinLessMode(True)
vect = perturbatorTask.sin
vect.value = appli.comRef.value
plug (perturbatorTask.sout,stabilizer.perturbationVel)
appli.robot.addTrace( perturbatorTask.name, 'sout')
perturbatorTask.perturbation.value=(0,1,0)
perturbatorTask.selec.value = '111'
perturbatorTask.setMode(4)
perturbatorTask.setPeriod(0)
perturbatorTask.activate(False)

appli.startTracer()

zmp = ZmpFromForces('zmp')
plug (robot.device.forceLLEG , zmp.force_0)
plug (robot.device.forceRLEG, zmp.force_1)
plug (robot.frames['leftFootForceSensor'].position , zmp.sensorPosition_0)
plug (robot.frames['rightFootForceSensor'].position, zmp.sensorPosition_1)
appli.robot.addTrace( zmp.name, 'zmp')
appli.robot.addTrace( appli.zmpRef.name, 'zmp')

appli.gains['trunk'].setConstant(2)
est.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-6,)*6)))

stabilizer.setFixedGains(True)
stabilizer.setHorizon(200)

est.calibration.start(500)
est.flexThetaU.value

appli.nextStep(1)
appli.comRef.value=(0.0, 0.0, 0.80771000000000004) # two feet Mx=My=0
stabilizer.setStateCost(matrixToTuple(1*np.diag((20000,20000,20000,20000,20000,1,1,1,1,1,1,1,1,1))))
stabilizer.setInputCost(matrixToTuple(1*np.diag((1,1,1,1,1))))

stabilizer.start()

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
stab.setInputCost(matrixToTuple(1*np.diag((1,1,1,1,1))))

stabilizer.setStateCost(matrixToTuple(1*np.diag((20000,20000,20000,20000,20000,1,1,1,1,1,1,1,1,1))))
stabilizer.setInputCost(matrixToTuple(1*np.diag((1,1,1,1,1))))

stabilizer.start()

est.calibration.start(500)
est.flexThetaU.value

stabilizer.setStateCost(matrixToTuple(1*np.diag((5000,5000,5000,1,1,5000,5000,1,1,1,1,1,1,1))))
stabilizer.setInputCost(matrixToTuple(1*np.diag((1,5000,1,1,1))))

perturbator = VectorPerturbationsGenerator('perturbation')
perturbator.setSinLessMode(True)
vect = perturbator.sin
vect.value = stabilizer.comRef.value
plug (perturbator.sout,stabilizer.perturbationVel)
perturbator.perturbation.value=(10,0,0)
perturbator.selec.value = '111'
perturbator.setMode(0)
perturbator.setPeriod(0)
perturbator.activate(False)

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

# To test CalibrateImu
from dynamic_graph.sot.application.state_observation import CalibrateImu
from dynamic_graph.sot.core import Stack_of_vector
sensorStack = Stack_of_vector ('Sensors')
plug(robot.device.accelerometer,sensorStack.sin1)
plug(robot.device.gyrometer,sensorStack.sin2)
sensorStack.selec1(0, 3)
sensorStack.selec2(0, 3)
calibration= CalibrateImu('calibration')
plug(sensorStack.sout,calibration.imuIn)
appli.robot.addTrace (calibration.name,'imuOut')

# Scenario for generating data to identificate the model
appli.waist.error.value=(0.2,0,0)
time.sleep(5)
appli.waist.error.value=(-0.2,0,0)
time.sleep(5)
appli.waist.error.value=(0,0,0)
time.sleep(10)

appli.waist.error.value=(0,0.2,0)
time.sleep(5)
appli.waist.error.value=(0,-0.2,0)
time.sleep(5)
appli.waist.error.value=(0,0,0)
time.sleep(10)

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

Qdiag=1*np.diag((20000,20000,20000,100,100,1000,1000,1,1,1,1,1,1,1))
Q=Qdiag+Qcoupl
stabilizer.setStateCost(matrixToTuple(Q))

Qdiag=1*np.diag((10000,10000,10000,100,100,10000,10000,1,1,1,1,1,1,1))
Q=Qdiag+Qcoupl
stabilizer.setStateCost(matrixToTuple(Q))

Qdiag=1*np.diag((10000,10000,10000,10000,10000,1,1,1,1,1,1,1,1,1))
Q=Qdiag+Qcoupl
stabilizer.setStateCost(matrixToTuple(Q))

Q=1*np.diag((10000,10000,10000,10000,10000,1,1,1,1,1,1,1,1,1))
stabilizer.setStateCost(matrixToTuple(Q))

Q=1*np.diag((20000,20000,20000,20000,20000,1,1,1,1,1,1,1,1,1))
stabilizer.setStateCost(matrixToTuple(Q))

R=1*np.diag((1,1,1,1,1))
stabilizer.setInputCost(matrixToTuple(R))


Q=1*np.diag((20000,20000,20000,1,10,20000,20000,1,1,1,1,1,1,1))
stabilizer.setStateCost(matrixToTuple(Q))

R=1*np.diag((1,20000,1,1,1))
stabilizer.setInputCost(matrixToTuple(R))

stabilizer.setStateCost(matrixToTuple(1*np.diag((10000,10000,10000,10000,10000,1,1,1,1,1,1,1,1,1))))
stabilizer.setInputCost(matrixToTuple(1*np.diag((1,1,1,1,1))))

stabilizer.setStateCost(matrixToTuple(1*np.diag((100,1000,1000,1000,100,1,1000,1,1,1,1,1,1,100))))
perturbator = VectorPerturbationsGenerator('perturbation')
perturbator.setSinLessMode(True)
vect = perturbator.sin
vect.value = stab.comRef.value
plug (perturbator.sout,stab.perturbationAcc)
perturbator.perturbation.value=(0,10,0)
perturbator.selec.value = '111'
perturbator.setMode(0)
perturbator.setPeriod(1500)
perturbator.activate(True)

stabilizer.setStateCost(matrixToTuple(1*np.diag((100,100,100,100,100,10000,10000,1000,1000,1000,100,100,1000,1000))))
stabilizer.setInputCost(matrixToTuple(1*np.diag((1,1,1,1,1))))
