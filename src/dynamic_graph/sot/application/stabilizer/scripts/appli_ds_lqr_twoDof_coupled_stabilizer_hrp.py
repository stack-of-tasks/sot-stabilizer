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

# Changer raideurs

# Simulation
#kfe=40000
#kfv=600
#kte=600
#ktv=60

# Robot
stabilizer.setkts(350)
stabilizer.setktd(10)
stabilizer.setkfs(40000)
stabilizer.setkfd(600)

plug(robot.device.velocity,robot.dynamic.velocity)

# Simulation
appli.comRef.value=(0.01068, 0.00014, 0.80771000000000004) # two feet Mx=My=0

zmp = ZmpFromForces('zmpReal')
plug (robot.device.forceLLEG , zmp.force_0)
plug (robot.device.forceRLEG, zmp.force_1)
plug (robot.frames['leftFootForceSensor'].position , zmp.sensorPosition_0)
plug (robot.frames['rightFootForceSensor'].position, zmp.sensorPosition_1)

zmpEst = ZmpFromForces('zmpEstimated')
plug (est.forcesSupport1 , zmpEst.force_0)
plug (est.forcesSupport2, zmpEst.force_1)
plug (robot.frames['leftFootForceSensor'].position , zmpEst.sensorPosition_0)
plug (robot.frames['rightFootForceSensor'].position, zmpEst.sensorPosition_1)

appli.gains['trunk'].setConstant(2)
est.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-3,)*3+(1e-6,)*3)))
est.setForceVariance(1e-4)

stabilizer.setFixedGains(True)
stabilizer.setHorizon(400)
est.setWithForceSensors(True)

appli.robot.addTrace( est.name,'flexibility' )
appli.robot.addTrace( est.name,'flexThetaU' )
appli.robot.addTrace( est.name,'flexInversePoseThetaU' )
appli.robot.addTrace( est.name,'flexInverseOmega')
appli.robot.addTrace( est.name,'flexInverseVelocityVector' )
appli.robot.addTrace( est.name,'flexMatrixInverse' )
appli.robot.addTrace( est.name,'input')
appli.robot.addTrace( est.name,'measurement')
appli.robot.addTrace( est.name,'simulatedSensors' )
appli.robot.addTrace( stabilizer.name,'task' )
appli.robot.addTrace( stabilizer.name,'nbSupport' )
appli.robot.addTrace( stabilizer.name,'error' )
appli.robot.addTrace( perturbator.name, 'sout')
appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name, 'gyrometer')
appli.robot.addTrace( realcom.name, 'sout')
appli.robot.addTrace ( stabilizer.name,'state')
appli.robot.addTrace ( stabilizer.name,'stateWorld')
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

est.setProcessNoiseCovariance(matrixToTuple(np.diag((1e-8,)*12+(1e-4,)*6+(1.e-15,)*2)))

