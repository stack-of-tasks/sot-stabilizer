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

appli.comRef.value=(0.00965, 0.0, 0.80771)
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

stabilizer.setStateCost(matrixToTuple(1*np.diag((20000,20000,20000,20000,20000,1,1,1,1,1,1,1,1,1))))
stabilizer.setInputCost(matrixToTuple(1*np.diag((1,1,1,1,1))))

stabilizer.start()

perturbator.perturbation.value=(1,0,0)
perturbator.selec.value = '111'
perturbator.setMode(0)
perturbator.setPeriod(500)
perturbator.activate(False)

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
