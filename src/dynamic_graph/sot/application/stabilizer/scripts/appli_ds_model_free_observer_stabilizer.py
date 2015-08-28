###
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose 
from dynamic_graph.sot.application.state_observation import DGIMUModelFreeFlexEstimation
from dynamic_graph.sot.application.stabilizer.scenarii.ds_model_free_observer_stabilizer_hrp2 import DSModelFreeObserverStabilizerHRP2
from dynamic_graph.sot.core.matrix_util import matrixToTuple


appli =  DSModelFreeObserverStabilizerHRP2(robot, False)
appli.withTraces()

est = appli.taskCoMStabilized.estimator
stabilizer = appli.taskCoMStabilized

stabilizer.comdotRef.value= (0,0,0)
stabilizer.comddotRef.value= (0,0,0)

appli.robot.addTrace( est.name,'flexibility' )
appli.robot.addTrace( est.name,'flexInverseVelocityVector' )
appli.robot.addTrace( est.name,'flexMatrixInverse' )
appli.robot.addTrace( est.name,'input')
appli.robot.addTrace( est.name,'measurement')
appli.robot.addTrace( est.name,'simulatedSensors' )

appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name,  'gyrometer')

appli.startTracer()

appli.gains['trunk'].setConstant(2)




stabilizer.setStateCostLat(matrixToTuple(np.diag((1e5,1,1e1,1,1e1,10))))
stabilizer.setStateCost1(matrixToTuple(np.diag((1e4,1,1e2,1,1e2,1))))
stabilizer.setStateCost2(matrixToTuple(np.diag((1e6,1,1e2,1,1e2,100))))
stabilizer.setZmpMode(False)


est.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-4,)*6)))
est.setVirtualMeasurementsCovariance(1e-10)


appli.nextStep()

