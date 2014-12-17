###
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose 
from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator import HRP2ModelBaseFlexEstimator
from dynamic_graph.sot.application.stabilizer.scenarii.seqplay_stabilizer_hrp2 import SeqPlayStabilizerHRP2
from dynamic_graph.sot.application.stabilizer import VectorPerturbationsGenerator
from dynamic_graph.sot.core.matrix_util import matrixToTuple 
from dynamic_graph.sot.core import Multiply_matrixHomo_vector
from dynamic_graph.sot.dynamics.zmp_from_forces import ZmpFromForces
from dynamic_graph.sot.application.state_observation import PositionStateReconstructor, MovingFrameTransformation

forceSeqplay = True
#traj = '/home/mbenalle/devel/ros/install/resources/seqplay/walkfwd-resampled'
#traj = '/home/mbenalle/devel/ros/install/resources/seqplay/walkfwd-resampled-30'
#traj = '/home/mbenalle/devel/ros/install/resources/seqplay/stand-on-left-foot'
traj = '/home/mbenalle/devel/ros/install/resources/seqplay/walkfwd-shifted'
#traj = '/home/mbenalle/devel/ros/install/resources/seqplay/stand-on-left-foot-shifted' 
#traj = '/home/mbenalle/devel/ros/install/resources/seqplay/onspot16s'; forceSeqplay = False
#traj = '/home/mbenalle/devel/ros/install/resources/seqplay/onspot'; forceSeqplay = False
#traj = '/home/mbenalle/devel/ros/install/resources/seqplay/walkstraight10cmperstep'; forceSeqplay = False
trunkCompensate = True
handsTask = False
postureTask = True

appli =  SeqPlayStabilizerHRP2(robot, traj , trunkCompensate, handsTask, postureTask,forceSeqplay)
appli.withTraces()

est1 = est = appli.taskCoMStabilized.estimator
stabilizer = appli.taskCoMStabilized

seq = appli.seq



appli.robot.addTrace( stabilizer.name,'com' )
appli.robot.addTrace( stabilizer.name,'comRef' )
appli.robot.addTrace( stabilizer.name,'task' )
appli.robot.addTrace( stabilizer.name,'nbSupport' )
appli.robot.addTrace( stabilizer.name,'error' )
appli.robot.addTrace( stabilizer.name,'comddot' )
appli.robot.addTrace( stabilizer.name,'comddotRefOUT' )
appli.robot.addTrace( stabilizer.name,'debug' )

appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name, 'gyrometer')


appli.robot.addTrace( est.name,'flexibility' )
appli.robot.addTrace( est.name,'flexInversePoseThetaU' )
appli.robot.addTrace( est.name,'flexInverseOmega')
appli.robot.addTrace( est.name,'flexInverseVelocityVector' )
appli.robot.addTrace( est.name,'flexMatrixInverse' )
appli.robot.addTrace( est.name,'input')
appli.robot.addTrace( est.name,'measurement')
appli.robot.addTrace( est.name,'simulatedSensors' )

appli.robot.addTrace( appli.leftAnkle.name , 'position')
appli.robot.addTrace( appli.rightAnkle.name , 'position')

appli.robot.addTrace( seq.name, 'leftAnkle')
appli.robot.addTrace( seq.name, 'rightAnkle')
appli.robot.addTrace( seq.name, 'com')
appli.robot.addTrace( seq.name, 'comdot')
appli.robot.addTrace( seq.name, 'comddot')
appli.robot.addTrace( seq.name, 'leftAnkleVel')
appli.robot.addTrace( seq.name, 'rightAnkleVel')

appli.robot.addTrace( appli.features['waist'].name, 'position')

appli.startTracer()

appli.gains['trunk'].setConstant(10)

est.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-2,)*3+(1e-6,)*3)))

zmp = ZmpFromForces('zmp')
plug (robot.device.forceLLEG , zmp.force_0)
plug (robot.device.forceRLEG, zmp.force_1)
plug (robot.frames['leftFootForceSensor'].position , zmp.sensorPosition_0)
plug (robot.frames['rightFootForceSensor'].position, zmp.sensorPosition_1)

appli.robot.addTrace( zmp.name, 'zmp')
appli.robot.addTrace( appli.zmpRef.name, 'zmp')

realcom = MovingFrameTransformation('comreal')
realcom.setPointMode(True)
plug(est.flexTransformationMatrix, realcom.gMl)
plug(robot.dynamic.com, realcom.lP0)
plug(est.flexVelocityVector, realcom.gVl)
plug(stabilizer.comdot, realcom.lV0)
plug(est.flexAccelerationVector, realcom.gAl)
plug(stabilizer.comddot, realcom.lA0)

appli.robot.addTrace( robot.dynamic.name, 'com')
appli.robot.addTrace( realcom.name, 'gP0')
appli.robot.addTrace( realcom.name, 'gA0')

#stabilizer.setKth(395)



#robot

stabilizer.setHorizon(250)

stabilizer.setStateCostLat(matrixToTuple(np.diag((1e5,1,1e1,1,1e1,10))))
stabilizer.setStateCost1(matrixToTuple(np.diag((1e4,1,1e2,1,1e2,1))))
stabilizer.setStateCost2(matrixToTuple(np.diag((1e6,1,1e2,1,1e2,100))))
stabilizer.setZmpMode(False)

stabilizer.setStateCost1(matrixToTuple(np.diag((1e5,1,1e2,1,1e2,1))))
stabilizer.setStateCost2(matrixToTuple(np.diag((1e6,1,1e1,1,1e1,100))))
stabilizer.setStateCostLat(matrixToTuple(np.diag((1e7,1,1,1,1,100))))

#high gains
stabilizer.setStateCostLat(matrixToTuple(np.diag((1e7,1,1,1,1e3,100))))
stabilizer.setStateCost1(matrixToTuple(np.diag((1e5,1,1e2,1,1e4,10))))

#low gains
#stabilizer.setStateCost1(matrixToTuple(np.diag((1e2,1,1e2,1,1e4,10))))
#stabilizer.setStateCostLat(matrixToTuple(np.diag((1e3,1,1,1,1e6,1e4))))


#stabilizer.setStateCostLat(matrixToTuple(np.diag((1e5,1,1e0,1,1e5,100))))
#stabilizer.setFixedGains(True)

#stabilizer.stateFlexDDot.value = (0,0,0)

#q=[1e8 1 1e2 1 1e2 2000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]'; lqr(q)
#appli.runSeqplay()

appli.nextStep()
