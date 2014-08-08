###
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose 
from dynamic_graph.sot.application.state_observation.initializations.hrp2_flexibility_estimator import HRP2FlexibilityEstimator 
from dynamic_graph.sot.application.stabilizer.scenarii.seqplay_stabilizer_hrp2 import SeqPlayStabilizerHRP2
from dynamic_graph.sot.application.stabilizer import VectorPerturbationsGenerator
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.dynamics.zmp_from_forces import ZmpFromForces


#traj = '/home/mbenalle/devel/ros/install/resources/seqplay/walkfwd-resampled'
#traj = '/home/mbenalle/devel/ros/install/resources/seqplay/walkfwd-resampled-30'
traj = '/home/mbenalle/devel/ros/install/resources/seqplay/walkfwd-shifted'
#traj = '/home/mbenalle/devel/ros/install/resources/seqplay/stand-on-left-foot'

appli =  SeqPlayStabilizerHRP2(robot, traj , True, False, True)
appli.withTraces()

est = appli.taskCoMStabilized.estimator
stabilizer = appli.taskCoMStabilized

seq = appli.seq

appli.robot.addTrace( est.name,'flexibility' )
appli.robot.addTrace( est.name,'flexInversePoseThetaU' )
appli.robot.addTrace( est.name,'flexInverseOmega')
appli.robot.addTrace( est.name,'flexInverseVelocityVector' )
appli.robot.addTrace( est.name,'flexMatrixInverse' )
appli.robot.addTrace( est.name,'input')
appli.robot.addTrace( est.name,'measurement')
appli.robot.addTrace( est.name,'simulatedSensors' )

appli.robot.addTrace( stabilizer.name,'com' )
appli.robot.addTrace( stabilizer.name,'comRef' )
appli.robot.addTrace( stabilizer.name,'task' )
appli.robot.addTrace( stabilizer.name,'nbSupport' )
appli.robot.addTrace( stabilizer.name,'error' )
appli.robot.addTrace( stabilizer.name,'d2com' )
appli.robot.addTrace( stabilizer.name,'debug' )

appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name, 'gyrometer')

appli.robot.addTrace( appli.leftAnkle.name , 'position')
appli.robot.addTrace( appli.rightAnkle.name , 'position')

appli.robot.addTrace( seq.name, 'leftAnkle')
appli.robot.addTrace( seq.name, 'rightAnkle')
appli.robot.addTrace( seq.name, 'com')
appli.robot.addTrace( seq.name, 'comdot')
appli.robot.addTrace( seq.name, 'leftAnkleVel')
appli.robot.addTrace( seq.name, 'rightAnkleVel')

appli.robot.addTrace( appli.features['waist'].name, 'position')

appli.startTracer()

appli.gains['trunk'].setConstant(10)

est.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-1,)*6)))
est.setVirtualMeasurementsCovariance(1e-10)

zmp = ZmpFromForces('zmp')
plug (robot.device.forceLLEG , zmp.force_0)
plug (robot.device.forceRLEG, zmp.force_1)
plug (robot.frames['leftFootForceSensor'].position , zmp.sensorPosition_0)
plug (robot.frames['rightFootForceSensor'].position, zmp.sensorPosition_1)

appli.robot.addTrace( appli.zmpRef.name, 'zmp')
appli.robot.addTrace( zmp.name, 'zmp')


#stabilizer.setGainLateral((800, -2000, 300, 0))
#stabilizer.setPoles1((-5,)*4)
#stabilizer.setPoles2((-8,)*4)
#stabilizer.setPolesLateral((-10,)*4)

#stabilizer.start()

#stabilizer.setKth(395)

#stabilizer.setStateCost(matrixToTuple(np.diag((100,10,100,10))))

appli.nextStep()
