from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
from dynamic_graph.sot.core.meta_tasks import *
from numpy import *
from numpy.linalg import inv,pinv,norm
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate,matrixToRPY
import dynamic_graph.sot.core.matrix_util 
rpyToMatrix = RPYToMatrix
#from dynamic_graph.sot.core.meta_tasks_relative import gotoNdRel, MetaTask6dRel
from dynamic_graph.sot.core.meta_task_posture import MetaTaskPosture
from dynamic_graph.sot.core.feature_vector3 import FeatureVector3
from dynamic_graph.tracer_real_time import *

import dynamic_graph.sot.application.state_observation as sotso
from dynamic_graph.sot.application.velocity.precomputed_tasks import Application

toList = lambda sot: map(lambda x: x[2:],sot.display().split('\n')[3:-2])

def change6dPositionReference(task,feature,gain,position,selec=None,ingain=None,resetJacobian=True):
    M=generic6dReference(position)
    if selec!=None:
        if isinstance(selec,str):  feature.selec.value = selec
        else: feature.selec.value = toFlags(selec)
    feature.reference.value = matrixToTuple(M)
    if gain!=None:  setGain(gain,ingain)
    if 'resetJacobianDerivative' in task.__class__.__dict__.keys() and resetJacobian:
        task.resetJacobianDerivative()
        
def createTrunkTask (robot, application, taskName, ingain = 1.):
    task = Task (taskName)
    task.add (application.features['chest'].name)
    task.add (application.features['waist'].name)
    task.add (application.features['gaze'].name)
    gain = GainAdaptive('gain'+taskName)
    gain.setConstant(ingain)
    plug(gain.gain, task.controlGain)
    plug(task.error, gain.error)      
    return (task, gain)

class HandCompensaterChicken(Application):

    threePhaseScrew = True
    tracesRealTime = True

    def __init__(self,robot):
        Application.__init__(self,robot)

        self.robot = robot
        plug(self.robot.dynamic.com,self.robot.device.zmp)
        
        self.sot = self.solver.sot

        self.createTasks()
        self.initTasks()
        self.initTaskGains()

        self.initialStack()

    # --- TASKS --------------------------------------------------------------------
    # --- TASKS --------------------------------------------------------------------
    # --- TASKS --------------------------------------------------------------------
    def createTasks(self):
	(self.tasks['trunk'],self.gains['trunk'])= createTrunkTask (self.robot, self, 'Tasktrunk')
        self.taskbalance = self.tasks['balance']
        self.taskRH      = self.tasks['right-wrist']
        self.taskLH      = self.tasks['left-wrist']
        self.taskPosture = self.tasks['posture']
        self.taskTrunk   = self.tasks['trunk']
        self.taskHalfStitting = MetaTaskPosture(self.robot.dynamic,'halfsitting')
	
        

    #initialization is separated from the creation of the tasks because if we want to switch
    #to second order controlm the initialization will remain while the creation is 
    #changed

    def initTasks(self):
        self.initTaskBalance()
        self.initTaskPosture()
        #self.initTaskHalfSitting()
        self.initTaskCompensate()

    #def initTaskHalfSitting(self):
    #    self.taskHalfStitting.gotoq(None,self.robot.halfSitting)

    def initTaskBalance(self):
        # --- BALANCE ---
        self.features['chest'].frame('desired')
        self.features['waist'].frame('desired')
        self.features['gaze'].frame('desired')
        #self.taskChest.feature.selec.value = '111111'
        self.features['chest'].selec.value = '111000'
        self.features['waist'].selec.value = '111100'
        self.features['gaze'].selec.value = '111000'
        self.featureCom.selec.value = '011'

    def initTaskPosture(self):
        # --- LEAST NORM
        weight_ff        = 0
        weight_leg       = 3
        weight_knee      = 5
        weight_chest     = 1
        weight_chesttilt = 10
        weight_head      = 0.3
        weight_arm       = 1

        weight = diag( (weight_ff,)*6 + (weight_leg,)*12 + (weight_chest,)*2 + (weight_head,)*2 + (weight_arm,)*14)
        weight[9,9] = weight_knee
        weight[15,15] = weight_knee
        weight[19,19] = weight_chesttilt
        #weight = weight[6:,:]

        self.featurePosture.jacobianIN.value = matrixToTuple(weight)
        self.featurePostureDes.errorIN.value = self.robot.halfSitting
        mask = '1'*36
        # mask = 6*'0'+12*'0'+4*'1'+14*'0'
        # mask = '000000000000111100000000000000000000000000'
        # robot.dynamic.displaySignals ()
        # robot.dynamic.Jchest.value
        self.features['posture'].selec.value = mask

    def initTaskGains(self, setup = "medium"):
        if setup == "medium":
            self.gains['balance'].setConstant(10)
            self.gains['trunk'].setConstant(10)
            self.gains['right-wrist'].setByPoint(4,0.2,0.01,0.8)
            self.gains['left-wrist'].setByPoint(4,0.2,0.01,0.8)
            self.taskHalfStitting.gain.setByPoint(2,0.2,0.01,0.8)
         
    # --- SOLVER ----------------------------------------------------------------

    def push(self,task,feature=None,keep=False):
        if isinstance(task,str): taskName=task
        elif "task" in task.__dict__:  taskName=task.task.name
        else: taskName=task.name
        if taskName not in toList(self.sot):
            self.sot.push(taskName)
        if taskName!="posture" and "posture" in toList(self.sot):
            self.sot.down("posture")
        if keep: feature.keep()

    def rm(self,task):
        if isinstance(task,str): taskName=task
        elif "task" in task.__dict__:  taskName=task.task.name
        else: taskName=task.name
        if taskName in toList(self.sot): self.sot.remove(taskName)

    # --- DISPLAY -------------------------------------------------------------
    def initDisplay(self):
        self.robot.device.viewer.updateElementConfig('red',[0,0,-1,0,0,0])
        self.robot.device.viewer.updateElementConfig('yellow',[0,0,-1,0,0,0])

    def updateDisplay(self):
        '''Display the various objects corresponding to the calcul graph. '''
        None

    # --- TRACES -----------------------------------------------------------
    def withTraces(self):
        if self.tracesRealTime:
            self.robot.tracerSize = 2**26
            self.robot.initializeTracer()
        else:
            self.robot.tracer = Tracer('trace')
            self.robot.device.after.addSignal('{0}.triger'.format(self.robot.tracer.name))
        self.robot.tracer.open('/tmp/','','.dat')
        #self.robot.tracer.add( self.taskRH.task.name+'.error','erh' )
        
    def stopTracer(self):
        self.robot.stopTracer()

    def dumpTracer(self):
        self.robot.tracer.dump()
    
    def startTracer(self):
        self.robot.startTracer()

    # --- RUN --------------------------------------------------------------
    def initialStack(self):
        self.sot.clear()
        self.push(self.tasks['balance'])
        self.push(self.taskTrunk)
        #self.push(self.taskPosture)

    def moveToInit(self):
        '''Go to initial pose.'''
        #gotoNd(self.taskRH,(0.3,-0.2,1.1,0,-pi/2,0),'111001')
        #gotoNd(self.taskLH,(0.3,0.2,1.1,0,-pi/2,0),'111001')
        change6dPositionReference(self.taskRH,self.features['right-wrist'],\
                                    self.gains['right-wrist'],\
                                    (0.3,-0.2,1.1,0,-pi/2,0),'111111')
        self.push(self.taskRH)
        change6dPositionReference(self.taskLH,self.features['left-wrist'],\
                                    self.gains['left-wrist'],\
                                     (0.3,0.2,1.00,0,-pi/2,0),'111111')
        self.push(self.taskLH)
        None

    def moveToInit2(self):
        '''Go to initial pose 2.'''
        #gotoNd(self.taskRH,(0.3,-0.2,1.1,0,-pi/2,0),'111001')
        #gotoNd(self.taskLH,(0.3,0.2,1.1,0,-pi/2,0),'111001')

        change6dPositionReference(self.taskRH,self.features['right-wrist'],\
                                     self.gains['right-wrist'],\
                                     (0.3,-0.10,1.15,pi*0.1,-pi/2,pi*0.2),'111111')

        change6dPositionReference(self.taskLH,self.features['left-wrist'],\
                                     self.gains['left-wrist'],\
                                     (0.25,0.1,0.95,0,-pi/2,-pi*0.2),'111111')

        None

    def goHalfSitting(self):
        '''End of application, go to final pose.'''
        self.featurePostureDes.errorIN.value = self.robot.halfSitting
        self.sot.clear()
        self.push(self.tasks['balance'])
        self.push(self.taskPosture)

    # --- SEQUENCER ---
    seqstep = 0
    def nextStep(self,step=None):
        if step!=None: self.seqstep = step
        if self.seqstep==0:
            self.moveToInit()
        elif self.seqstep==1:
            self.moveToInit2()
        elif self.seqstep==2:
            self.startCompensate()
        elif self.seqstep==3:
            self.goHalfSitting()
        self.seqstep += 1
        
    def __add__(self,i):
        self.nextStep()


    # COMPENATION ######################################

    def initTaskCompensate(self):
        # The constraint is:
        #    cMhref !!=!! cMh = cMcc ccMh
        # or written in ccMh
        #    ccMh !!=!! ccMc cMhref

        # c : central frame of the robot
        # cc : central frame for the controller  (without the flexibility)
        # cMcc= flexibility
        # ccMc= flexibility inverted

        self.transformerR = sotso.MovingFrameTransformation('tranformation_right')

        self.ccMc = self.transformerR.gMl # inverted flexibility
        self.cMrhref = self.transformerR.lM0 # reference position in the world control frame
        # You need to set up the inverted flexibility : plug( ..., self.ccMc)
        # You need to set up a reference value here: plug( ... ,self.cMhref)

        self.ccVc = self.transformerR.gVl # inverted flexibility velocity
        self.cVrhref = self.transformerR.lV0 # reference velocity in the world control frame
        # You need to set up the inverted flexibility velocity : plug( ..., self.ccVc)
        # You need to set up a reference velocity value here: plug( ... ,self.cVhref)

        self.ccMrhref = self.transformerR.gM0 # reference matrix homo in the control frame
        self.ccVrhref = self.transformerR.gV0
        
        ######

        self.cMrhref.value = (matrixToTuple(diag([1,1,1,1])))
        self.cVrhref.value = (0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0)

    def startCompensate(self):


        '''Start to compensate for the hand movements.'''
        self.cMrhref.value = self.robot.dynamic.signal('right-wrist').value
        self.cVrhref.value = (0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0)
        
        plug(self.ccMrhref,self.features['right-wrist'].reference)
        plug(self.ccVrhref,self.features['right-wrist'].velocity)
        
        self.gains['right-wrist'].setByPoint(4,0.2,0.01,0.8)        
        self.tasks['right-wrist'].setWithDerivative (True)
        self.features['right-wrist'].frame('desired')


        print matrix(self.ccMc.value)


        #######

        #self.cMlhref.value = self.robot.dynamic.lh.value
        #self.cVlhref.value = (0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0)
        #print matrix(self.cMlhref.value)

        ######
