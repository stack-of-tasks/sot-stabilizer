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


from dynamic_graph.sot.application.stabilizer import HRP2DecoupledStabilizer
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

def createAnklesTask (robot, application, taskName, ingain = 1.):
    task = Task (taskName)
    task.add (application.leftAnkle.name)
    task.add (application.rightAnkle.name)
    gain = GainAdaptive('gain'+taskName)
    gain.setConstant(ingain)
    plug(gain.gain, task.controlGain)
    plug(task.error, gain.error)      
    return (task, gain)

class DSStabilizer(Application):

    threePhaseScrew = True
    tracesRealTime = True

    def __init__(self,robot,trunkStabilize = True, hands = True, posture=True):
        Application.__init__(self,robot)
        self.hands =hands
        self.posture = posture
        self.trunkStabilize = trunkStabilize
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
        (self.tasks['ankles'],self.gains['ankles'])= createAnklesTask (self.robot, self, 'TaskAnkles')
        self.taskLF      = self.tasks['left-ankle']
        self.taskLF      = self.tasks['right-ankle']
        self.taskCom     = self.tasks['com']
        self.taskRH      = self.tasks['right-wrist']
        self.taskLH      = self.tasks['left-wrist']
        self.taskPosture = self.tasks['posture']
        self.taskTrunk   = self.tasks['trunk']
        self.taskAnkles  = self.tasks['ankles']
        self.taskHalfStitting = MetaTaskPosture(self.robot.dynamic,'halfsitting')
        (self.tasks['com-stabilized'],self.gains['com-stabilized']) = self.createStabilizedCoMTask()
        self.taskCoMStabilized = self.tasks['com-stabilized']

    #initialization is separated from the creation of the tasks because if we want to switch
    #to second order controlm the initialization will remain while the creation is 
    #changed

    def initTasks(self):
        self.initTaskBalance()
        self.initTaskPosture()
        self.initTaskStabilize()


    def initTaskBalance(self):
        # --- BALANCE ---
        self.features['chest'].frame('desired')
        self.features['waist'].frame('desired')
        self.features['gaze'].frame('desired')
        #self.taskChest.feature.selec.value = '111111'
        self.features['chest'].selec.value = '111000'
        self.features['waist'].selec.value = '111000'
        self.features['gaze'].selec.value = '111000'
        self.featureCom.selec.value = '111'

    def initTaskGains(self, setup = "medium"):
        if setup == "medium":
            self.gains['balance'].setConstant(10)
            self.gains['trunk'].setConstant(10)
            self.gains['com-stabilized'].setConstant(10)
            self.gains['ankles'].setConstant(10)
            self.gains['right-wrist'].setByPoint(4,0.2,0.01,0.8)
            self.gains['left-wrist'].setByPoint(4,0.2,0.01,0.8)
            self.taskHalfStitting.gain.setByPoint(2,0.2,0.01,0.8)

    def createStabilizedCoMTask (self):
        raise Exception("createStabilizedCoMTask is not overloaded")
         
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
        if self.hands:
            self.push(self.taskRH)
            self.push(self.taskLH)
        if self.posture:
            self.push(self.taskPosture)
        #self.push(self.taskPosture)

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
            print ('Stabilizer ready')
            self.plugStabilizer()
        elif self.seqstep==1:
            print ('Stabilizer on')
            self.tasks['com-stabilized'].start()
        elif self.seqstep==2:
            print ('Halfsitting')
            self.goHalfSitting()
        self.seqstep += 1
        
    def __add__(self,i):
        self.nextStep()

    # Stabilization ######################################

    def initTaskStabilize(self):
        from dynamic_graph.sot.application.state_observation import MovingFrameTransformation

        if self.trunkStabilize:
            ### waist           
            
            self.transformerWaist = MovingFrameTransformation('tranformation_waist')
            plug(self.ccMc,self.transformerWaist.gMl) # inverted flexibility
            self.cMwaist = self.transformerWaist.lM0 # reference position in the world control frame
            # You need to set up the inverted flexibility : plug( ..., self.ccMc)
            # You need to set up a reference value here: plug( ... ,self.cMhref)

            plug(self.ccVc,self.transformerWaist.gVl) # inverted flexibility velocity
            self.cVwaist = self.transformerWaist.lV0 # reference velocity in the world control frame
            # You need to set up the inverted flexibility velocity : plug( ..., self.ccVc)
            # You need to set up a reference velocity value here: plug( ... ,self.cVhref)

            self.ccMwaist = self.transformerWaist.gM0 # reference matrix homo in the control frame
            self.ccVwaist = self.transformerWaist.gV0

            ### chest
            self.transformerChest = MovingFrameTransformation('tranformation_chest')
            plug(self.ccMc,self.transformerChest.gMl) # inverted flexibility
            self.cMchest = self.transformerChest.lM0 # reference position in the world control frame
            # You need to set up the inverted flexibility : plug( ..., self.ccMc)
            # You need to set up a reference value here: plug( ... ,self.cMhref)

            plug(self.ccVc,self.transformerChest.gVl) # inverted flexibility velocity
            self.cVchest = self.transformerChest.lV0 # reference velocity in the world control frame
            # You need to set up the inverted flexibility velocity : plug( ..., self.ccVc)
            # You need to set up a reference velocity value here: plug( ... ,self.cVhref)

            self.ccMchest = self.transformerChest.gM0 # reference matrix homo in the control frame
            self.ccVchest = self.transformerChest.gV0

            ### gaze
            self.transformerGaze = MovingFrameTransformation('tranformation_gaze')
            plug(self.ccMc,self.transformerGaze.gMl) # inverted flexibility
            self.cMgaze = self.transformerGaze.lM0 # reference position in the world control frame
            # You need to set up the inverted flexibility : plug( ..., self.ccMc)
            # You need to set up a reference value here: plug( ... ,self.cMhref)

            plug(self.ccVc,self.transformerGaze.gVl) # inverted flexibility velocity
            self.cVgaze = self.transformerGaze.lV0 # reference velocity in the world control frame
            # You need to set up the inverted flexibility velocity : plug( ..., self.ccVc)
            # You need to set up a reference velocity value here: plug( ... ,self.cVhref)

            self.ccMgaze = self.transformerGaze.gM0 # reference matrix homo in the control frame
            self.ccVgaze = self.transformerGaze.gV0            
            

    def plugStabilizer(self):


        '''Plug The stabilizer.'''
        self.sot.clear()
        if self.posture:
            self.push(self.taskPosture)
        self.push(self.tasks['com-stabilized'])
        self.push(self.tasks['ankles'])
        self.push(self.taskTrunk)
        if self.hands:
            self.push(self.taskRH)
            self.push(self.taskLH)
                       
        if self.trunkStabilize:
            #waist
            self.cMwaist.value = self.robot.dynamic.signal('waist').value
            self.cVwaist.value = (0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0)
        
            plug(self.ccMwaist,self.features['waist'].reference)
            plug(self.ccVwaist,self.features['waist'].velocity)
        
            #self.tasks['waist'].setWithDerivative (True)

            ### chest            
            self.cMchest.value = self.robot.dynamic.signal('chest').value
            self.cVchest.value = (0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0)
        
            plug(self.ccMchest,self.features['chest'].reference)
            plug(self.ccVchest,self.features['chest'].velocity)
        
            #self.tasks['chest'].setWithDerivative (True)

            ### value            
            self.cMgaze.value = self.robot.dynamic.signal('gaze').value
            self.cVgaze.value = (0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0)
        
            plug(self.ccMgaze,self.features['gaze'].reference)
            plug(self.ccVgaze,self.features['gaze'].velocity)
        
            #self.tasks['gaze'].setWithDerivative (True)            
        
        #######

        #self.cMlhref.value = self.robot.dynamic.lh.value
        #self.cVlhref.value = (0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0)
        #print matrix(self.cMlhref.value)

        ######
