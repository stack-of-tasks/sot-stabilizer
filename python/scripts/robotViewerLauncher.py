import sys
# --- ROBOT DYNAMIC SIMULATION -------------------------------------------------
from dynamic_graph.sot.hrp2_14.robot import Robot
robot = Robot( 'robot' )

# --- LINK ROBOT VIEWER -------------------------------------------------------
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer
addRobotViewer(robot.device,small=True,verbose=False)
robot.timeStep=5e-3
usingRobotViewer = True

# --- MAIN LOOP ----------------------------------------------------------------

from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,optionalparentheses,loopShortcuts

refreshList = list()
@loopInThread
def loop():
    robot.device.increment(robot.timeStep)
    for cmd in refreshList: cmd()
runner=loop()

[go,stop,next,n] = loopShortcuts(runner)

@optionalparentheses
def iter():         print 'iter = ',robot.device.state.time
@optionalparentheses
def status():       print runner.isPlay

# ----------------------------------------------------------------------

for scripts in sys.argv[1:]:
    if scripts[0]!='+':
        raw_input('Enter when you are ready to execute **'+scripts+'** :')
    else: scripts = scripts[1:]

    loop = scripts[0]=='*'
    if loop: scripts = scripts[1:]
    while True:
        if scripts[0]=='=':
            print "["+scripts[1:]+"]"
            exec(scripts[1:])
        else:
            execfile(scripts)
        if loop:  raw_input('Again <'+scripts+'> ?')
        else: break
    
