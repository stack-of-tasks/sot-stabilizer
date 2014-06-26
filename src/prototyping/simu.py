import numpy as np
from numpy import linalg as LA
import matplotlib.pyplot as plt
from dynamic_graph import plug
from dynamic_graph.sot.application.stabilizer import LinearizedTableCartDevice, ZMPtoCoMAlgerbraicController
import math

cart = LinearizedTableCartDevice("cart")
controller = ZMPtoCoMAlgerbraicController("controller")

hcom=0.80771
dt = 0.005

cart.setCartMass(59.8)
cart.setCartHeight(hcom)
cart.setStiffness(53200.0*0.2)
cart.setViscosity(100.0)

openloop = False

if openloop:
    plug(cart.comHeight,controller.comHeight)
    plug(cart.state,controller.comIn)
    controller.comddotIN.value = (0.0,)
    plug(controller.comddot,cart.control)
else:
    plug(cart.comHeight,controller.comHeight)
    plug(cart.comreal,controller.comIn)
    plug(cart.flexcomddot,controller.comddotIN)
    plug(controller.comddot,cart.control)

controller.zmpref.value=(0.0,)



stepTime = 0.05
simuTime = 9

logZMP = np.array([])
logZMP.resize(simuTime/dt,2)

logZMPRef = np.array([])
logZMPRef.resize(simuTime/dt,2)

logComddot = np.array([])
logComddot.resize(simuTime/dt,2)


for i in range(1,int(stepTime/dt)):
    print(i)
    cart.incr(dt)
    logZMP[i,0] = i
    logZMP[i,1] = cart.zmp.value[0]
    logZMPRef[i,0] = i
    logZMPRef[i,1] = controller.zmpref.value[0]
    logComddot[i,0]= i
    logComddot[i,1]= controller.comddot.value[0]

controller.zmpref.value=(0.01,)

for i in range(int(stepTime/dt),int(simuTime/dt)):
    print(i)
    cart.incr(dt)
    logZMP[i,0] = i
    logZMP[i,1] = cart.zmp.value[0]
    logZMPRef[i,0] = i
    logZMPRef[i,1] = controller.zmpref.value[0]
    logComddot[i,0]= i
    logComddot[i,1]= controller.comddot.value[0]

fig = plt.figure(); axfig = fig.add_subplot(111)
axfig.plot(logZMP[:,0], logZMP[:,1], label='zmp X')
axfig.plot(logZMPRef[:,0], logZMPRef[:,1], label='zmpRef X')
#axfig.plot(logComddot[:,0], logComddot[:,1], label='comddot')
handles, labels = axfig.get_legend_handles_labels()

axfig.legend(handles, labels)
plt.show()

