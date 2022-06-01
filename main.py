import os, sys
from random import randint 

from cyclist import Cyclist

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

sumoBinary = "/usr/bin/sumo-gui"
sumoCmd = [sumoBinary, "-c", "osm.sumocfg", "--waiting-time-memory", '10000']


import traci
import sumolib

traci.start(sumoCmd)

net = sumolib.net.readNet('osm.net.xml')
edges = net.getEdges()

tab_cyclists = []


for i in traci.trafficlight.getIDList():
    traci.trafficlight.setProgram(i, 'off')

OD_struct = ["237920408", "207728319"]
edges_struct = [-1, -1]


for e in edges:
    id = e.getID().split('#')[0]
    if(id == OD_struct[0]):
        edges_struct[0] = e
    if(id == OD_struct[1]):
        edges_struct[1] = e

tab_diff = []

id=0
step=0
while step < 10000:
    if(len(tab_cyclists)<1):
        e1 = randint(0, len(edges)-1)
        e2 = e1
        while(e2 == e1):
            e2 = randint(0, len(edges)-1)

        e1=0
        e2=-1

        path = net.getShortestPath(edges[e1], edges[e2], vClass='bicycle')
        if(path[0] != None):
            traci.route.add(str(id), [e.getID() for e in path[0]])
            traci.vehicle.add(str(id), str(id), departLane="best", typeID='bike_bicycle')#, departSpeed=traci.vehicletype.getMaxSpeed('bike_bicycle'))
            traci.vehicle.setSpeed(str(id), traci.vehicle.getMaxSpeed(str(id)))
            tab_cyclists.append(Cyclist(id, step, traci.vehicle.getMaxSpeed(str(id)), path[0], tab_cyclists, net, edges_struct, traci, sumolib))
            id+=1
    traci.simulationStep()

    for c in tab_cyclists:
        c.step(step, tab_diff)

    step += 1

print(sum(tab_diff)/len(tab_diff))
traci.close()