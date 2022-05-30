import os, sys
from random import randint 

from cyclist import Cyclist

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

sumoBinary = "/usr/bin/sumo-gui"
sumoCmd = [sumoBinary, "-c", "osm.sumocfg"]


import traci
import sumolib

traci.start(sumoCmd)

net = sumolib.net.readNet('osm.net.xml')
edges = net.getEdges()

tab_cyclists = []

OD_struct = ["237920408", "207728319"]
edges_struct = [-1, -1]


for e in edges:
    id = e.getID().split('#')[0]
    if(id == OD_struct[0]):
        edges_struct[0] = e
    if(id == OD_struct[1]):
        edges_struct[1] = e



id=0
step=0
while step < 1000:
    if(len(tab_cyclists)<1):
        e1 = randint(0, len(edges)-1)
        e2 = randint(0, len(edges)-1)

        path = net.getShortestPath(edges[e1], edges[e2], vClass='bicycle')
        if(path[0] != None):
            traci.route.add(str(id), [e.getID() for e in path[0]])
            traci.vehicle.add(str(id), str(id), departLane="best", typeID='bike_bicycle')
            tab_cyclists.append(Cyclist(id, path[0], tab_cyclists, net, edges_struct, traci.trafficlight))
            id+=1
    traci.simulationStep()

    for c in tab_cyclists:
        c.step(traci.vehicle)

    step += 1
traci.close()