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
id=0

step=0
while step < 1000:
    if(traci.vehicle.getIDCount()<10):
        e1 = randint(0, len(edges)-1)
        e2 = randint(0, len(edges)-1)

        path = net.getShortestPath(edges[e1], edges[e2], vClass='bicycle')
        if(path[0] != None):
            traci.route.add(str(id), [e.getID() for e in path[0]])
            traci.vehicle.add(str(id), str(id), departLane="best", typeID='bike_bicycle')
            tab_cyclists.append(Cyclist(id, path, tab_cyclists))
            id+=1
    traci.simulationStep()

    for c in tab_cyclists:
        c.step(traci.vehicle)

    step += 1
traci.close()