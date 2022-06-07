import os, sys
from random import randint 

from Cyclist import Cyclist
from Structure import Structure

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

dict_cyclists = {}


'''for i in traci.trafficlight.getIDList():
    traci.trafficlight.setProgram(i, 'off')'''


structure = Structure("237920408#0", "207728319#6", edges, net, dict_cyclists, traci)


tab_diff = []

id=0
step=0
while step < 10000:
    if(len(dict_cyclists)<10):
        e1 = randint(0, len(edges)-1)
        e2 = e1
        while(e2 == e1):
            e2 = randint(0, len(edges)-1)

        '''e1=0
        e2=-1'''

        path = net.getShortestPath(edges[e1], edges[e2], vClass='bicycle')
        if(path[0] != None):

            dict_cyclists[str(id)]= Cyclist(str(id), step, path[0], dict_cyclists, net, structure, traci, sumolib)
            id+=1
    traci.simulationStep()

    for i in list(dict_cyclists.keys()):
        dict_cyclists[i].step(step, tab_diff)
    structure.step()

    step += 1

print(sum(tab_diff)/len(tab_diff))
traci.close()