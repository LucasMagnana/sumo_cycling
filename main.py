import os, sys
from random import randint 
import pickle

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

save = False
load = False

tab_od = []

if(load):
    with open('OD.tab', 'rb') as infile:
        tab_od_loaded = pickle.load(infile)


while step <= 10000:
    if(len(dict_cyclists)<100):

        if(not load):
            e1 = randint(0, len(edges)-1)
            e2 = randint(0, len(edges)-1)
            path = net.getShortestPath(edges[e1], edges[e2], vClass='bicycle')
        else:
            e1 = tab_od_loaded[0][0]
            e2 = tab_od_loaded[0][1]
            tab_od_loaded.pop(0)
            path = net.getShortestPath(edges[e1], edges[e2], vClass='bicycle')

        if(save):
            tab_od.append([e1, e2])


        if(path[0] != None and len(path[0])>2 and edges[e1] not in structure.path and edges[e2] not in structure.path):
            dict_cyclists[str(id)]= Cyclist(str(id), step, path[0], dict_cyclists, net, structure, traci, sumolib)
            id+=1
    traci.simulationStep()


    for i in list(dict_cyclists.keys()):
        try:
            dict_cyclists[i].step(step, tab_diff)
        except traci.exceptions.TraCIException:
            if(save):
                with open('OD.tab', 'wb') as outfile:
                    pickle.dump(tab_od, outfile)
            raise KeyError
        structure.step()


    step += 1

print(sum(tab_diff)/len(tab_diff))
traci.close()