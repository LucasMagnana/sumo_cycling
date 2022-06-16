import os, sys
from random import randint
import numpy as np 
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
import traci.constants as tc

traci.start(sumoCmd)

net = sumolib.net.readNet('osm.net.xml')
edges = net.getEdges()

dict_cyclists = {}


'''for i in traci.trafficlight.getIDList():
    traci.trafficlight.setProgram(i, 'off')'''


structure = Structure("237920408#0", "207728319#9", edges, net, dict_cyclists, traci, min_group_size=10)


tab_diff = []
tab_ratio = []

id=0
step=0

save = True
load = False

tab_od = []

if(load):
    with open('OD.tab', 'rb') as infile:
        tab_od_loaded = pickle.load(infile)




'''junctionID = "110421994"
traci.junction.subscribeContext(junctionID, tc.CMD_GET_VEHICLE_VARIABLE, 1000000, [tc.VAR_STOP_ENDING_VEHICLES_IDS])
stepLength = traci.simulation.getDeltaT()'''


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
            max_speed = 5.5 #np.random.normal(5.5, 2)
            dict_cyclists[str(id)]= Cyclist(str(id), step, path[0], dict_cyclists, net, structure, max_speed, traci, sumolib, struct_candidate=id%2==0)
            id+=1


    traci.simulationStep()

    '''scResults = traci.junction.getContextSubscriptionResults(junctionID)
    halting = 0
    if scResults:
        relSpeeds = [d[tc.VAR_STOP_ENDING_VEHICLES_IDS] for d in scResults.values()]
        print(relSpeeds)   '''   

    for i in list(dict_cyclists.keys()):
        try:
            dict_cyclists[i].step(step, tab_diff, tab_ratio)
        except traci.exceptions.TraCIException:
            if(save):
                print("Saving....")
                with open('OD.tab', 'wb') as outfile:
                    pickle.dump(tab_od, outfile)
            raise KeyError
        structure.step()


    step += 1

print(sum(tab_diff)/len(tab_diff), sum(tab_ratio)/len(tab_ratio))
traci.close()