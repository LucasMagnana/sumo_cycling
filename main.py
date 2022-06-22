import os, sys
from random import randint
import numpy as np 
import pickle
import osmnx as ox
import shelve
import copy

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


def calculate_estimated_waiting_time(path, net):
    red_duration = 0
    total_duration = 0
    num_tls = 0
    for e in path:
        tls = net.getEdge(e).getTLS()
        if(tls):
            num_tls+=1                
            tl_concerned = []
            i=0
            for l in traci.trafficlight.getControlledLinks(tls.getID()):                
                if(e in l[0][0]):
                    tl_concerned.append(i)
                i+=1

            for p in traci.trafficlight.getAllProgramLogics(tls.getID())[0].getPhases():
                #print(p, tl_concerned)
                total_duration += p.minDur
                if('r' in p.state[tl_concerned[0]:tl_concerned[-1]]):
                    red_duration += p.minDur
    if(total_duration == 0):
        estimated_wait_tls = 0
    else:
        estimated_wait_tls = red_duration/total_duration*red_duration
        
    return estimated_wait_tls

traci.start(sumoCmd)

net = sumolib.net.readNet('osm.net.xml')
edges = net.getEdges()

dict_cyclists = {}


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


load_shortest_paths = False

if(load_shortest_paths):
    num_step = len(edges)**2
    i=0
    dict_shortest_path = {}
    for e1 in edges:
        for e2 in edges:
            print("\rStep: {}/{}, {}".format(i, num_step, len(dict_shortest_path)), end="")
            if(e1 != e2):
                path = net.getShortestPath(e1, e2, vClass='bicycle')
                if(path[0] != None):
                    path = [e.getID() for e in path[0]]
                    dict_shortest_path[e1.getID()+";"+e2.getID()] = {"path": path, "length":sumolib.route.getLength(net, path),\
                    "estimated_waiting_time": calculate_estimated_waiting_time(path, net)}
            i+=1

    with open('sp.dict', 'wb') as outfile:
        pickle.dump(dict_shortest_path, outfile)

else:
    with open('sp.dict', 'rb') as infile:
        dict_shortest_path = pickle.load(infile)

structure = Structure("237920408#0", "207728319#9", edges, net, dict_shortest_path, dict_cyclists, traci, sumolib, min_group_size=10)

last_dict_cyclists_keys = None


while step <= 10000:
    if(len(dict_cyclists)<100):

        if(not load):
            e1 = randint(0, len(edges)-1)
            e2 = randint(0, len(edges)-1)
            key_dict = edges[e1].getID()+";"+edges[e2].getID()
            if(key_dict in dict_shortest_path):
                path = dict_shortest_path[key_dict]
            else:
                path = None
        else:
            e1 = tab_od_loaded[0][0]
            e2 = tab_od_loaded[0][1]
            tab_od_loaded.pop(0)
            key_dict = edges[e1].getID()+";"+edges[e2].getID()
            if(key_dict in dict_shortest_path):
                path = dict_shortest_path[key_dict]
            else:
                path = None

        if(save):
            tab_od.append([e1, e2])


        if(path != None and len(path)>2 and edges[e1].getID() not in structure.path["path"] and edges[e2].getID() not in structure.path["path"]):
            max_speed = np.random.normal(15, 3)
            c = Cyclist(str(id), step, path, dict_shortest_path, dict_cyclists, net, structure, max_speed, traci, sumolib,struct_candidate=False)#id%2==0)
            if(c.alive):
                dict_cyclists[str(id)]=c
            id+=1


    traci.simulationStep()

    if(last_dict_cyclists_keys == None):
        last_dict_cyclists_keys = copy.deepcopy(list(dict_cyclists.keys()))   

    for i in last_dict_cyclists_keys:
        try:
            dict_cyclists[i].step(step, tab_diff, tab_ratio)
        except KeyError:
            traci.vehicle.remove(self.id)
            print(i, "removed from dict while still in simu (main)")

    structure.step(step)

    last_dict_cyclists_keys = copy.deepcopy(list(dict_cyclists.keys()))   
        


    step += 1

print("temp diff:", sum(tab_diff)/len(tab_diff), "ratio:", sum(tab_ratio)/len(tab_ratio), "data number:", len(tab_diff), "last id:", id, )
traci.close()