import os, sys
from random import randint
import numpy as np 
import pickle
import osmnx as ox
import copy
import matplotlib.pyplot as plt

from Cyclist import Cyclist
from Structure import Structure

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

sumoBinary = "/usr/bin/sumo-gui"
sumoCmd = [sumoBinary, "-c", "osm.sumocfg", "--waiting-time-memory", '10000', '--start', '--quit-on-end', '--delay', '0', '--no-warnings']


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


def spawn_cyclist(id, step, path, dict_shortest_path, net, structure, edges, tab_scenario=None, max_speed=None, finish_step=-1):
    if(path != None and len(path)>2 and edges[e1].getID() not in structure.path["path"] and edges[e2].getID() not in structure.path["path"]):
        if(max_speed==None):
            max_speed = np.random.normal(15, 3)
        c = Cyclist(id, step, path, dict_shortest_path, net, structure, max_speed, traci, sumolib, finish_step=finish_step)
        if(c.alive):
            dict_cyclists[id]=c
            if(tab_scenario != None):
                tab_scenario.append({"start_step": step, "start_edge":e1, "end_edge":e2, "max_speed": max_speed, "finish_step":-1})
            return True
    return False


traci.start(sumoCmd)

net = sumolib.net.readNet('osm.net.xml')
edges = net.getEdges()

dict_cyclists = {}
dict_cyclists_deleted = {}
dict_cyclists_arrived = {}

id=0
step=0

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




new_scenario = False

if(new_scenario):
    print("WARNING : Creating a new scenario...")
    tab_scenario=[]
else:
    print("WARNING : Loading the scenario...")
    with open('scenario.tab', 'rb') as infile:
        tab_scenario = pickle.load(infile)


structure = Structure("237920408#2", "207728319#9", edges, net, dict_shortest_path, dict_cyclists, dict_cyclists_deleted, dict_cyclists_arrived, traci,\
open=not new_scenario, min_group_size=5, step_gap=15, time_travel_multiplier=0.85)

if(structure.open):
    print("WARNING : Structure is open...")
else:
    print("WARNING : Structure is closed...")


num_cyclists = 2500
max_num_cyclists_same_time = 100


while(new_scenario and len(dict_cyclists)<max_num_cyclists_same_time):
    e1 = randint(0, len(edges)-1)
    e2 = randint(0, len(edges)-1)
    key_dict = edges[e1].getID()+";"+edges[e2].getID()
    while(key_dict not in dict_shortest_path):
        e1 = randint(0, len(edges)-1)
        e2 = randint(0, len(edges)-1)
        key_dict = edges[e1].getID()+";"+edges[e2].getID()
    path = dict_shortest_path[key_dict]
    if(spawn_cyclist(str(id), step, path, dict_shortest_path, net, structure, edges, tab_scenario=tab_scenario)):
        id+=1


while(len(dict_cyclists) != 0 or id<=num_cyclists):
    path=None
    if(new_scenario):
        if(id<=num_cyclists):
            if(len(dict_cyclists)<max_num_cyclists_same_time):
                e1 = randint(0, len(edges)-1)
                e2 = randint(0, len(edges)-1)
                key_dict = edges[e1].getID()+";"+edges[e2].getID()
                while(key_dict not in dict_shortest_path):
                    e1 = randint(0, len(edges)-1)
                    e2 = randint(0, len(edges)-1)
                    key_dict = edges[e1].getID()+";"+edges[e2].getID()
                path = dict_shortest_path[key_dict]

                if(spawn_cyclist(str(id), step, path, dict_shortest_path, net, structure, edges, tab_scenario=tab_scenario)):
                    id+=1
    else:
        while(id<=num_cyclists and step == tab_scenario[id]["start_step"]):
            e1=tab_scenario[id]["start_edge"]
            e2=tab_scenario[id]["end_edge"]
            key_dict = edges[e1].getID()+";"+edges[e2].getID()
            path = dict_shortest_path[key_dict]
            finish_step=-1
            if("finish_step" in tab_scenario[id]):
                finish_step=tab_scenario[id]["finish_step"]
            if(spawn_cyclist(str(id), step, path, dict_shortest_path, net, structure, edges, finish_step=finish_step, max_speed=tab_scenario[id]["max_speed"])):
                id+=1

    traci.simulationStep() 

    for i in copy.deepcopy(list(dict_cyclists_deleted.keys())):
        if(i in traci.vehicle.getIDList()):
            dict_cyclists[i] = dict_cyclists_deleted[i]
            del dict_cyclists_deleted[i]
            dict_cyclists[i].alive=True
        elif(step-dict_cyclists_deleted[i].finish_step>100):
            dict_cyclists_arrived[i] = dict_cyclists_deleted[i]
            del dict_cyclists_deleted[i]


    for i in copy.deepcopy(list(dict_cyclists.keys())):
        dict_cyclists[i].step(step, tab_scenario, new_scenario)
        if(not dict_cyclists[i].alive):
            if(not dict_cyclists[i].arrived):
                dict_cyclists_deleted[i] = dict_cyclists[i]
            else:
                dict_cyclists_arrived[i] = dict_cyclists[i]
            del dict_cyclists[i]

    structure.step(step)

    print("\rStep {}: {} cyclists in simu, {} cyclists spawned since start, {} in dict_deleted."\
    .format(step, len(traci.vehicle.getIDList()), id, len(dict_cyclists_deleted)), end="")

    step += 1

if(new_scenario):
    print("WARNING: Saving scenario...")
    with open('scenario.tab', 'wb') as outfile:
        pickle.dump(tab_scenario, outfile)

traci.close()

for i in dict_cyclists_deleted:
    dict_cyclists_arrived[i]=dict_cyclists_deleted[i]


print("\ndata number:", len(dict_cyclists_arrived), ",", structure.num_cyclists_crossed, "cyclits used struct, last step:", step)

tab_diff_finish_step = [[],[],[], []]
tab_diff_waiting_time = [[],[],[], []]
tab_diff_distance_travelled = [[],[],[], []]
tab_num_type_cyclists = [0, 0, 0, 0]

tab_all_diff_arrival_time=[]

if(not new_scenario):
    for i in dict_cyclists_arrived:
        c = dict_cyclists_arrived[i]
        tab_all_diff_arrival_time.append(tab_scenario[int(c.id)]["finish_step"]-c.finish_step)
        if(structure.open):
            if(c.canceled_candidature):
                tab_diff_finish_step[2].append(tab_scenario[int(c.id)]["finish_step"]-c.finish_step)
                tab_diff_waiting_time[2].append(tab_scenario[int(c.id)]["waiting_time"]-c.waiting_time)
                tab_diff_distance_travelled[2].append(tab_scenario[int(c.id)]["distance_travelled"]-c.distance_travelled)
                tab_num_type_cyclists[2]+=1
            elif(c.struct_crossed):
                if(c.finish_step>tab_scenario[int(c.id)]["finish_step"]):
                    tab_diff_finish_step[1].append(tab_scenario[int(c.id)]["finish_step"]-c.finish_step)
                    tab_diff_waiting_time[1].append(tab_scenario[int(c.id)]["waiting_time"]-c.waiting_time)
                    tab_diff_distance_travelled[1].append(tab_scenario[int(c.id)]["distance_travelled"]-c.distance_travelled)
                    tab_num_type_cyclists[1]+=1
                elif(c.finish_step<tab_scenario[int(c.id)]["finish_step"]):
                    tab_diff_finish_step[0].append(tab_scenario[int(c.id)]["finish_step"]-c.finish_step)
                    tab_diff_waiting_time[0].append(tab_scenario[int(c.id)]["waiting_time"]-c.waiting_time)
                    tab_diff_distance_travelled[0].append(tab_scenario[int(c.id)]["distance_travelled"]-c.distance_travelled)
                    tab_num_type_cyclists[0]+=1
            else:
                tab_diff_finish_step[3].append(tab_scenario[int(c.id)]["finish_step"]-c.finish_step)
                tab_diff_waiting_time[3].append(tab_scenario[int(c.id)]["waiting_time"]-c.waiting_time)
                tab_diff_distance_travelled[3].append(tab_scenario[int(c.id)]["distance_travelled"]-c.distance_travelled)
                tab_num_type_cyclists[3]+=1

                


    tab_mean_diff_arrival_time = []
    for i in range(len(tab_diff_finish_step)):
        if(len(tab_diff_finish_step[i])==0):
            tab_mean_diff_arrival_time.append(0)
        else:
            tab_mean_diff_arrival_time.append(sum(tab_diff_finish_step[i])/len(tab_diff_finish_step[i]))


    tab_mean_diff_waiting_time = []
    for i in range(len(tab_diff_waiting_time)):
        if(len(tab_diff_waiting_time[i])==0):
            tab_mean_diff_waiting_time.append(0)
        else:
            tab_mean_diff_waiting_time.append(sum(tab_diff_waiting_time[i])/len(tab_diff_waiting_time[i]))

    tab_mean_diff_distance_travelled = []
    for i in range(len(tab_diff_distance_travelled)):
        if(len(tab_diff_distance_travelled[i])==0):
            tab_mean_diff_distance_travelled.append(0)
        else:
            tab_mean_diff_distance_travelled.append(sum(tab_diff_distance_travelled[i])/len(tab_diff_distance_travelled[i]))

    plt.clf()
    fig1, ax1 = plt.subplots()
    ax1.set_title('')
    ax1.boxplot(tab_all_diff_arrival_time)
    if(structure.open):
        plt.savefig("images/time_diff_struct_open.png")
    else:
        plt.savefig("images/time_diff_struct_close.png")

    print("mean finish time diff:", sum(tab_all_diff_arrival_time)/len(tab_all_diff_arrival_time))


    if(structure.open):

        labels=["Gagnants", "Perdants", "Annul??s", "Reste"]

        plt.clf()
        fig1, ax1 = plt.subplots()
        ax1.set_title('')
        ax1.bar(range(len(tab_mean_diff_arrival_time)), tab_mean_diff_arrival_time, tick_label=labels)
        plt.savefig("images/mean_time_diff.png")

        plt.clf()
        fig1, ax1 = plt.subplots()
        ax1.set_title('')
        ax1.bar(range(len(tab_mean_diff_waiting_time)), tab_mean_diff_waiting_time, tick_label=labels)
        plt.savefig("images/mean_waiting_time.png")

        plt.clf()
        fig1, ax1 = plt.subplots()
        ax1.set_title('')
        ax1.bar(range(len(tab_mean_diff_distance_travelled)), tab_mean_diff_distance_travelled, tick_label=labels)
        plt.savefig("images/mean_distance_travelled.png")

        plt.clf()
        fig1, ax1 = plt.subplots()
        ax1.set_title('')
        ax1.bar(range(len(tab_num_type_cyclists)), tab_num_type_cyclists, tick_label=labels)
        plt.savefig("images/cyclists_type.png")
