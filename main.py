import os, sys
from random import randint
import numpy as np 
import pickle
import osmnx as ox
import copy
import torch

from Cyclist import Cyclist
from Structure import Structure
from graphs import *
from Model import Model



load_shortest_paths = False

new_scenario = False
edge_separation = True

open_struct=not new_scenario
min_group_size=5
step_gap=15
time_travel_multiplier=0.85

use_model = True
save_model = True


step_length = 1

num_cyclists = 2500
max_num_cyclists_same_time = 100





if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

sumoBinary = "/usr/bin/sumo"
sumoCmd = [sumoBinary, "-c", "osm.sumocfg", "--waiting-time-memory", '10000', '--start', '--quit-on-end', '--delay', '0', '--step-length', str(step_length), '--no-warnings']


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
    if(path != None and len(path["path"])>10 and edges[e1].getID() not in structure.path["path"] and edges[e2].getID() not in structure.path["path"]):
        if(max_speed==None):
            max_speed = np.random.normal(15, 3)
        c = Cyclist(id, step, path, dict_shortest_path, net, structure, max_speed, traci, sumolib, finish_step=finish_step)
        if(c.alive):
            dict_cyclists[id]=c
            if(tab_scenario != None):
                tab_scenario.append({"start_step": step, "start_edge":e1, "end_edge":e2, "max_speed": max_speed, "finish_step":-1})
            return True
    return False


def separate_edges(edges):
    leftmost_edge_x = 0
    rightmost_edge_x = sys.float_info.max
    dict_edges = {}
    tab_left_edges = []
    tab_right_edges = []
    for e in edges:
        to_node_coord = e.getToNode().getCoord()
        from_node_coord = e.getFromNode().getCoord()
        if(from_node_coord[0] < to_node_coord[0]):
            e_coord = (from_node_coord[0]+(to_node_coord[0]-from_node_coord[0])/2, from_node_coord[1]+(to_node_coord[1]-from_node_coord[1])/2)
        else:
            e_coord = (to_node_coord[0]+(from_node_coord[0]-to_node_coord[0])/2, to_node_coord[1]+(from_node_coord[1]-to_node_coord[1])/2)
        dict_edges[e_coord]=e
        if(e_coord[0] > leftmost_edge_x):
            leftmost_edge_x = e_coord[0]
        if(e_coord[0] < rightmost_edge_x):
            rightmost_edge_x = e_coord[0]
    middle_x = leftmost_edge_x+(rightmost_edge_x-leftmost_edge_x)/2

    for e_coord in dict_edges:
        if(e_coord[0]<=middle_x):
            tab_left_edges.append(dict_edges[e_coord])
        else:
            tab_right_edges.append(dict_edges[e_coord])

    return tab_left_edges, tab_right_edges
    




traci.start(sumoCmd)

net = sumolib.net.readNet('osm.net.xml')
edges = net.getEdges()

if(edge_separation):
    tab_left_edges, tab_right_edges = separate_edges(edges)



dict_cyclists = {}
dict_cyclists_arrived = {}

id=0
step=0

if(load_shortest_paths):
    num_step = len(edges)**2
    i=0
    dict_shortest_path = {}
    for e1 in edges:
        for e2 in edges:
            print("\rStep: {}/{}, {}".format(i, num_step, len(dict_shortest_path)), end="")
            if(e1 != e2):
                path = net.getShortestPath(e1, e2, vClass='bicycle')
                if(path[0] == None):
                    path = net.getOptimalPath(e1, e2, vClass='bicycle')
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


print(len(edges)**2-len(edges), len(dict_shortest_path))




if(new_scenario):
    print("WARNING : Creating a new scenario...")
    tab_scenario=[]
else:
    print("WARNING : Loading the scenario...")
    with open('scenario.tab', 'rb') as infile:
        tab_scenario = pickle.load(infile)

dict_edges_index = {}
for i, e in enumerate(edges) :
    dict_edges_index[e.getID()] = i


if(use_model == True):
    model = Model(len(edges), 256, 128)
    if(os.path.exists("models/model.pt")):
        model.load_state_dict(torch.load("models/model.pt"))
        model.eval()
else:
    model = None


structure = Structure("237920408#2", "207728319#9", edges, net, dict_shortest_path, dict_cyclists, traci, dict_edges_index, model,\
open=open_struct, min_group_size=min_group_size, step_gap=step_gap, time_travel_multiplier=time_travel_multiplier)

if(structure.open):
    print("WARNING : Structure is open...")
else:
    print("WARNING : Structure is closed...")



while(new_scenario and len(dict_cyclists)<max_num_cyclists_same_time):
    if(edge_separation):
        e1 = randint(0, len(tab_right_edges)-1)
        e2 = randint(0, len(tab_left_edges)-1)
        key_dict = tab_right_edges[e1].getID()+";"+tab_left_edges[e2].getID()
        while(key_dict not in dict_shortest_path):
            e1 = randint(0, len(tab_right_edges)-1)
            e2 = randint(0, len(tab_left_edges)-1)
            key_dict = tab_right_edges[e1].getID()+";"+tab_left_edges[e2].getID()
    else:
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
                if(edge_separation):
                    e1 = randint(0, len(tab_right_edges)-1)
                    e2 = randint(0, len(tab_left_edges)-1)
                    key_dict = tab_right_edges[e1].getID()+";"+tab_left_edges[e2].getID()
                    while(key_dict not in dict_shortest_path):
                        e1 = randint(0, len(tab_right_edges)-1)
                        e2 = randint(0, len(tab_left_edges)-1)
                        key_dict = tab_right_edges[e1].getID()+";"+tab_left_edges[e2].getID()
                else:
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
        while(id<=num_cyclists and step >= tab_scenario[id]["start_step"]):
            e1=tab_scenario[id]["start_edge"]
            e2=tab_scenario[id]["end_edge"]
            if(edge_separation):
                key_dict = tab_right_edges[e1].getID()+";"+tab_left_edges[e2].getID()
            else:
                key_dict = edges[e1].getID()+";"+edges[e2].getID()
            path = dict_shortest_path[key_dict]
            finish_step=-1
            if("finish_step" in tab_scenario[id]):
                finish_step=tab_scenario[id]["finish_step"]
            if(spawn_cyclist(str(id), step, path, dict_shortest_path, net, structure, edges, finish_step=finish_step, max_speed=tab_scenario[id]["max_speed"])):
                id+=1

    traci.simulationStep() 

    for i in copy.deepcopy(list(dict_cyclists.keys())):
        dict_cyclists[i].step(step, tab_scenario, new_scenario)
        if(not dict_cyclists[i].alive):
            dict_cyclists_arrived[i] = dict_cyclists[i]
            if(i in structure.dict_model_output):
                if(dict_cyclists[i].finish_step<tab_scenario[int(dict_cyclists[i].id)]["finish_step"]):
                    target = torch.Tensor([1])
                else:
                    target = torch.Tensor([0])
                structure.list_output_to_learn.append(structure.dict_model_output[i])
                structure.list_target.append(target)
                del structure.dict_model_output[i]
            del dict_cyclists[i]

    if(step%1==0):
        structure.step(step, edges)

    print("\rStep {}: {} cyclists in simu, {} cyclists spawned since start."\
    .format(step, len(traci.vehicle.getIDList()), id), end="")

    step += step_length

if(new_scenario):
    print("WARNING: Saving scenario...")
    with open('scenario.tab', 'wb') as outfile:
        pickle.dump(tab_scenario, outfile)

traci.close()

if(model != None and save_model):
    if(not os.path.exists("models")):
        os.makedirs("models")
    torch.save(model.state_dict(), "models/model.pt")


print("\ndata number:", len(dict_cyclists_arrived), ",", structure.num_cyclists_crossed, "cyclits used struct, last step:", step)


if(not new_scenario):
    tab_all_diff_arrival_time, tab_diff_finish_step, tab_diff_waiting_time, tab_diff_distance_travelled, tab_num_type_cyclists =\
    compute_graphs_data(structure.open, dict_cyclists_arrived, tab_scenario)

    plot_and_save_boxplot(tab_all_diff_arrival_time, "time_diff_struct", structure_was_open=structure.open)

    print("mean finish time diff:", sum(tab_all_diff_arrival_time)/len(tab_all_diff_arrival_time))

    if(structure.open):
        labels=["Gagnants", "Perdants", "Annulés", "Reste"]

        plot_and_save_boxplot(tab_diff_finish_step, "mean_time_diff", labels=labels)
        plot_and_save_boxplot(tab_diff_waiting_time, "mean_waiting_time", labels=labels)
        plot_and_save_boxplot(tab_diff_distance_travelled, "mean_distance_travelled", labels=labels)

        plot_and_save_bar(tab_num_type_cyclists, "cyclists_type", labels=labels)