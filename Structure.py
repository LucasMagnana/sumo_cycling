import threading
import torch

class Structure:
    def __init__(self, start_edge, end_edge, edges, net, dict_shortest_path, dict_cyclists, traci,\
    dict_edges_index=None, model=None, open=True, min_group_size=5, step_gap=15, time_travel_multiplier=1, batch_size=32):

        for e in edges:
            id = e.getID()
            if(id == start_edge):
                self.start_edge = e
            if(id == end_edge):
                self.end_edge  = e

        self.dict_shortest_path = dict_shortest_path

        self.path = self.dict_shortest_path[self.start_edge.getID()+";"+self.end_edge.getID()]

        self.dict_cyclists = dict_cyclists

        self.module_traci = traci

        self.id_cyclists_crossing_struct = []
        self.id_cyclists_waiting = []

        self.model = model
        self.batch_size=batch_size
        if(self.model != None):
            self.optimizer = torch.optim.Adam(self.model.parameters(), lr=1e-4)

        self.loss = torch.nn.BCELoss()
        self.dict_edges_index = dict_edges_index
        self.dict_model_input = {}
        self.list_input_to_learn = []
        self.list_target = []
        self.list_loss = []


        for e in self.path["path"]:
            tls = net.getEdge(e).getTLS()
            if(tls):
                str_phase = ""
                str_phase_transition = ""
                str_inverse_phase = ""
                for l in self.module_traci.trafficlight.getControlledLinks(tls.getID()):                
                    if(e in l[0][0]):
                        str_phase+= 'G'
                        str_phase_transition+='y'
                        str_inverse_phase+='r'
                    else:
                        str_phase += 'r'
                        str_phase_transition+='G'
                        str_inverse_phase+='y'

                self.module_traci.trafficlight.setProgramLogic(tls.getID(), self.module_traci.trafficlight.Logic(1, 0, 0, \
                    phases=[self.module_traci.trafficlight.Phase(duration=99999, state=str_phase, minDur=9999, maxDur=9999), \
                        self.module_traci.trafficlight.Phase(duration=3, state=str_phase_transition, minDur=3, maxDur=3),
                        self.module_traci.trafficlight.Phase(duration=3, state=str_inverse_phase, minDur=3, maxDur=3)]))
                self.module_traci.trafficlight.setProgram(tls.getID(), 0)
        
        self.min_group_size = min_group_size
        self.activated = False

        self.net = net
        self.step_gap = step_gap

        self.num_cyclists_crossed = 0

        self.open = open
        self.time_travel_multiplier = time_travel_multiplier


    def step(self, step, edges):
        #print(step, self.id_cyclists_waiting)

        if(len(self.list_input_to_learn)>=self.batch_size):
            self.optimizer.zero_grad()
            tens_edges_occupation = torch.stack([i[0] for i in self.list_input_to_learn])
            tens_actual_edge = torch.stack([i[1] for i in self.list_input_to_learn])
            tens_target = torch.FloatTensor(self.list_target).unsqueeze(1)
            out = self.model(tens_edges_occupation, tens_actual_edge)
            l = self.loss(out, tens_target)
            self.list_loss.append(l.item())
            l.backward()
            self.optimizer.step()
            self.list_input_to_learn = []
            self.list_target = []


        if(self.open and step%self.step_gap==0):
            self.check_for_candidates(step, edges)           



        for i in self.module_traci.edge.getLastStepVehicleIDs(self.start_edge.getID()):
            if(self.module_traci.vehicle.getSpeed(i)<= 1 and i not in self.id_cyclists_waiting\
            and i not in self.id_cyclists_crossing_struct and self.dict_cyclists[i].struct_candidate):
                self.id_cyclists_waiting.append(i)
                self.dict_cyclists[i].step_cancel_struct_candidature = step+150
                


                    #print(i, "waiting")

        if(len(self.id_cyclists_waiting)>=self.min_group_size):
            self.activated = True
            min_max_speed = 100
            for i in self.id_cyclists_waiting:
                self.dict_cyclists[i].cross_struct()
                if(self.dict_cyclists[i].max_speed < min_max_speed):
                    min_max_speed = self.dict_cyclists[i].max_speed
                #print(i, "crossing")
                self.id_cyclists_crossing_struct.append(i)
                self.num_cyclists_crossed += 1
            self.id_cyclists_waiting = []

            for i in self.id_cyclists_crossing_struct:
                self.dict_cyclists[i].set_max_speed(min_max_speed)

            #print("activated at step", step)

        if(len(self.id_cyclists_crossing_struct)>0):

            if(set(self.module_traci.edge.getLastStepVehicleIDs(self.start_edge.getID())) & set(self.id_cyclists_crossing_struct)):
                for i in self.id_cyclists_waiting:
                    self.id_cyclists_crossing_struct.append(i)
                    self.dict_cyclists[i].cross_struct()
                    self.dict_cyclists[i].set_max_speed(self.dict_cyclists[self.id_cyclists_crossing_struct[0]].max_speed)
                    self.id_cyclists_waiting.remove(i)
            
        for e in self.path["path"]:
            tls = self.net.getEdge(e).getTLS()
            if(tls):
                if(set(self.module_traci.edge.getLastStepVehicleIDs(e)) & set(self.id_cyclists_crossing_struct)):
                    if(self.module_traci.trafficlight.getProgram(tls.getID()) == "0"):
                        self.module_traci.trafficlight.setProgram(tls.getID(), 1)
                else:
                    if(self.module_traci.trafficlight.getProgram(tls.getID()) == "1"):
                        self.module_traci.trafficlight.setProgram(tls.getID(), 0)

    def check_for_candidates(self, step, edges):
        list_id_candidates = []

        if(self.model != None and self.dict_edges_index != None):
            edges_occupation=[len(self.module_traci.edge.getLastStepVehicleIDs(e.getID())) for e in edges]
        for i in self.dict_cyclists:
            if(i not in self.id_cyclists_waiting and i not in self.id_cyclists_crossing_struct\
            and not self.dict_cyclists[i].struct_crossed and not self.dict_cyclists[i].canceled_candidature):
                if(self.dict_cyclists[i].actual_edge_id[0] != ":"):
                    if(self.model != None and self.dict_edges_index != None):
                        tens_edges_occupation = torch.tensor(edges_occupation, dtype=torch.float)
                        tens_actual_edge = torch.tensor([self.dict_edges_index[self.dict_cyclists[i].actual_edge_id],\
                        self.dict_edges_index[self.dict_cyclists[i].original_path["path"][-1]]], dtype=torch.float)
                        with torch.no_grad():
                            out = self.model(tens_edges_occupation, tens_actual_edge)
                        if(out >= 0.5):
                            self.dict_cyclists[i].struct_candidate=True
                        self.dict_model_input[i] = (tens_edges_occupation, tens_actual_edge)
                    else:
                            key_path_to_struct = self.dict_cyclists[i].actual_edge_id+";"+self.start_edge.getID()
                            if(key_path_to_struct in self.dict_shortest_path):
                                travel_time_by_struct = self.dict_shortest_path[key_path_to_struct]["length"]/self.dict_cyclists[i].max_speed+\
                                self.dict_shortest_path[key_path_to_struct]["estimated_waiting_time"]
                                travel_time_by_struct += self.path["length"]/self.dict_cyclists[i].max_speed
                                travel_time_by_struct += self.dict_cyclists[i].path_from_struct["length"]/self.dict_cyclists[i].max_speed+\
                                self.dict_cyclists[i].path_from_struct["estimated_waiting_time"]
                                step_arriving_by_crossing_struct = step+travel_time_by_struct*self.time_travel_multiplier

                                if(step_arriving_by_crossing_struct<=self.dict_cyclists[i].estimated_finish_step):
                                    list_id_candidates.append(i)

        if(self.model == None and len(list_id_candidates)>=self.min_group_size*1):
            for i in list_id_candidates:
                self.dict_cyclists[i].struct_candidate=True

        return



                                

