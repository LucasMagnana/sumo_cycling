import time 

class Cyclist:

    def __init__(self, id, step, path, dict_shortest_path, net, structure, max_speed, traci, sumolib, step_length, struct_candidate=False, finish_step=-1):
        self.id = id
        self.start_step = step
        self.net=net
        self.structure = structure

        self.module_sumolib = sumolib
        self.module_traci = traci

        self.struct_candidate = struct_candidate

        self.original_path = path


        try:
            self.path_to_struct = dict_shortest_path[self.original_path["path"][0]+";"+self.structure.start_edge.getID()]
            self.path_from_struct = dict_shortest_path[self.structure.start_edge.getID()+";"+self.original_path["path"][-1]]
        except KeyError:
            #print("Spawn failed, no path from or to the structure")
            self.alive = False
            return 

        if(self.path_from_struct["path"][-1] in self.structure.path["path"]):
            self.last_edge_in_struct_id = self.path_from_struct["path"][-1]
        else:    
            for i in range(len(self.path_from_struct["path"])):
                if(self.path_from_struct["path"][i] not in self.structure.path["path"]):
                    self.last_edge_in_struct_id = self.path_from_struct["path"][i-1]
                    break

        self.module_traci.route.add(str(self.id)+"_sp", path["path"])
        self.module_traci.vehicle.add(str(self.id), str(self.id)+"_sp", departLane="best", typeID='bike_bicycle')#, departSpeed=traci.vehicletype.getMaxSpeed('bike_bicycle'))
        
        self.module_traci.vehicle.setMaxSpeed(self.id, max_speed)
        self.max_speed = self.module_traci.vehicle.getMaxSpeed(str(self.id)) 

        self.module_traci.vehicle.setActionStepLength(self.id, step_length)
        self.module_traci.vehicle.setTau(self.id, step_length)
        self.module_traci.vehicle.setMinGap(self.id, 0)


        self.dict_shortest_path = dict_shortest_path 


        self.actual_path = self.original_path


        self.highlited = False


        self.estimated_travel_time=path["length"]/self.max_speed+path["estimated_waiting_time"]
        self.estimated_finish_step = step+self.estimated_travel_time

        #self.estimated_finish_step = finish_step


        self.actual_edge_id = self.actual_path["path"][0]

        self.alive=True
        self.struct_crossed = False
        self.canceled_candidature = False

        self.step_cancel_struct_candidature = -1

        self.waiting_time = 0
        self.distance_travelled = 0

        self.arrived=False

        self.crossing_struct = False

        self.estimated_time_diff = 0

        self.path_used = []

        self.tab_speed = []
        self.tab_speed_w_stop = []





    def step(self, step):

        if(self.id in self.module_traci.vehicle.getIDList()):
            self.actual_edge_id = self.module_traci.vehicle.getRoadID(self.id)
            if(len(self.path_used) == 0 or self.path_used[-1] != self.actual_edge_id):
                self.path_used.append(self.actual_edge_id)
            speed = self.module_traci.vehicle.getSpeed(self.id)
            self.tab_speed_w_stop.append(speed)
            if(speed<0.5):
                self.waiting_time += 1
            else:
                self.tab_speed.append(speed)
            self.distance_travelled = self.module_traci.vehicle.getDistance(self.id)

            if(self.actual_edge_id==self.original_path["path"][-1]):
                self.arrived = True

            if(self.struct_candidate and not self.highlited):
                self.module_traci.vehicle.highlight(self.id)
                self.highlited = True
            
            if(self.struct_candidate and self.actual_path == self.original_path):
                path_to_struct_found = self.go_to_struct()
                if(not path_to_struct_found):
                    return

            if(self.step_cancel_struct_candidature > 0 and step>=self.step_cancel_struct_candidature and (self.id in self.structure.id_cyclists_waiting or self.actual_path == self.path_to_struct)):
                self.cancel_struct_candidature()

            if(self.actual_edge_id in self.structure.path["path"]):
                if(self.actual_path == self.original_path):
                    self.module_traci.vehicle.changeLane(self.id, 1, 1)
                elif(self.actual_edge_id != self.last_edge_in_struct_id):
                    self.module_traci.vehicle.changeLane(self.id, 0, 1)

                '''if(self.id == "48"):
                    print(self.module_traci.vehicle.getLaneIndex(self.id), self.module_traci.vehicle.wantsAndCouldChangeLane(self.id, -1), self.module_traci.vehicle.wantsAndCouldChangeLane(self.id, 1))
                if(self.actual_path == self.original_path):
                    if(self.module_traci.vehicle.getLaneIndex(self.id) == 0 or self.module_traci.vehicle.wantsAndCouldChangeLane(self.id, -1)):
                        self.module_traci.vehicle.changeLane(self.id, 1, self.net.getEdge(self.actual_edge_id).getLength()/self.max_speed)
                else: 
                    if(self.module_traci.vehicle.getLaneIndex(self.id) == 1 or self.module_traci.vehicle.wantsAndCouldChangeLane(self.id, 1)):
                        self.module_traci.vehicle.changeLane(self.id, 0, self.net.getEdge(self.actual_edge_id).getLength()/self.max_speed)'''

            if(self.crossing_struct and self.actual_edge_id[0] != ':' and self.actual_edge_id not in self.structure.path["path"]):
                self.exit_struct()

            if(self.canceled_candidature):
                self.module_traci.vehicle.highlight(self.id, color=(0, 0, 255, 255))
            if(self.struct_crossed):
                self.module_traci.vehicle.highlight(self.id, color=(0, 255, 0, 255))
            if(self.struct_candidate):
                self.module_traci.vehicle.highlight(self.id)

        elif(self.id in self.module_traci.simulation.getArrivedIDList()):
            self.alive = False
            self.finish_step=step
            self.mean_speed = sum(self.tab_speed)/len(self.tab_speed)
            self.mean_speed_w_stop = sum(self.tab_speed_w_stop)/len(self.tab_speed_w_stop)
            if(self.id in self.structure.id_cyclists_crossing_struct):
                self.structure.id_cyclists_crossing_struct.remove(self.id)
            if(self.id in self.structure.id_cyclists_waiting):
                self.structure.id_cyclists_waiting.remove(self.id)



    def calculate_ETA(self, step, path=None):
        if(path == None):
            path = self.actual_path

        waiting_time = self.calculate_estimated_waiting_time(path)
        self.estimated_distance = self.module_sumolib.route.getLength(self.net, path)
        #self.estimated_distance = self.module_traci.vehicle.getDrivingDistance(self.id, self.actual_path[-1].getID(), 0)
        travel_time = self.estimated_distance/self.max_speed
        self.estimated_travel_time=travel_time+waiting_time*1
        return step+self.estimated_travel_time


    def go_to_struct(self):
        self.actual_path = self.path_to_struct
        try:
            self.module_traci.vehicle.changeTarget(self.id, self.structure.start_edge.getID())
            self.module_traci.vehicle.setStop(self.id, self.structure.start_edge.getID(), self.structure.start_edge.getLength()-1)
            return True
        except self.module_traci.exceptions.TraCIException:
            self.alive = False
            self.finish_step = -1
            print(self.id, "bugged (no path found to the struct)")
            if self.id in self.structure.id_cyclists_waiting:
                self.structure.id_cyclists_waiting.remove(self.id)
            return False

    def cross_struct(self):
        self.actual_path = self.path_from_struct
        self.module_traci.vehicle.setRoute(self.id, self.path_from_struct["path"])
        if(self.module_traci.vehicle.isStopped(self.id)):
            self.module_traci.vehicle.resume(self.id)
        if(self.module_traci.vehicle.getNextStops(self.id)):
            self.module_traci.vehicle.setStop(self.id, self.structure.start_edge.getID(), self.structure.start_edge.getLength()-1, duration=0)
        self.crossing_struct = True


    def exit_struct(self):
        self.struct_candidate = False
        self.structure.id_cyclists_crossing_struct.remove(self.id)
        self.module_traci.vehicle.setMaxSpeed(self.id, self.max_speed)
        self.struct_crossed = True
        self.crossing_struct = False

    def cancel_struct_candidature(self):
        if(self.id in self.structure.id_cyclists_waiting):
            self.structure.id_cyclists_waiting.remove(self.id)
        else:
            print(self.id, "cancelling while not in waiting list")

        if(self.actual_edge_id[0]!=':'):
            try:
                self.original_path = self.dict_shortest_path[self.actual_edge_id+";"+self.original_path["path"][-1]]
            except KeyError:
                print(self.id, "bugged during cancelling candidature, no path found.")
                self.alive = False
                return
            self.module_traci.vehicle.setRoute(self.id, self.original_path["path"])
        else:
            self.module_traci.vehicle.changeTarget(self.id, self.original_path["path"][-1])
            
        self.actual_path = self.original_path
        self.struct_candidate = False
        self.canceled_candidature = True
        
        if(self.module_traci.vehicle.isStopped(self.id)):
            self.module_traci.vehicle.resume(self.id)
        if(self.module_traci.vehicle.getNextStops(self.id)):
            self.module_traci.vehicle.setStop(self.id, self.structure.start_edge.getID(), self.structure.start_edge.getLength()-1, duration=0)
        self.step_cancel_struct_candidature = -1

        self.structure.num_cyclists_canceled += 1



    def set_max_speed(self, max_speed):
        self.module_traci.vehicle.setMaxSpeed(self.id, max_speed)

            
                
