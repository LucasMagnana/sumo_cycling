import time 

class Cyclist:

    def __init__(self, id, step, path, dict_shortest_path, net, structure, max_speed, traci, sumolib, struct_candidate=False, finish_step=-1):
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
            self.path_from_struct = dict_shortest_path[self.structure.end_edge.getID()+";"+self.original_path["path"][-1]]
        except KeyError:
            #print("Spawn failed, no path from or to the structure")
            self.alive = False
            return 

        self.module_traci.route.add(str(self.id)+"_sp", path["path"])
        self.module_traci.vehicle.add(str(self.id), str(self.id)+"_sp", departLane="best", typeID='bike_bicycle')#, departSpeed=traci.vehicletype.getMaxSpeed('bike_bicycle'))
        
        self.module_traci.vehicle.setMaxSpeed(self.id, max_speed)
        self.max_speed = self.module_traci.vehicle.getMaxSpeed(str(self.id)) 

        self.module_traci.vehicle.setActionStepLength(self.id, 1)

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





    def step(self, step, tab_scenario, new_scenario):

        if(self.id in self.module_traci.vehicle.getIDList()):
            self.actual_edge_id = self.module_traci.vehicle.getRoadID(self.id)
            if(self.module_traci.vehicle.getSpeed(self.id)<0.5):
                self.waiting_time += 1
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

            if(self.actual_edge_id==self.structure.start_edge.getID() and len(self.structure.id_cyclists_waiting)>0):
                if(self.actual_path == self.original_path and self.module_traci.vehicle.getLaneIndex(self.id) == 0):
                    self.module_traci.vehicle.changeLane(self.id, 1, self.structure.start_edge.getLength()/self.max_speed)
                elif(self.actual_path == self.path_to_struct and self.module_traci.vehicle.getLaneIndex(self.id) == 1):
                    self.module_traci.vehicle.changeLane(self.id, 0, self.structure.start_edge.getLength()/self.max_speed)

            if(self.actual_path == self.structure.path and self.actual_edge_id==self.structure.end_edge.getID()):
                self.exit_struct()

            if(self.canceled_candidature):
                self.module_traci.vehicle.highlight(self.id, color=(0, 0, 255, 255))
            if(self.struct_crossed):
                self.module_traci.vehicle.highlight(self.id, color=(0, 255, 0, 255))
            if(self.struct_candidate):
                self.module_traci.vehicle.highlight(self.id)

        else:
            self.alive = False
            self.finish_step=step
            if(self.id in self.structure.id_cyclists_crossing_struct):
                self.structure.id_cyclists_crossing_struct.remove(self.id)
            if(self.id in self.structure.id_cyclists_waiting):
                self.structure.id_cyclists_waiting.remove(self.id)
            if(new_scenario):
                tab_scenario[int(self.id)]["finish_step"]=step
                tab_scenario[int(self.id)]["distance_travelled"]=self.distance_travelled
                tab_scenario[int(self.id)]["waiting_time"]=self.waiting_time


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
            print(self.id, "bugged (no path found to the struct)")
            return False

    def cross_struct(self):
        self.actual_path = self.structure.path
        self.module_traci.vehicle.setRoute(self.id, self.structure.path["path"])
        if(self.module_traci.vehicle.isStopped(self.id)):
            self.module_traci.vehicle.resume(self.id)
        if(self.module_traci.vehicle.getNextStops(self.id)):
            self.module_traci.vehicle.setStop(self.id, self.structure.start_edge.getID(), self.structure.start_edge.getLength()-1, duration=0)
        self.module_traci.vehicle.changeLane(self.id, 0, self.module_traci.vehicle.getDrivingDistance(self.id, self.actual_path["path"][-1], 0)/self.max_speed)


    def exit_struct(self):
        self.actual_path = self.path_from_struct
        self.module_traci.vehicle.setRoute(self.id, self.path_from_struct["path"])
        self.struct_candidate = False
        self.structure.id_cyclists_crossing_struct.remove(self.id)
        self.module_traci.vehicle.setMaxSpeed(self.id, self.max_speed)
        self.struct_crossed = True

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



    def set_max_speed(self, max_speed):
        self.module_traci.vehicle.setMaxSpeed(self.id, max_speed)

            
                
