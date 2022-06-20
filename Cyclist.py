import time 

class Cyclist:

    def __init__(self, id, step, path, dict_cyclists, net, structure, max_speed, traci, sumolib, struct_candidate=True):
        self.id = id
        self.start_step = step
        self.net=net
        self.dict_cyclists = dict_cyclists
        self.structure = structure

        self.module_sumolib = sumolib
        self.module_traci = traci

        self.struct_candidate = struct_candidate

        '''if(self.id == "4"):
            self.struct_candidate = False
            for e in net.getEdges():
                id = e.getID()
                if(id == "157655114#2"):
                    start_edge = e
                if(id == "779291540#0"):
                    end_edge  = e
            path = net.getShortestPath(start_edge, end_edge, vClass='bicycle')[0]'''


        self.module_traci.route.add(str(self.id)+"_sp", [e.getID() for e in path])
        self.module_traci.vehicle.add(str(self.id), str(self.id)+"_sp", departLane="best", typeID='bike_bicycle')#, departSpeed=traci.vehicletype.getMaxSpeed('bike_bicycle'))
        
        self.module_traci.vehicle.setMaxSpeed(self.id, max_speed)
        self.max_speed = self.module_traci.vehicle.getMaxSpeed(str(self.id))

        self.original_path = path

        self.path_to_struct = net.getShortestPath(self.original_path[0], self.structure.start_edge, vClass='bicycle')[0]
        self.path_from_struct = net.getShortestPath(self.structure.end_edge, self.original_path[-1], vClass='bicycle')[0]


        self.actual_path = self.original_path


        self.estimated_finish_step = None
        self.highlited = False

        self.nb_step_disappeared = 0
        self.max_step_disappeared = 5

        self.estimated_finish_step = self.calculate_ETA(self.start_step)

        self.actual_edge_id = self.actual_path[0].getID()




    def step(self, step, tab_diff, tab_ratio):

        if(self.id in self.module_traci.vehicle.getIDList()):
            self.actual_edge_id = self.module_traci.vehicle.getRoadID(self.id)
            self.nb_step_disappeared = 0

            if(self.struct_candidate and not self.highlited):
                self.module_traci.vehicle.highlight(self.id)
                self.highlited = True
            
            if(self.struct_candidate and self.actual_path == self.original_path):
                path_to_struct_found = self.go_to_struct()
                if(not path_to_struct_found):
                    return

            if(self.module_traci.vehicle.getRoadID(self.id)==self.structure.start_edge.getID()):
                if(self.actual_path == self.original_path and self.module_traci.vehicle.getLaneIndex(self.id) == 0):
                    self.module_traci.vehicle.changeLane(self.id, 1, self.structure.start_edge.getLength()/self.max_speed)
                elif(self.actual_path == self.path_to_struct and self.module_traci.vehicle.getLaneIndex(self.id) == 1):
                    self.module_traci.vehicle.changeLane(self.id, 0, self.structure.start_edge.getLength()/self.max_speed)

            if(self.actual_path == self.structure.path and self.module_traci.vehicle.getRoadID(self.id)==self.structure.end_edge.getID()):
                self.exit_struct()

            self.waiting_time = self.module_traci.vehicle.getAccumulatedWaitingTime(self.id)
            self.distance_travelled = self.module_traci.vehicle.getDistance(self.id)
        else:
            if(self.nb_step_disappeared < self.max_step_disappeared):
                self.nb_step_disappeared+=1
            else :
                self.remove()
                if(self.estimated_finish_step):
                    tab_ratio.append(((step-self.start_step)-self.estimated_travel_time)/self.estimated_travel_time)
                    tab_diff.append((step-self.start_step)-self.estimated_travel_time)
                return

    def calculate_ETA(self, step, path=None):
        if(path == None):
            path = self.actual_path

        waiting_time = self.calculate_estimated_waiting_time(path)
        self.estimated_distance = self.module_sumolib.route.getLength(self.net, path)
        #self.estimated_distance = self.module_traci.vehicle.getDrivingDistance(self.id, self.actual_path[-1].getID(), 0)
        travel_time = self.estimated_distance/self.max_speed
        self.estimated_travel_time=travel_time+waiting_time
        self.estimated_travel_time+=self.estimated_travel_time*0.2
        return step+self.estimated_travel_time

    def calculate_estimated_waiting_time(self, path):
        red_duration = 0
        total_duration = 0
        num_tls = 0
        for e in path:
            tls = e.getTLS()
            if(tls):
                num_tls+=1                
                tl_concerned = []
                i=0
                for l in self.module_traci.trafficlight.getControlledLinks(tls.getID()):                
                    if(e.getID() in l[0][0]):
                        tl_concerned.append(i)
                    i+=1

                for p in self.module_traci.trafficlight.getAllProgramLogics(tls.getID())[0].getPhases():
                    #print(p, tl_concerned)
                    total_duration += p.minDur
                    if('r' in p.state[tl_concerned[0]:tl_concerned[-1]]):
                        red_duration += p.minDur
        if(total_duration == 0):
            estimated_wait_tls = 0
        else:
            estimated_wait_tls = red_duration/total_duration*red_duration
        self.estimated_waiting_time = estimated_wait_tls
        return estimated_wait_tls


    def go_to_struct(self):
        self.actual_path = self.path_to_struct
        try:
            self.module_traci.vehicle.changeTarget(self.id, self.structure.start_edge.getID())
            self.module_traci.vehicle.setStop(self.id, self.structure.start_edge.getID(), self.structure.start_edge.getLength()-1)
            return True
        except self.module_traci.exceptions.TraCIException:
            self.remove()
            print(self.id, "bugged (no path found to the struct)")
            return False

    def cross_struct(self):
        self.actual_path = self.structure.path
        self.module_traci.vehicle.changeTarget(self.id, self.structure.end_edge.getID())
        if(self.module_traci.vehicle.isStopped(self.id)):
            self.module_traci.vehicle.resume(self.id)
        if(self.module_traci.vehicle.getNextStops(self.id)):
            self.module_traci.vehicle.setStop(self.id, self.structure.start_edge.getID(), self.structure.start_edge.getLength()-1, duration=0)
        self.module_traci.vehicle.changeLane(self.id, 0, self.module_traci.vehicle.getDrivingDistance(self.id, self.actual_path[-1].getID(), 0)/self.max_speed)


    def remove(self):
        #print(self.id, "removed")
        del self.dict_cyclists[self.id]
        if(self.id in self.structure.id_cyclists_crossing_struct):
            self.structure.id_cyclists_crossing_struct.remove(self.id)
        if(self.id in self.structure.id_cyclists_waiting):
            self.structure.id_cyclists_waiting.remove(self.id)
        try:
            self.module_traci.vehicle.remove(self.id)
            print(self.id, "removed from dict while still in simu")
        except self.module_traci.exceptions.TraCIException:
            return

    def exit_struct(self):
        self.actual_path = self.original_path
        self.module_traci.vehicle.setRoute(self.id, [e.getID() for e in self.path_from_struct])
        self.struct_candidate = False
        self.structure.id_cyclists_crossing_struct.remove(self.id)
        self.module_traci.vehicle.setMaxSpeed(self.id, self.max_speed)

    def set_max_speed(self, max_speed):
        self.module_traci.vehicle.setMaxSpeed(self.id, max_speed)

            
                
