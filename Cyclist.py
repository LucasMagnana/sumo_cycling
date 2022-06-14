import time 

class Cyclist:

    def __init__(self, id, step, path, dict_cyclists, net, structure, traci, sumolib, struct_candidate=True):
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
        self.max_speed = self.module_traci.vehicle.getMaxSpeed(str(self.id))

        self.original_path = path

        self.path_to_struct = net.getShortestPath(self.original_path[0], self.structure.start_edge, vClass='bicycle')[0]
        self.path_from_struct = net.getShortestPath(self.structure.end_edge, self.original_path[-1], vClass='bicycle')[0]

        self.actual_path = self.original_path


        self.finish_step = None




    def step(self, step, tab_diff):

        if(self.id not in self.module_traci.vehicle.getIDList()):
            self.remove()
            if(self.finish_step):
                tab_diff.append(((step-self.start_step)-self.estimated_travel_time))
            return


        if(self.struct_candidate):
            self.module_traci.vehicle.highlight(self.id)
        
        if(self.struct_candidate and self.actual_path == self.original_path):
            path_to_struct_found = self.go_to_struct()
            if(not path_to_struct_found):
                return

        '''if(not self.struct_candidate and self.module_traci.vehicle.getRoadID(self.id) == self.structure.start_edge.getID() and self.module_traci.vehicle.getSpeed(self.id)==0 and len(self.structure.id_cyclists_waiting)>0):
            self.module_traci.vehicle.changeSublane(self.id, 1)
            print("bite")'''

        if(self.actual_path == self.structure.path and self.module_traci.vehicle.getRoadID(self.id)==self.structure.end_edge.getID()):
            self.actual_path = self.original_path
            try:
                self.module_traci.vehicle.changeTarget(self.id, self.original_path[-1].getID())
            except self.module_traci.exceptions.TraCIException:
                print(self.id, self.module_traci.vehicle.getRoadID(self.id), self.original_path[-1].getID(), self.path_from_struct)
                raise self.module_traci.exceptions.TraCIException(0)

            self.struct_candidate = False
            self.structure.id_cyclists_crossing_struct.remove(self.id)

        if(self.finish_step == None):
            self.finish_step = self.calculate_ETA()

        self.waiting_time = self.module_traci.vehicle.getAccumulatedWaitingTime(self.id)
        self.distance_travelled = self.module_traci.vehicle.getDistance(self.id)

    def calculate_ETA(self):
        waiting_time = self.calculate_estimated_waiting_time()
        #self.estimated_distance = self.module_sumolib.route.getLength(self.net, self.module_sumolib.route.addInternal(self.net, self.actual_path))
        self.estimated_distance = self.module_traci.vehicle.getDrivingDistance(self.id, self.actual_path[-1].getID(), 0)
        travel_time = self.estimated_distance/self.max_speed
        self.estimated_travel_time=travel_time+waiting_time
        self.estimated_travel_time+=self.estimated_travel_time*-0.03
        return self.start_step+self.estimated_travel_time

    def calculate_estimated_waiting_time(self):
        red_duration = 0
        total_duration = 0
        num_tls = 0
        for e in self.actual_path:
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
