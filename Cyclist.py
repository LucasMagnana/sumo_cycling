class Cyclist:

    def __init__(self, id, step, path, dict_cyclists, net, structure, traci, sumolib):
        self.id = id
        self.start_step = step
        self.net=net
        self.dict_cyclists = dict_cyclists
        self.structure = structure

        self.module_sumolib = sumolib
        self.module_traci = traci

        self.module_traci.route.add(str(id)+"_sp", [e.getID() for e in path])
        self.module_traci.vehicle.add(str(id), str(id)+"_sp", departLane="best", typeID='bike_bicycle')#, departSpeed=traci.vehicletype.getMaxSpeed('bike_bicycle'))
        self.max_speed = self.module_traci.vehicle.getMaxSpeed(str(id))

        self.original_path = path

        self.path_to_struct = net.getShortestPath(self.original_path[0], self.structure.start_edge, vClass='bicycle')[0]
        self.path_from_struct = net.getShortestPath(self.structure.end_edge, self.original_path[-1], vClass='bicycle')[0]

        self.actual_path = self.original_path


        self.finish_step = None
        self.struct_passed = False
        self.crossing_struct = False


    def step(self, step, tab_diff):
        if(str(self.id) not in self.module_traci.vehicle.getIDList()):
            del self.dict_cyclists[self.id]
            if(self.finish_step): #WHY USEFUL ????
                tab_diff.append(((step-self.start_step)-self.estimated_travel_time))
            return

        if(self.actual_path == self.original_path and not self.struct_passed):
            self.go_to_struct()

        if(self.actual_path == self.structure.path and self.module_traci.vehicle.getRoadID(str(self.id))==self.structure.end_edge.getID()):
            self.actual_path = self.original_path
            self.module_traci.vehicle.changeTarget(str(self.id), self.original_path[-1].getID())
            self.crossing_struct = False
            self.struct_passed = True
            self.structure.id_cyclists_crossing_struct.remove(self.id)


        if(self.module_traci.vehicle.getRoadID(str(self.id))==self.original_path[-1].getID()):
            self.module_traci.vehicle.remove(str(self.id))
        elif(self.finish_step == None):
            self.finish_step = self.calculate_ETA()
        else:
            self.waiting_time = self.module_traci.vehicle.getAccumulatedWaitingTime(str(self.id))
            self.distance_travelled = self.module_traci.vehicle.getDistance(str(self.id))

    def calculate_ETA(self):
        waiting_time = self.calculate_estimated_waiting_time()
        #self.estimated_distance = self.module_sumolib.route.getLength(self.net, self.module_sumolib.route.addInternal(self.net, self.actual_path))
        self.estimated_distance = self.module_traci.vehicle.getDrivingDistance(str(self.id), self.actual_path[-1].getID(), 0)
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
        self.module_traci.vehicle.changeTarget(str(self.id), self.structure.start_edge.getID())
        self.module_traci.vehicle.setStop(str(self.id), self.structure.start_edge.getID(), self.structure.start_edge.getLength()-1)

    def cross_struct(self):
        self.crossing_struct = True
        self.actual_path = self.structure.path
        self.module_traci.vehicle.changeTarget(str(self.id), self.structure.end_edge.getID())
        if(self.module_traci.vehicle.isStopped(str(self.id))):
            self.module_traci.vehicle.resume(str(self.id))
        else:
            self.module_traci.vehicle.setStop(str(self.id), self.structure.start_edge.getID(), self.structure.start_edge.getLength()-1, duration=0)
