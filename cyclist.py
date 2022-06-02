class Cyclist:

    def __init__(self, id, step, path, tab_cyclists, net, edges_struct, traci, sumolib):
        self.id = id
        self.start_step = step
        self.net=net
        self.tab_cyclists = tab_cyclists

        self.module_sumolib = sumolib
        self.module_traci = traci

        self.module_traci.route.add(str(id)+"_sp", [e.getID() for e in path])
        self.module_traci.vehicle.add(str(id), str(id)+"_sp", departLane="best", typeID='bike_bicycle')#, departSpeed=traci.vehicletype.getMaxSpeed('bike_bicycle'))
        self.max_speed = self.module_traci.vehicle.getMaxSpeed(str(id))

        self.original_path = path

        self.path_to_struct = net.getShortestPath(self.original_path[0], edges_struct[0], vClass='bicycle')[0]
        self.path_from_struct = net.getShortestPath(edges_struct[1], self.original_path[-1], vClass='bicycle')[0]

        self.actual_path = self.original_path


        self.finish_step = None


    def step(self, step, tab_diff):    
        if(str(self.id) not in self.module_traci.vehicle.getIDList()):
            self.tab_cyclists.remove(self)
            '''print("estimated waiting time:", self.estimated_waiting_time, "waiting time:", self.waiting_time, "diff:", self.estimated_waiting_time-self.waiting_time)
            print("estimated distance:", self.estimated_distance, "distance:", self.distance_travelled, "diff:", self.estimated_distance-self.distance_travelled)
            print("estimated travel time :", self.estimated_travel_time, "travel time:", step-self.start_step, "diff:", self.estimated_travel_time-(step-self.start_step))
            print("================================================")'''
            if(self.finish_step): #WHY USEFUL ????
                tab_diff.append(((step-self.start_step)-self.estimated_travel_time)/self.estimated_travel_time)
        elif(self.finish_step == None):
            self.finish_step = self.calculate_ETA()
        else:
            self.waiting_time = self.module_traci.vehicle.getAccumulatedWaitingTime(str(self.id))
            self.distance_travelled = self.module_traci.vehicle.getDistance(str(self.id))
            self.calculate_ETA()

    def calculate_ETA(self):
        waiting_time = self.calculate_estimated_waiting_time()
        #self.estimated_distance = self.module_sumolib.route.getLength(self.net, self.module_sumolib.route.addInternal(self.net, self.actual_path))
        self.actual_path = self.path_to_struct
        self.module_traci.vehicle.changeTarget(str(self.id), self.actual_path[-1].getID())
        self.estimated_distance = self.module_traci.vehicle.getDrivingDistance(str(self.id), self.actual_path[-1].getID(), self.actual_path[-1].getLength())
        print(self.estimated_distance)
        self.actual_path = self.original_path
        self.module_traci.vehicle.changeTarget(str(self.id), self.actual_path[-1].getID())
        self.estimated_distance = self.module_traci.vehicle.getDrivingDistance(str(self.id), self.actual_path[-1].getID(), self.actual_path[-1].getLength())
        print(self.estimated_distance)
        print("=======================================")
        travel_time = self.estimated_distance/self.max_speed
        self.estimated_travel_time=travel_time+waiting_time
        self.estimated_travel_time+=self.estimated_travel_time*0.2
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
                #print("=====================================")
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
        return 0 #estimated_wait_tls

