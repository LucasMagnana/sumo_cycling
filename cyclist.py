class Cyclist:

    def __init__(self, id, step, max_speed, path, tab_cyclists, net, edges_struct, traci, sumolib):
        self.id = id
        self.tab_cyclists = tab_cyclists

        self.max_speed = max_speed

        self.module_sumolib = sumolib
        self.module_traci = traci

        self.start_step = step

        self.net=net

        self.path = path
        #self.finish_step = self.start_step+self.calculate_ETA(self.path, net)
        self.finish_step = None
        self.path_to_struct = net.getShortestPath(self.path[0], edges_struct[0], vClass='bicycle')
        self.path_from_struct = net.getShortestPath(edges_struct[1], self.path[-1], vClass='bicycle')


    def step(self, step, tab_diff):    
        if(str(self.id) not in self.module_traci.vehicle.getIDList()):
            self.tab_cyclists.remove(self)
            print(self.ewt, self.waited_time, self.estimated_distance, self.distance)
            if(self.finish_step): #WHY USEFUL ????
                tab_diff.append(step-self.finish_step)
        elif(self.finish_step == None):
            self.finish_step = self.calculate_ETA()
        else:
            self.waited_time = self.module_traci.vehicle.getAccumulatedWaitingTime(str(self.id))
            self.distance = self.module_traci.vehicle.getDistance(str(self.id))

    def calculate_ETA(self):
        waiting_time = self.calculate_estimated_waiting_time()
        #self.estimated_distance = self.module_sumolib.route.getLength(self.net, self.module_sumolib.route.addInternal(self.net, self.path))
        self.estimated_distance = self.module_traci.vehicle.getDrivingDistance(str(self.id), self.path[-1].getID(), self.path[-1].getLength())
        travel_time = self.estimated_distance/self.max_speed
        return self.start_step+travel_time+waiting_time

    def calculate_estimated_waiting_time(self):
        red_duration = 0
        total_duration = 0
        num_tls = 0
        for e in self.path:
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
        self.ewt = estimated_wait_tls
        return 0 #estimated_wait_tls

