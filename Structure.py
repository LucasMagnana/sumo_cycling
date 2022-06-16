class Structure:
    def __init__(self, start_edge, end_edge, edges, net, dict_cyclists, traci, min_group_size=5):
        for e in edges:
            id = e.getID()
            if(id == start_edge):
                self.start_edge = e
            if(id == end_edge):
                self.end_edge  = e

        self.path = net.getShortestPath(self.start_edge, self.end_edge, vClass='bicycle')[0]

        self.dict_cyclists = dict_cyclists

        self.module_traci = traci

        self.id_cyclists_crossing_struct = []
        self.id_cyclists_waiting = []

        for e in self.path:
            tls = e.getTLS()
            if(tls):
                str_phase = ""
                str_phase_transition = ""
                str_inverse_phase = ""
                for l in self.module_traci.trafficlight.getControlledLinks(tls.getID()):                
                    if(e.getID() in l[0][0]):
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


    def step(self):
        if(len(self.module_traci.edge.getLastStepVehicleIDs(self.start_edge.getID()))>=self.min_group_size):
            for i in self.module_traci.edge.getLastStepVehicleIDs(self.start_edge.getID()):
                if(self.module_traci.vehicle.getSpeed(i)==0 and i not in self.id_cyclists_waiting and i not in self.id_cyclists_crossing_struct and self.dict_cyclists[i].struct_candidate):
                    self.id_cyclists_waiting.append(i)

                    #print(i, "waiting")

        if(len(self.id_cyclists_waiting)>=self.min_group_size and not self.activated):
            self.activated = True
            min_max_speed = 100
            for i in self.id_cyclists_waiting:
                try:
                    self.dict_cyclists[i].cross_struct()
                except KeyError:
                    self.module_traci.vehicle.remove(i)
                    print(i, "bugged (disapperead from id list)")
                    continue
                if(self.dict_cyclists[i].max_speed < min_max_speed):
                    min_max_speed = self.dict_cyclists[i].max_speed
                #print(i, "crossing")
                self.id_cyclists_crossing_struct.append(i)
            self.id_cyclists_waiting = []

            for i in self.id_cyclists_crossing_struct:
                self.dict_cyclists[i].set_max_speed(min_max_speed)

        if(self.activated):
            for e in self.path:
                tls = e.getTLS()
                if(tls):
                    if(set(self.module_traci.edge.getLastStepVehicleIDs(e.getID())) & set(self.id_cyclists_crossing_struct)):
                            if(self.module_traci.trafficlight.getProgram(tls.getID()) == "0"):
                                self.module_traci.trafficlight.setProgram(tls.getID(), 1)
                    else:
                        if(self.module_traci.trafficlight.getProgram(tls.getID()) == "1"):
                            self.module_traci.trafficlight.setProgram(tls.getID(), 0)

        if(len(self.id_cyclists_crossing_struct)==0):
            self.activated = False



                                

