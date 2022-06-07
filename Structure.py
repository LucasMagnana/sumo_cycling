class Structure:
    def __init__(self, start_edge, end_edge, edges, net, dict_cyclists, traci):
        for e in edges:
            id = e.getID()
            if(id == start_edge):
                self.start_edge = e
            if(id == end_edge):
                self.end_edge  = e

        self.path = net.getShortestPath(self.start_edge, self.end_edge, vClass='bicycle')

        self.dict_cyclists = dict_cyclists

        self.module_traci = traci

    def step(self):
        if(len(self.module_traci.edge.getLastStepVehicleIDs(self.start_edge.getID()))>8):
            for i in self.module_traci.edge.getLastStepVehicleIDs(self.start_edge.getID()):
                self.dict_cyclists[i].cross_struct()
