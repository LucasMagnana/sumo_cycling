class Cyclist:

    def __init__(self, id, path, tab_cyclists, net, edges_struct, traffic_lights):
        self.path = path
        self.calculate_ETA(self.path, traffic_lights)

        self.id = id
        self.tab_cyclists = tab_cyclists

        self.path_to_struct = net.getShortestPath(self.path[0], edges_struct[0], vClass='bicycle')
        self.path_from_struct = net.getShortestPath(edges_struct[1], self.path[-1], vClass='bicycle')

    def step(self, vehicle):
        if(str(self.id) not in vehicle.getIDList()):
            self.tab_cyclists.remove(self)

    def calculate_ETA(self, path, traffic_lights):
        for e in path:
            tls = e.getTLS()
            if(tls):
                tl_concerned = []
                i=0
                for l in traffic_lights.getControlledLinks(tls.getID()):                
                    if(e.getID() in l[0][0]):
                        tl_concerned.append(i)
                    i+=1

                red_duration = 0
                total_duration = 0

                for p in traffic_lights.getAllProgramLogics(tls.getID())[0].getPhases():
                    total_duration += p.duration
                    if('r' in p.state[tl_concerned[0]:tl_concerned[-1]]):
                        red_duration += p.duration
                
                print(red_duration, total_duration, red_duration/total_duration)

