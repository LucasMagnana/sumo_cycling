class Cyclist:

    def __init__(self, id, path, tab_cyclists):
        self.path = path
        self.id = id
        self.tab_cyclists = tab_cyclists

    def step(self, vehicle):
        if(str(self.id) not in vehicle.getIDList()):
            self.tab_cyclists.remove(self)
        
