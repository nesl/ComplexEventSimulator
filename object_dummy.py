class Object_Dummy:
    def __init__(self, blueprint, position, heading, elevation):
        self.blueprint = blueprint
        self.position = position
        self.bounding_box = None
        self.elevation = elevation
        self.carlaActor = None
        self.rolename = None
        self.id = None
        self.heading = heading
        self.pitch = None
        self.yaw = None
        self.roll = None

    def get_transform(self):
        return self.carlaActor.get_transform()
