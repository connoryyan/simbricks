from world import World


class FlatWhiteWorld(World):
    '''
    Simulate a white field with no obstacles.
    '''
    def __init__(self, reflection=85, *, ambient_light=5.0):
        super().__init__(ambient_light, reflection=reflection)
