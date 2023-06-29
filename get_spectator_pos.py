import sys

try:
    sys.path.append('CARLA_0.9.10/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg')
        #'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

client = carla.Client('localhost', 2000)
client.set_timeout(2.0)

world = client.get_world()
if world.get_map().name != "Town10HD":
    world = client.load_world('Town10HD')

#input("Press enter to get the position")

spectator = world.get_spectator()
spectator_transform = spectator.get_transform()

print(spectator_transform)
# print(world.get_settings())

#blueprints = [bp for bp in world.get_blueprint_library().filter('*')]
#for blueprint in blueprints:
#   print(blueprint.id)
