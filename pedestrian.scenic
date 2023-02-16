#################################
# MAP AND MODEL                 #
#################################

from utils import read_data
import pdb
import math
import random
import socket
from threading import Thread
import threading

import sys
try:
    sys.path.append('CARLA_0.9.10/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg')
    sys.path.append('CARLA_0.9.10/PythonAPI/carla/')
    import carla as _carla
except ImportError as e:
    raise ModuleNotFoundError('CARLA scenarios require the "carla" Python package') from e

from object_dummy import Object_Dummy
import scenic.simulators.carla.utils.utils as utils
import scenic.simulators.carla.blueprints as blueprints

param map = localPath('./CARLA_0.9.10/CarlaUE4/Content/Carla/Maps/OpenDrive/Town10HD.xodr')
param carla_map = 'Town10HD'
model scenic.simulators.carla.model



fps = 10 #Simulation fps


global_state = 0 #To coordinate mutiple behaviors
obj_dumm = None #For boxes
actors_bb = [] #List with all objects (cars,pedestrians,boxes,etc) to be annotated with bounding boxes

#################################
# AGENT BEHAVIORS               #
#################################


#Make the pedestrian arrive to destination and then walk around it
behavior WalkAround(destination):
    global fps
    
    if self.carlaController:
        state = 0
        past_time = 0
        #destination = Uniform(10, -10)*destination
        take SetWalkAction(True)
        take SetWalkDestination(destination)
        threshold = Uniform(1,120)*fps
        while True:
            if state == 0:
                d = distance from self to destination
                if d < 2:
                    take SetWalkingSpeedAction(0)
                    state += 1
                    past_time = simulation().currentTime
                else:
                    wait
            elif state == 1 and simulation().currentTime - past_time > threshold:
                destination = Range(-94.278351,-60.667870) @ Range(29.682917,51.475655) #Uniform(10, -10)*destination
                take SetWalkingSpeedAction(1.4)
                take SetWalkDestination(destination)
                state -= 1
            else:

                wait
            
#Go and take a box
behavior StealingPackagesBehaviorSingle():
    global fps
    state = 0
    past_time = 0
    take SetWalkAction(True)
    take SetWalkDestination(box)
    while True:
    
        if state == 0:
            d = distance from self to box
            if d < 2:
                take SetWalkingSpeedAction(0)
                state += 1
                past_time = simulation().currentTime
            else:
                take SetWalkingSpeedAction(1.4)
                #take SetWalkDestination(box)
        elif state == 1 and simulation().currentTime - past_time > Uniform(10,20)*fps:
            idx = actors_bb.index(box)
            del actors_bb[idx]
            box.destroy(simulation())
            

            state += 1
            past_time = simulation().currentTime
        elif state == 2 and simulation().currentTime - past_time > Uniform(1,5)*fps:
            take SetWalkingSpeedAction(1.4)
            state += 1
            past_time = simulation().currentTime
        elif state == 3 and simulation().currentTime - past_time > Uniform(10,60)*fps:
            terminate
            
        else:
            wait
   



behavior WaitAndGo(destination):
    global global_state
    
    local_state = 0
    while True:
        if global_state == 1 and local_state == 0:
            take SetWalkAction(True)
            take SetWalkDestination(destination)
            local_state += 1
            
        wait

#Go to a location, leave a box and then leave
behavior LeavingPackagesBehaviorSingle(box_destination):
    global fps, global_state
    state = 0
    past_time = 0
    take SetWalkAction(True)
    take SetWalkDestination(box_destination)
    while True:
    
        if state == 0:
            d = distance from self to box_destination
            if d < 2:
                take SetWalkingSpeedAction(0)
                state += 1
                past_time = simulation().currentTime
            else:
                take SetWalkingSpeedAction(1.4)
                #take SetWalkDestination(box)
        elif state == 1 and simulation().currentTime - past_time > Uniform(10,20)*fps:
            obj_dumm = Object_Dummy(Uniform(*blueprints.boxModels),box_destination,0,0)
            simulation().createObjectInSimulator(obj_dumm)
            #simulation().objects.append(obj_dumm)
            actors_bb.append(obj_dumm)
            global_state += 1

            state += 1
            past_time = simulation().currentTime
        elif state == 2 and simulation().currentTime - past_time > 60*fps:
            state += 1
            past_time = simulation().currentTime
        elif state == 3 and simulation().currentTime - past_time > 60*fps:
            #take SetWalkingSpeedAction(1.4)
            take SetWalkDestination(67.793434 @ -3.075324)
            take SetWalkingSpeedAction(1.4)
            state += 1
            
            
        else:
            wait   
            

#Go and take package but with an additional animation when taking the package that makes it more realistic. Note here the use of both SetWalkingSpeedAction as well as SetWalkingSpeedAction2. They correspond to two different interfaces for pedestrian control. The SetWalkingSpeedAction2 let us set the speed at which a pedestrian arrives into a location if it was going in a straight line without any trajectory control, as opposed to the other action.
behavior StealingPackagesBehaviorSupplement(destination):
    global fps
    state = 0
    past_time = 0
    take SetWalkAction(True)
    take SetWalkDestination(destination)
    while True:
    
        if state == 0:
            d = distance from self to destination
            if d < 2:
                take SetWalkingSpeedAction(0)
                state += 1
                past_time = simulation().currentTime
                take SetWalkAction(False)
            else:
                take SetWalkingSpeedAction(1.4)

        elif state == 1 and simulation().currentTime - past_time > Uniform(10,20)*fps:

            
            d = distance from self to destination
            if d < 1:
                take SetWalkingSpeedAction2(0)
                idx = actors_bb.index(box2)
                del actors_bb[idx]
                box2.destroy(simulation())
      

                state += 1
                past_time = simulation().currentTime
                
            else:
                take SetWalkingSpeedAction2(1.4)
            

   
        elif state == 2 and simulation().currentTime - past_time > Uniform(1,5)*fps:
            take SetWalkAction(True)
            take SetWalkingSpeedAction(1.4)
            state += 1
            past_time = simulation().currentTime

            
        else:
            wait


#Randomly decide if aimlessly walk or steal a package            
behavior RandomActionPackage(destination):
    decision = Uniform(0,1)
    print(decision)
    #decision = 1
    if not decision:
        take SetWalkAction(True)
        while True:
            wait
    else:
        do StealingPackagesBehaviorSupplement(destination)
        

#Steal a package and then leave in a car
behavior StealingPackagesBehaviorSingleCar(destination,car):
    global fps,global_state
    state = 0

    past_time = 0
    take SetWalkAction(True)
    take SetWalkDestination(destination)
    while True:

        if state == 0:
            d = distance from self to destination
            if d < 2:
                take SetWalkingSpeedAction(0)
                take SetWalkAction(False)
                state += 1
                past_time = simulation().currentTime
            else:
                take SetWalkingSpeedAction(1.4)

                #take SetWalkDestination(box)
        elif state == 1 and simulation().currentTime - past_time > Uniform(10,20)*fps:
            d = distance from self to destination
            if d < 1:
                take SetWalkingSpeedAction2(0)
                idx = actors_bb.index(box)
                del actors_bb[idx]
                box.destroy(simulation())
                global_state += 0.5

                state += 1
                past_time = simulation().currentTime
                
            else:
                take SetWalkingSpeedAction2(1.4)
        elif global_state == 1 and state == 2 and simulation().currentTime - past_time > Uniform(1,5)*fps:
            #take SetWalkingSpeedAction(1.4)
            #take SetWalkDestination(car)
            #take SetWalkAction(False)
            take SetWalkingDirectionAction(angle from self to car)
            state += 1
        
        elif state == 3:
            #print(angle from self to car)
            
            take SetWalkingSpeedAction2(1.4)
            d = distance from self to car
            #print(d)
            if d < 2:
                global_state += 1
                state += 1
                idx = actors_bb.index(self)
                del actors_bb[idx]
                self.destroy(simulation())

                
            else:
                wait
 
        
        else:
            wait

#Leave a package in a certain location and wait for someone else to take it
behavior StealingPackagesBehaviorDoubleA(destination):
    
    global obj_dumm,global_state
    take SetWalkAction(True)
    take SetWalkDestination(destination)
    state = 0
    past_time = 0
    while True:
        if state == 0:
            d = distance from self to destination
            if d < 3:
                take SetWalkingSpeedAction(0)
                global_state += 0.5
                state += 1
                past_time = simulation().currentTime
                obj_dumm = Object_Dummy(Uniform(*blueprints.boxModels),destination,0,0)
                simulation().createObjectInSimulator(obj_dumm)
                #simulation().objects.append(obj_dumm)
                actors_bb.append(obj_dumm)
                #print(global_state)
            else:
                take SetWalkingSpeedAction(1.4)
        elif state == 1 and global_state == 1:
            state += 1
            past_time = simulation().currentTime
        elif state == 2 and simulation().currentTime - past_time > Uniform(10,20)*fps:
            take SetWalkingSpeedAction(1.4)
            state += 1
        else:
            wait
    
#Go to a location to retrieve a package, someone is waiting there
behavior StealingPackagesBehaviorDoubleB(destination):
    
    global obj_dumm,global_state
    take SetWalkAction(True)
    take SetWalkDestination(destination)
    state = 0
    past_time = 0
    while True:
        if state == 0:
            d = distance from self to destination

            if d < 2:
                take SetWalkingSpeedAction(0)
                global_state += 0.5
                state += 1

            else:
                take SetWalkingSpeedAction(1.4)
        elif state == 1 and global_state == 1:
            state += 1
            past_time = simulation().currentTime
        elif state == 2 and simulation().currentTime - past_time > Uniform(10,20)*fps:
            idx = actors_bb.index(obj_dumm)
            del actors_bb[idx]
            obj_dumm.carlaActor.destroy()
            take SetWalkingSpeedAction(1.4)
            past_time = simulation().currentTime
            state += 1
        elif state == 3 and simulation().currentTime - past_time > Uniform(30,60)*fps:
            terminate
        else:
            wait
        
#For bounding boxes and to store the images
behavior CameraBehavior(path):
    while True:
        take GetBoundingBox(actors_bb,path)

#For streaming images to server
behavior CameraStreamingBehavior(camera_id,server_socket,stop_listening_event,path):

    frame_index = 0
    while True:
    
        #print(self.connected, camera_id)
        if not self.connected:  
            
            server_connection,current_server_listening_thread = setupListeningServer(camera_id,server_socket,stop_listening_event)
            if server_connection:
                self.connected = True
    
        if self.connected:
            # MADE A CHANGE: This will always send the camera ID rather than the frame_index
            take SendImages(frame_index,camera_id,server_connection,current_server_listening_thread,stop_listening_event,path)
            #take SendImages(camera_id,camera_id,server_connection,current_server_listening_thread,stop_listening_event)    
            frame_index += 1
        else:
            wait
        
        
#Not used
behavior CarFollowingBehavior():

    try:
        do FollowLaneBehavior()
    interrupt when self.distanceToClosest(Pedestrian) < 20:
        take SetBrakeAction(1)
        
#Control over a car to get into a destination and then leave
behavior CarFoBehavior(destination):
    
    global fps, global_state
    
    #position = box.carlaActor.get_transform().location
    position = utils.scenicToCarlaLocation(destination,world=simulation().world)
    agent = BasicAgent(self.carlaActor)
    map = simulation().map

    chosen_waypoint = map.get_waypoint(position,project_to_road=True, lane_type=_carla.LaneType.Driving)
    current_waypoint = map.get_waypoint(self.carlaActor.get_transform().location,project_to_road=True, lane_type=_carla.LaneType.Driving)
    new_route_trace = agent._trace_route(current_waypoint, chosen_waypoint)
    #pdb.set_trace()
    #print(new_route_trace)
    agent._local_planner.set_global_plan(new_route_trace)
    self.carlaActor.apply_control(agent.run_step())
    state = 0
    past_time = 0
    while True:            
        if state == 0:           
            state += 1
        elif state == 1:
            control = agent._local_planner.run_step()
            self.carlaActor.apply_control(control)
            
            if agent.done(): #Arrived at location, now stop
                state += 1
                past_time = simulation().currentTime
                agent._target_speed = 0
                self.carlaActor.apply_control(agent.run_step())
                print("done")
                global_state += 0.5
        elif global_state == 2 and state == 2 and simulation().currentTime - past_time > Uniform(10,20)*fps: #After pedestrian gets in the car, drive over a random route and then terminate
            past_time = simulation().currentTime
            print("followLane")
            try:
                do FollowLaneBehavior()
            interrupt when simulation().currentTime - past_time > Uniform(10,20)*fps:
                terminate
            
        
        wait
    
#Not used
behavior CarFollowBehavior():
    #take GetPathVehicle(self,box)
    
    map = simulation().map
    agent = BasicAgent(self.carlaActor)
    position = box.carlaActor.get_transform().location
    chosen_waypoint = map.get_waypoint(position,project_to_road=True, lane_type=_carla.LaneType.Driving)
    current_waypoint = map.get_waypoint(self.carlaActor.get_transform().location,project_to_road=True, lane_type=_carla.LaneType.Driving)
    new_route_trace = agent._trace_route(current_waypoint, chosen_waypoint)

    route = []
    network_lanes = [n.id for n in network.lanes]
    road_lanes = {}
    for n in network.lanes:
        r_id = n.road.id
        if r_id in road_lanes:
            road_lanes[r_id] += 1
        else:
            road_lanes[r_id] = 1
 
    
    waypoints = simulation().map.generate_waypoints(distance=2.0)
    # Now we take our waypoints generated from the map, and get the corresponding roads.
    # road_dict is {road_id: [waypoint, waypoint, etc.]}
    road_dict = {}
    for wp in waypoints:
        if wp.road_id not in road_dict:
            road_dict[wp.road_id] = []
        if wp.lane_id not in road_dict[wp.road_id]:
            road_dict[wp.road_id].append(wp.lane_id)
        
    #for i in road_dict.keys():
    #    print(i,road_dict[i])

    idx = 0
    last_idx = -1
    #print(road_lanes)
    count = 0
    for i in new_route_trace:
        print(i[0].road_id,i[0].section_id,i[0].lane_id,i[0].s)
        
        lane_id = i[0].lane_id+math.floor(road_lanes[i[0].road_id]/2.0)
        
        if road_lanes[i[0].road_id] == 1:
            lane_id = 0
        elif i[0].lane_id > 0:
            lane_id -= 1
        

        lane_id = road_dict[i[0].road_id].index(i[0].lane_id)
       
        road_name = 'road'+str(i[0].road_id)+'_'+'lane'+str(lane_id)
        
        if road_name in network_lanes:
            idx = network_lanes.index(road_name)

            if idx != last_idx:
                if count == 1:
                    del route[-1]
                route.append(network.lanes[idx])
                count = 0
            count += 1
            last_idx = idx
        else:
            print("bad", road_name, i[0].lane_id, i[0].road_id, road_lanes[i[0].road_id], math.floor(road_lanes[i[0].road_id]/2.0))
        
    #take TrackWaypointsAction(new_route_trace)
    print(route)
    try:
        do FollowTrajectoryBehavior(trajectory=route,target_speed=5)
    interrupt when self.distanceToClosest(Box) < 20:
        take SetBrakeAction(1)
        print("yaa")

    #while True:
    #    wait
    
    
#Pedestrian walking around until it starts running from a certain location
behavior BombScare(destination,box_destination):
    global global_state
    
    try:
        do WalkAround(destination)
    interrupt when global_state == 1:
        take SetWalkAction(False)
        #print(angle from self to box_destination)
        take SetWalkingDirectionAction(math.pi/2 - (angle from self to box_destination))
        take SetWalkingSpeedAction2(3)
        while True:
            wait
        
#Pedestrian that leaves a package in a crowded area
behavior Terrorist(destination):
    
    global obj_dumm,global_state
    take SetWalkAction(True)
    take SetWalkDestination(destination)
    state = 0
    past_time = 0
    while True:
        if state == 0:
            d = distance from self to destination
            if d < 2:
                take SetWalkingSpeedAction(0)
                
                state += 1
                past_time = simulation().currentTime
                obj_dumm = Object_Dummy(Uniform(*blueprints.boxModels),destination,0,0)
                simulation().createObjectInSimulator(obj_dumm)
                #simulation().objects.append(obj_dumm)
                actors_bb.append(obj_dumm)
                #print(global_state)
            else:
                take SetWalkingSpeedAction(1.4)
        elif state == 1:
            state += 1
            past_time = simulation().currentTime
        elif state == 2 and simulation().currentTime - past_time > Uniform(10,20)*fps:
            take SetWalkingSpeedAction(1.4)
            global_state += 1
            state += 1
        else:
            wait    
#################################
# SCENARIO SPECIFICATION        #
#################################

def create_multitude(num_pedestrians,destination_locations,walkerModels):
    global actors_bb

    divisor = num_pedestrians / len(destination_locations)
    destination_idx = 0
    divisor_mult = divisor


    for i in range(num_pedestrians):
        if i >= divisor_mult:
            destination_idx +=1
            divisor_mult = divisor*(destination_idx+1)
        
        ped = Pedestrian in sidewalk,
            with behavior WalkAround(destination_locations[destination_idx]),
            with blueprint random.choice(walkerModels)

            
        actors_bb.append(ped)
        

def activate_cameras(output_dir,cameras, camera_function, bind_address):

    points_data = read_data("locations.txt")
    camera_descriptions = [x for x in points_data if "tc" in x[-1] and int(x[-1][2:]) in cameras]

    stop_listening_event = threading.Event()

    for c in camera_descriptions:

        if camera_function == 'CameraBehavior':
                depth_camera = depthCamera at c[0] @ -c[1], 
                    with elevation c[2],
                    with pitch c[3],
                    with yaw c[4],
                    with roll 0

                rgbCamera at depth_camera,
                    with elevation c[2],
                    with pitch c[3],
                    with yaw c[4],
                    with roll 0,
                    with depth depth_camera, 
                    with camera_id int(c[-1][-1]),
                    with behavior CameraBehavior(output_dir)
        else:
                camera_id = int(c[-1][-1])
                print(camera_id)
                server_socket = setup_connections_and_handling(camera_id, bind_address)
                
                
                #Review this camera_id
                rgbCamera at c[0] @ -c[1],
                    with elevation c[2],
                    with pitch c[3],
                    with yaw c[4],
                    with roll 0,
                    with camera_id camera_id,
                    with connected False,
                    with behavior CameraStreamingBehavior(camera_id,server_socket,stop_listening_event,output_dir)




def run_scenario(num_scenario=0,cameras_on=[], num_extra_pedestrians=0, output_dir=".", bind_address=''):
    
    
    scenarios = [first_scenario,second_scenario,third_scenario,fourth_scenario, test_scenario]
    
    destination_locations,walkerModels = scenarios[num_scenario]()
    
    if num_extra_pedestrians > 0:
        create_multitude(num_extra_pedestrians,destination_locations,walkerModels)
    

    if cameras_on:
        activate_cameras(output_dir=output_dir, cameras=cameras_on, camera_function='', bind_address=bind_address)


#In this scenario, a pedestrian leaves a package and then leaves the scene. A second pedestrian comes afterwards and retrieves the package
def first_scenario():
    global actors_bb
    
    destination_locations = [Range(-94.278351,-60.667870) @ Range(29.682917,51.475655),Range(66.135605,93.087204) @ Range(-5.828710,14.528165),Range(-11.539524,20.287785) @ Range(74.251755,108.597641),Range(-138.710266,-122.311989) @ Range(60.340248,85.676659)]

    walkerModels = blueprints.walkerModels



    bluep = walkerModels[0] #random.choice(walkerModels)
        
    walkerModels.remove(bluep)
    
    bluep2 = walkerModels[0]
    
    walkerModels.remove(bluep2)

    ego = Pedestrian at 67.793434 @ -3.075324,
        with behavior LeavingPackagesBehaviorSingle(Range(-10,1) @ Range(-5,-5.5)), 
        with blueprint bluep #random.choice(walkerModels)
        
    ped2 = Pedestrian at 70.793434 @ -3.075324,
        with behavior WaitAndGo(-31.996809 @ -3.644337),
        with blueprint bluep2 #random.choice(walkerModels)


    actors_bb = [ego, ped2]

    return destination_locations,walkerModels
    
#Package gets exchanged between two pedestrians at a certain point
def second_scenario():
    global actors_bb
    
    destination_locations = [Range(-94.278351,-60.667870) @ Range(29.682917,51.475655),Range(66.135605,93.087204) @ Range(-5.828710,14.528165),Range(-11.539524,20.287785) @ Range(74.251755,108.597641),Range(-138.710266,-122.311989) @ Range(60.340248,85.676659)]

    walkerModels = blueprints.walkerModels
    box_destination = Uniform(*destination_locations)

    bluep = random.choice(walkerModels)
    two_ped = Pedestrian  in sidewalk, #at -94.900848 @ 24.582340,
        with behavior StealingPackagesBehaviorDoubleA(box_destination),
        with blueprint bluep
        
    walkerModels.remove(bluep)
    bluep = random.choice(walkerModels)
    ego = Pedestrian in sidewalk, #at -59.610615 @ 23.937918,
        with behavior StealingPackagesBehaviorDoubleB(box_destination),
        with blueprint bluep


    walkerModels.remove(bluep)

    destination = -75.037537 @ 49.348728


    actors_bb = [ego,two_ped]
    
    return destination_locations,walkerModels


#Terrorist attack
def third_scenario():
    global actors_bb
    
    destination_locations = [Range(-94.278351,-60.667870) @ Range(29.682917,51.475655),Range(66.135605,93.087204) @ Range(-5.828710,14.528165),Range(-11.539524,20.287785) @ Range(74.251755,108.597641),Range(-138.710266,-122.311989) @ Range(60.340248,85.676659)]

    walkerModels = blueprints.walkerModels
    box_destination = Uniform(*destination_locations)

    bluep = random.choice(walkerModels)
    ego = Pedestrian in sidewalk,
        with behavior Terrorist(box_destination),
        with blueprint bluep

    actors_bb = [ego]
    walkerModels.remove(bluep)
    for i in range(100):
        ped = Pedestrian in sidewalk,
            with behavior BombScare(Range(-94.278351,-60.667870) @ Range(29.682917,51.475655),box_destination),
            with blueprint random.choice(walkerModels)

        #if ped.carlaActor is not None:
        actors_bb.append(ped)
        
        
    return destination_locations,walkerModels



#Steal package and escape with car
def fourth_scenario():
    global actors_bb
    
    destination_locations = [Range(-94.278351,-60.667870) @ Range(29.682917,51.475655),Range(66.135605,93.087204) @ Range(-5.828710,14.528165),Range(-11.539524,20.287785) @ Range(74.251755,108.597641),Range(-138.710266,-122.311989) @ Range(60.340248,85.676659)]
    box_destination = Uniform(*destination_locations)

    

    walkerModels = blueprints.walkerModels



    #Second scenario
    car = Car with behavior CarFoBehavior(box_destination)
        #with behavior FollowTrajectoryBehavior(trajectory=[network.lanes[0],network.lanes[38]])


    box = Box at box_destination #Range(-10,1) @ Range(-5,-5.5) #19.195606 @ -2.534948 #39.821236 @ 11.909901 #box_destination

    box_destination2 = Uniform(*destination_locations)

    box2 = Box at box_destination2

    bluep = walkerModels[0] #random.choice(walkerModels)


    ego = Pedestrian in sidewalk, #at -94.900848 @ 24.582340,
        with behavior StealingPackagesBehaviorSingleCar(box_destination,car),
        with blueprint bluep

        
    walkerModels.remove(bluep)

    ped2 = Pedestrian at 67.793434 @ -3.075324,
        with behavior RandomActionPackage(box_destination2),
        with blueprint random.choice(walkerModels)


    actors_bb = [ego,box,car,ped2,box2]
    
    
     
    return destination_locations,walkerModels



def setup_connections_and_handling(camera_id, bind_address):

        # Insert our networking stuff
        print("Setting up Server...", camera_id)
        server_connections = []
        
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.setblocking(0)
        server_socket.bind((bind_address, 55000+camera_id))
        server_socket.listen()

        
        
        return server_socket


def listenHandler(camera_id, server_connection, event):

        size = 4096
        global ZOOM_PARAMETERS
        while True:

            if event.is_set():
                break

            try:
                data = str(server_connection.recv(size).decode("utf8"))

                # Other preprocessing stuff
                # Remove all newlines and tabs
                data = data.replace("\n", "")
                data = data.replace("\t", "")
                data = data.replace(" ", "")
                # re.sub('\\s+', ' ', data)

                if data and "offset" in data:

                    # print(repr(data))

                    if data[0] != "{":
                        data_start_index = data.index("{")
                        data = data[data_start_index:]
                    if data[-1] != "}":
                        data_end_index = data[::-1].index("}")
                        data = data[:data_end_index+1]
                    data = data.lstrip('\x00') # Remove null bytes

                    # print(repr(data))

                    received_data = json.loads(data)

                    # print("hi2 " + str(data))
                    offset_data = received_data["offset(left/top)"]

                    current_frame_id = received_data["frame"]

                    # Add our received timestamp
                    # received_frame_latencies[str(current_frame_id)].append(time.time())
                    # print(time.time() - received_frame_latencies[str(current_frame_id)][0])

                    # print(offset_data)

                    # If we have offset information, we can move the cameras
                    if len(offset_data) > 0:

                        # Pick the smallest track id
                        print(received_data["others"])

                        # HERE we track only the first object that shows up in a list
                        # print("Offset data: " + str(offset_data[0]))
                        left_offset = offset_data[0][0]
                        top_offset = offset_data[0][1]

                        # Determine the direction of pitch and yaw
                        yaw_direction = 1 if left_offset > 0.5 else -1 # move along X axis
                        pitch_direction = -1 if top_offset > 0.5 else 1 # move along Y axis

                        change_magnitude = 1.5
                        yaw_update = yaw_direction * change_magnitude
                        pitch_update = pitch_direction * change_magnitude

                        # Also, check how big the object is in the video
                        relative_area = received_data["areas"][0]
                        zoom_in = False
                        if relative_area < 0.25: # Less than 10th of the frame
                            zoom_in = True

                        # Now we alter the cameras and spectator
                        current_camera = camera_id
                        new_rotation_update = (pitch_update, yaw_update, 0)

                        # Perform the rotation
                        # new_transform = rotate_actor(current_camera, new_rotation_update, zoom_in)
                        # rotate_actor(spectator, new_rotation_update, zoom_in)

                        # Perform zoom in
                        # if zoom_in:
                        #     # Get new attributes
                        #     # blueprint_library = world.get_blueprint_library()
                        #     # rgb_camera_bp_new = blueprint_library.find('sensor.camera.rgb')
                        #     # carla.command.DestroyActor(camera_list[0][0])
                        #     # position_actor(world, rgb_camera_bp_new, [], new_transform)
                        #     # print("Placing new camera!")
                        #     print("ZOOM SET TO TRUE")
                        #     ZOOM_PARAMETERS[0] = True
                        # else:
                        #     ZOOM_PARAMETERS[0] = False

            except Exception as e:
                pass
                # return False
                # print("\n*****HERE****\n")
                #print(e)
                # asfd
                # return False
                # Handle logic for listening for commands

def setupListeningServer(camera_id, server_socket,stop_listening_event):
        #print("Setting up listening handler")
        
        try:
            server_connection, addr = server_socket.accept()
        except:
            return 0,0

        # server_socket.connect(("127.0.0.1", 55000))
        print("Server set up...")
        server_connection.setblocking(0) #Non blocking socket
        
        #current_listening_thread = Thread(target = listenHandler, args = (camera_id, server_connection, stop_listening_event))
            
        #current_listening_thread.start()
        current_listening_thread = []
        
        return server_connection,current_listening_thread

def test_scenario():

    cameras = [1,2]
    # cameras = [1,2,3]
    points_data = read_data("locations.txt")
    camera_descriptions = [x for x in points_data if "tc" in x[-1] and int(x[-1][2:]) in cameras]
    
    print(camera_descriptions)

    stop_listening_event = threading.Event()

    path = '' #'./camera_image'
    #ego = Pedestrian in sidewalk
    
    for c in camera_descriptions:
        camera_id = int(c[-1][-1])
        print(camera_id)
        server_socket = setup_connections_and_handling(camera_id)
        
        
        #Review this camera_id
        rgbCamera at c[0] @ -c[1],
            with elevation c[2],
            with pitch c[3],
            with yaw c[4],
            with roll 0,
            with camera_id camera_id,
            with connected False,
            with behavior CameraStreamingBehavior(camera_id,server_socket,stop_listening_event,path)



    destination_locations,walkerModels = first_scenario()
    
    return destination_locations,walkerModels

"""
select_lane = ""
for lane in network.lanes:
    if lane.id == "road5_lane1":
        select_lane = lane
"""
  


"""
for i in range(100):
    ped = Pedestrian in sidewalk,
        with behavior WalkAround(Range(-94.278351,-60.667870) @ Range(29.682917,51.475655)),
        with blueprint random.choice(walkerModels)

    #if ped.carlaActor is not None:
    actors_bb.append(ped)
"""

cameras_on_str = globalParameters.cameras_on.split(',')
cameras_on = [int(x) for x in cameras_on_str]



run_scenario(num_scenario=int(globalParameters.num_scenario),num_extra_pedestrians=int(globalParameters.num_extra_pedestrians), output_dir=globalParameters.output_dir, bind_address=globalParameters.bind_address, cameras_on = cameras_on)




        

