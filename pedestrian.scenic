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

import scenic.simulators.carla.model as _carlaModel

fps = 10 #Simulation fps


global_state = 0 #To coordinate mutiple behaviors
num_ped = 0
num_car = 0
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
behavior LeavingPackagesBehaviorSingleA(box_destination, final_destination):
    global fps, global_state, obj_dumm
    state = 0
    past_time = 0
    take SetWalkAction(True)
    take SetWalkDestination(box_destination)
    while True:
    
        if state == 0: #Walk towards destination
            d = distance from self to box_destination
            if d < 2:
                take SetWalkingSpeedAction(0)
                state += 1
                past_time = simulation().currentTime
            else:
                take SetWalkingSpeedAction(1.4)
                #take SetWalkDestination(box)
        elif state == 1 and simulation().currentTime - past_time > Uniform(10,20)*fps: #Leave package
            obj_dumm = Object_Dummy(Uniform(*blueprints.boxModels),box_destination,0,0)
            simulation().createObjectInSimulator(obj_dumm)
            #simulation().objects.append(obj_dumm)
            actors_bb.append(obj_dumm)

            state += 1
            past_time = simulation().currentTime

        elif state == 2 and simulation().currentTime - past_time > Uniform(10,20)*fps: #Go to spawn point
            #take SetWalkingSpeedAction(1.4)
            take SetWalkDestination(final_destination)
            take SetWalkingSpeedAction(1.4)
            state += 1
        elif state == 3: #Despawn
            d = distance from self to final_destination
            if d < 2:
                idx = actors_bb.index(self)
                del actors_bb[idx]
                self.destroy(simulation())
                state += 1
                global_state += 1
            else:
                take SetWalkingSpeedAction(1.4)
            
            
        else:
            wait   
            
#Go to a location to retrieve a package, someone is waiting there
behavior LeavingPackagesBehaviorSingleB(destination, final_destination):
    
    global obj_dumm,global_state
    take SetWalkAction(True)
    take SetWalkDestination(destination)
    state = 0
    past_time = 0
    while True:
        if state == 0: #Wait until other pedestrian finishes
            take SetWalkingSpeedAction(0)
            state += 1
        elif state == 1 and global_state == 1: #Go towards package location
            d = distance from self to destination
            take SetWalkDestination(destination)
            if d < 2:
                take SetWalkingSpeedAction(0)
                past_time = simulation().currentTime
                state += 1
                print("state finished")

            else:
                take SetWalkingSpeedAction(1.4)

        elif state == 2 and simulation().currentTime - past_time > Uniform(10,20)*fps: #Take package
            idx = actors_bb.index(obj_dumm)
            del actors_bb[idx]
            obj_dumm.carlaActor.destroy()
            take SetWalkingSpeedAction(1.4)
            past_time = simulation().currentTime
            state += 1
            print(state)
        elif state == 3: #Go away
            take SetWalkDestination(final_destination)
            take SetWalkingSpeedAction(1.4)
            state += 1
        elif state == 4: #Despawn
            d = distance from self to final_destination
            if d < 2:
                idx = actors_bb.index(self)
                del actors_bb[idx]
                self.destroy(simulation())
                state += 1
                terminate
            else:
                take SetWalkingSpeedAction(1.4)
        else:
            wait
            

behavior SpawnDrop(box_destination, final_destination):
    global obj_dumm,global_state
    take SetWalkAction(True)
    take SetWalkDestination(box_destination)
    state = 0
    past_time = 0
    while True:
        if state == 0: #Wait some time before start moving
            take SetWalkingSpeedAction(0)
            past_time = simulation().currentTime
            state += 1
        elif state == 1 and simulation().currentTime - past_time > Uniform(1,60)*fps: #Move towards destination
            d = distance from self to box_destination
            #print(d)
            if d < 3:
                take SetWalkingSpeedAction(0)
                past_time = simulation().currentTime
                state += 1
                

            else:
                take SetWalkingSpeedAction(1.4)

        elif state == 2 and simulation().currentTime - past_time > Uniform(10,20)*fps: #Leave package
            obj_dumm = Object_Dummy(Uniform(*blueprints.boxModels),box_destination,0,0)
            simulation().createObjectInSimulator(obj_dumm)

            actors_bb.append(obj_dumm)

            past_time = simulation().currentTime
            state += 1
        elif state == 3: #Move towards spawn point
            take SetWalkDestination(final_destination)
            take SetWalkingSpeedAction(1.4)
            state += 1
        elif state == 4: #Disappear
            d = distance from self to final_destination
            if d < 2:
                idx = actors_bb.index(self)
                del actors_bb[idx]
                self.destroy(simulation())
                state += 1
                global_state += 1
                print(global_state)
                if global_state == 3:
                    terminate
            else:
                take SetWalkingSpeedAction(1.4)
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
            
            

behavior Flash_Robber(destination1, destination2):

    global global_state, num_ped
    
    
    
    if not self.carlaActor:
        num_ped -= 1
        return

    take SetWalkAction(True)
    take SetWalkDestination(destination1)

    state = 0
    past_time = simulation().currentTime
    while True:

        if state == 0: #Walk towards destination
            
        
            d = distance from self to destination1
            if d < 2.5 or simulation().currentTime - past_time > Uniform(50,60)*fps:
                take SetWalkingSpeedAction(0)
                state += 1
                
            else:
                take SetWalkingSpeedAction(3)

                
        elif state == 1: #Despawn
            idx = actors_bb.index(self)
            del actors_bb[idx]
            self.destroy(simulation())
            state += 1
            past_time = simulation().currentTime
            
            global_state += 1
            
            print(global_state, num_ped)
            
            if global_state == num_ped:
                terminate
            
        else:
            wait
        """
        elif state == 2 and simulation().currentTime - past_time > Uniform(10,20)*fps:
        

            
            ped2 = Pedestrian at self,
                with behavior Spawned2(destination1),
                with blueprint self.blueprint

        
            '''
            ped2 = Object_Dummy(self.blueprint,Range(destination1[0]-5,destination1[0]+5) @ Range(destination1[1]-5,destination1[1]+5),0,0)
            ped2.behavior = WalkForwardBehavior()
            simulation().createObjectInSimulator(ped2)
            simulation().agents.append(ped2)
            '''
            
            #pdb.set_trace()


            state += 1
        """
        
 
 
behavior Spawned(destination):
    print(self, self.carlaActor)
    take SetWalkAction(False)
    #take SetWalkDestination(destination)
    while True:
        #pdb.set_trace()
        #take SetWalkingSpeedAction(0)
        wait        
behavior Spawned2(destination):
    print(self, self.carlaActor)
    take SetWalkAction(True)
    #take SetWalkDestination(destination)
    while True:
        #pdb.set_trace()
        #take SetWalkingSpeedAction(0)
        wait


behavior PedTakeover(destination1, destination2):
    global global_state, num_ped
    
    if not self.carlaActor:
        num_ped -= 1
        return
    
    take SetWalkAction(True)
    take SetWalkDestination(destination1)

    state = 0
    past_time = simulation().currentTime
    while True:
        if state == 0: #Move to sidewalk point that is closest to street point
            
        
            d = distance from self to destination1
            if d < 30 or simulation().currentTime - past_time > Uniform(10,20)*fps:
                take SetWalkingSpeedAction(0)
                take SetWalkAction(False)

                state += 1
                print("changing state")
                
            else:
                take SetWalkingSpeedAction(3)

            
        elif state == 1: #Change direction towards street point
                take SetWalkingDirectionAction(angle from self to destination1)
                state += 1
                print("state")
                
                past_time = simulation().currentTime
                
        elif state == 2: #move to street point and wait
                d = distance from self to destination1
                if d < Uniform(1,4) or simulation().currentTime - past_time > Uniform(10,20)*fps:
                    take SetWalkingSpeedAction2(0)


                    state += 1
                    global_state += 1
                    print("changing state2", global_state)
                
                else:
                    take SetWalkingSpeedAction2(3)
                
        elif state == 3 and global_state >= num_ped + num_car: #Change direction to car destination
            #take SetWalkDestination(destination2)
            take SetWalkingDirectionAction(angle from self to destination2)
            state += 1
            past_time = simulation().currentTime
            
        elif state == 4: #Move to car destination
            d = distance from self to destination2
            if d < 2 or simulation().currentTime - past_time > Uniform(30,60)*fps:
                take SetWalkingSpeedAction2(0)
                global_state += 1
                state += 1
                
                idx = actors_bb.index(self)
                del actors_bb[idx]
                self.destroy(simulation())
                
                if global_state >= 2*num_ped + num_car:
                    terminate
                
            else:
                take SetWalkingSpeedAction2(3)
        else:
            wait


behavior CarTakeover(destination):
    global global_state
    state = 0
    
    agent = BasicAgent(self.carlaActor)
    
    car_planning(agent,self.carlaActor.get_transform().location, destination)
    #pdb.set_trace()
    #print(new_route_trace)
    
    

    print(agent._target_speed)
    
    """
    self.carlaActor.apply_control(agent.run_step())
    
    agent._target_speed = 0
    self.carlaActor.apply_control(agent.run_step())
    
    """
    while True:
        
        if state == 0: #Wait until pedestrians are in position
            if global_state >= num_ped:
                state += 1

        elif state == 1: # Move to destination
            control = agent._local_planner.run_step()
            self.carlaActor.apply_control(control)
            
            if agent.done():
                state += 1
                global_state += 1
                idx = actors_bb.index(self)
                del actors_bb[idx]
                self.destroy(simulation())
                
        
            
        wait
            
def car_planning(agent,current_position,destination):

    position = utils.scenicToCarlaLocation(destination,world=simulation().world)
    
    map = simulation().map

    chosen_waypoint = map.get_waypoint(position,project_to_road=True, lane_type=_carla.LaneType.Driving)
    current_waypoint = map.get_waypoint(current_position,project_to_road=True, lane_type=_carla.LaneType.Driving)
    new_route_trace = agent._trace_route(current_waypoint, chosen_waypoint)

    agent._local_planner.set_global_plan(new_route_trace)
    
    print("chosen_waypoint lane: ", chosen_waypoint.lane_id, chosen_waypoint.lane_type, "current_waypoint lane: ", current_waypoint.lane_id, current_waypoint.lane_type)


def car_planning2(agent,current_position,destination):

    
    map = simulation().map

    chosen_waypoint = destination
    current_waypoint = map.get_waypoint(current_position,project_to_road=True, lane_type=_carla.LaneType.Driving)
    new_route_trace = agent._trace_route(current_waypoint, chosen_waypoint)

    agent._local_planner.set_global_plan(new_route_trace)

behavior CrashCar(car_to_crash, final_destination):

    state = 0
    
    reverse_control = _carla.VehicleControl(reverse=True,throttle=1)
    
    agent = BasicAgent(self.carlaActor)
    car_planning(agent,self.carlaActor.get_transform().location, car_to_crash)

    self.carlaActor.apply_control(agent.run_step())
    state = 0
    past_time = 0
    while True:            
        if state == 0:           
            state += 1
        elif state == 1: #Crash with car
            control = agent._local_planner.run_step()
            self.carlaActor.apply_control(control)
            
            
            if agent.done(): #Arrived at location, now stop
                state += 1
                past_time = simulation().currentTime
                agent._target_speed = 0
                self.carlaActor.apply_control(agent.run_step())
                print("done")
                #ped = Pedestrian at 3.669543 @ -128.731094
                #actors_bb.append(ped)
                
                
                
        elif state == 2 and simulation().currentTime - past_time > Uniform(10,20)*fps: #Wait and then spawn pedestrian
            ped = Object_Dummy(Uniform(*blueprints.walkerModels),3.669543 @ -128.731094,0,0)
            simulation().createObjectInSimulator(ped)

            actors_bb.append(ped)
            past_time = simulation().currentTime
            state += 1
            
        elif state == 3 and simulation().currentTime - past_time > Uniform(10,20)*fps: #Wait and despawn pedestrian
            idx = actors_bb.index(ped)
            del actors_bb[idx]
            ped.carlaActor.destroy()
            past_time = simulation().currentTime
            state += 1
            
        elif state == 4 and simulation().currentTime - past_time > Uniform(10,20)*fps: #Apply reverse
            past_time = simulation().currentTime
            
            self.carlaActor.apply_control(reverse_control)
            
            state += 1
        elif state == 5 and simulation().currentTime - past_time > Uniform(5,8)*fps: #End reverse
            self.carlaActor.apply_control(_carla.VehicleControl())
            state += 1

            
        
        elif state == 6: #Go to a waypoint next to blocking car, adjacent lane

            map = simulation().map
            wps = map.generate_waypoints(distance=2.0)
            position = utils.scenicToCarlaLocation(car_to_crash,world=simulation().world)

            blocked_waypoint = map.get_waypoint(position,project_to_road=True, lane_type=_carla.LaneType.Driving)
            
            dist = 99999999
            temp_destination = []
            for wp in wps:
                if wp.road_id == blocked_waypoint.road_id:
                    if wp.lane_id != blocked_waypoint.lane_id:
                        #print(wp.transform.location)

                        temp_dist = np.linalg.norm(np.array([wp.transform.location.x,wp.transform.location.y]) - np.array([blocked_waypoint.transform.location.x,blocked_waypoint.transform.location.y]))
                        
                        if temp_dist < 10:
                            second_dist = np.linalg.norm(np.array([wp.transform.location.x,wp.transform.location.y])- np.array([self.carlaActor.get_transform().location.x,self.carlaActor.get_transform().location.y]))
                        
                            if second_dist < dist:
                                temp_destination = wp
                                dist = second_dist
                
            print(temp_destination.lane_id,temp_destination.road_id,blocked_waypoint.lane_id,blocked_waypoint.road_id)
            car_planning2(agent, self.carlaActor.get_transform().location, temp_destination)
            


            self.carlaActor.apply_control(agent.run_step())
            
            state += 1
            print("state 4")
        elif state == 7: #Go afterwards to final destination
            control = agent._local_planner.run_step()
            self.carlaActor.apply_control(control)
            
            if agent.done(): #Arrived at location, now stop
                state += 1
                past_time = simulation().currentTime
                car_planning(agent,self.carlaActor.get_transform().location, final_destination)
                self.carlaActor.apply_control(agent.run_step())
            
                
        elif state == 8:
            control = agent._local_planner.run_step()
            self.carlaActor.apply_control(control)
            
            if agent.done(): #Arrived at location, now stop
                state += 1
                past_time = simulation().currentTime
                self.carlaActor.apply_control(agent.run_step())
                terminate
            
        
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

            
        print(ped)
        actors_bb.append(ped)
        

def activate_cameras(output_dir,cameras, camera_function, bind_address):

    points_data = read_data("locations.txt")
    # print(cameras)
    camera_descriptions = [x for x in points_data if "tc" in x[-1] and int(x[-1][2:]) in cameras]
    # print("DESCRIPTION:")
    # print(camera_descriptions)
    # asdf
    stop_listening_event = threading.Event()

    for c in camera_descriptions:

        # print("HIHIHI")

        if camera_function == 'CameraBehavior':
                # print("Activating as 1")
                
                depth_camera = depthCamera at c[0] @ -c[1], 
                    with elevation c[2],
                    with pitch c[3],
                    with yaw c[4],
                    with roll 0
                '''
                lidar = Lidar at c[0] @ -c[1], 
                    with elevation c[2],
                    with pitch c[3],
                    with yaw c[4],
                    with roll 0
                '''
                rgbCamera at depth_camera, #lidar, #depth_camera,
                    with elevation c[2],
                    with pitch c[3],
                    with yaw c[4],
                    with roll 0,
                    with depth depth_camera, 
                    #with lidar lidar,
                    with camera_id int(c[-1][-1]),
                    with behavior CameraBehavior(output_dir)
        else:
                # print("Activating as 2")
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
    
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)

    scenarios = [flash_robbery, street_takeover, package_theft, coordinated_attack, hit_and_run] #[first_scenario,second_scenario,third_scenario,fourth_scenario, test_scenario]
    
    destination_locations,walkerModels = scenarios[num_scenario-1]()
    print("HIEGHIEGHIEGHIEH\n")
    
    if num_extra_pedestrians > 0:
        create_multitude(num_extra_pedestrians,destination_locations,walkerModels)
    
    if cameras_on:
        activate_cameras(output_dir=output_dir, cameras=cameras_on, camera_function='CameraBehavior', bind_address=bind_address)


#In this scenario, a pedestrian leaves a package and then leaves the scene. A second pedestrian comes afterwards and retrieves the package
def flash_robbery():

    global num_ped

    num_ped = 20
    
    walkerModels = blueprints.walkerModels
    """
    origin = [-63.698700, -34.847969]
    ped = Pedestrian at Range(origin[0]-5,origin[0]+5) @ Range(origin[1]-5,origin[1]+5),
        with behavior Flash_Robber(-12.127034 @ -38.057995,32.604286 @ -44.886600), 
        with blueprint bluep
        
    ego = Pedestrian at -12.127034 @ -38.057995,
        with behavior Spawned(32.604286 @ -44.886600), 
        with blueprint bluep
    """
        
    origin = [-63.698700, -34.847969]
    for n in range(num_ped):
        bluep = random.choice(walkerModels)
        ped = Pedestrian in CircularRegion(origin[0] @ origin[1],30), #at Range(origin[0]-5,origin[0]+5) @ Range(origin[1]-5,origin[1]+5),
            with behavior Flash_Robber(-12.127034 @ -38.057995,32.604286 @ -44.886600), 
            with blueprint bluep

        actors_bb.append(ped)
        if not n:
            ego = ped
        

       
    return [],[]

def street_takeover():

    global num_ped, num_car

    ped_origin = [(-52.700798,84.729713),(-50.499549484352116, 69.2086258548547), (-48.915406024869085, 70.79433171626007), (-51.41624495533474, 75.79557755585151), (-47.74294468540671, 70.11535935254894), (-57.15193991690292, 71.34674031511335), (-48.7788107160527, 71.64701092410759), (-54.17947556248375, 73.75608981035926), (-51.00525428014775, 74.23126559437019), (-53.40434563996765, 70.78113073519829), (-53.806692844257306, 76.3336974352782), (-53.569666250525145, 73.13202494443009), (-54.99849058058672, 71.66453029808088), (-49.422514464888984, 74.00039681637239), (-53.89118225983337, 69.58207233642119), (-52.564221097670654, 70.54383961838529), (-48.00288236501322, 75.35237660438187), (-56.134851649658145, 75.99626322721842), (-56.0403324871943, 71.93368918256245), (-57.15603586174784, 73.14657569602976)] #[-52.700798, 84.729713]
    ped_location1 = [-41.880692, 64.317993]
    vehicle_origin = [(-90.100082, 53.627705), (-94.329887, 46.639248), (-101.084084, 42.784473)]#[-88.674194, 57.855698]
    final_location = [-48.606792, -42.110130]
    
    walkerModels = blueprints.walkerModels

    
    
    center_point = Point at -52.700798 @ 84.729713
    
    num_ped = 20
    num_car = 3
    
    for n in range(num_ped):
        bluep = random.choice(walkerModels)
        ped = Pedestrian in CircularRegion(-52.700798 @ 84.729713,30),#visible from center_point, #-52.700798 @ 84.729713,100), #Range(ped_origin[n][0]-50,ped_origin[n][0]+50) @ Range(ped_origin[n][1]-50,ped_origin[n][1]+50),
            with behavior PedTakeover(ped_location1[0] @ ped_location1[1],final_location[0] @ final_location[1]), 
            with blueprint bluep

        actors_bb.append(ped)
        if not n:
            ego = ped
        
        
    
    for n in range(num_car):
        car = Car at Range(vehicle_origin[n][0]-10,vehicle_origin[n][0]+10) @ Range(vehicle_origin[n][1]-10,vehicle_origin[n][1]+10),
            with behavior CarTakeover(final_location[0] @ final_location[1])
        #ego = car
        actors_bb.append(car)
    
    """
    for ped_idx in range(2):
        ped = Pedestrian at Range(ped_origin[0]-5,ped_origin[0]+5) @ Range(ped_origin[1]-5,ped_origin[1]+5),
            with behavior PedTakeover(ped_location1[0] @ ped_location1[1],final_location[0] @ final_location[1]), 
            with blueprint bluep
        
    for car_idx in range(2):
        car = Car at Range(vehicle_origin[0]-10,vehicle_origin[0]+10) @ Range(vehicle_origin[1]-10,vehicle_origin[1]+10),
            with behavior CarTakeover(final_location[0] @ final_location[1])
    """
        
    return [],[]
    
    
def package_theft():
    global actors_bb
    


    walkerModels = blueprints.walkerModels



    bluep = walkerModels[0] #random.choice(walkerModels)
        
    walkerModels.remove(bluep)
    
    bluep2 = walkerModels[0]
    
    walkerModels.remove(bluep2)
    
    spawn_point = [-59.032146, 31.74700]
    spawn_location1 = Range(spawn_point[0]-10,spawn_point[0]+10) @ Range(spawn_point[1]-10,spawn_point[1]+10)
    spawn_location2 = Range(spawn_point[0]-10,spawn_point[0]+10) @ Range(spawn_point[1]-10,spawn_point[1]+10)
    
    box_location = -61.271282 @ 12.471437

    ped2 = Pedestrian at spawn_location1,
        with behavior LeavingPackagesBehaviorSingleA(box_location, spawn_location1), 
        with blueprint bluep #random.choice(walkerModels)
        
    ego = Pedestrian at spawn_location2,
        with behavior LeavingPackagesBehaviorSingleB(box_location,-71.100212 @ -5.161844),
        with blueprint bluep2 #random.choice(walkerModels)


    actors_bb = [ego, ped2]

    return [],[]
    

def coordinated_attack():
    
    
    global actors_bb
    


    walkerModels = blueprints.walkerModels

    spawn_locations = [[10.286928, -37.113400, -131.359329],[-6.142745,86.108368,-44.225304]]
    
    box_destinations = [26.043566 @ -7.181683, -24.393764 @ 89.643570, -122.396088 @ -55.014839]



    for sp_idx in range(len(spawn_locations[0])):
        
        bluep = random.choice(walkerModels)
        
        walkerModels.remove(bluep)
        
        sp_location = Range(spawn_locations[0][sp_idx]-10,spawn_locations[0][sp_idx]+10) @ Range(spawn_locations[1][sp_idx]-10,spawn_locations[1][sp_idx]+10)
        
        ped = Pedestrian at sp_location,
            with behavior SpawnDrop(box_destinations[sp_idx], sp_location), 
            with blueprint bluep
        if sp_idx == 2:
            ego = ped
            
        
        
        actors_bb.append(ped)
        
       
        
    

    return [],[]
    
def hit_and_run():

    print("HIT AND RUN")
    global actors_bb
    
    car_to_crash_location = [0.923356,-127.128075]
    
    car_to_crash = Car at Range(car_to_crash_location[0]-3,car_to_crash_location[0]+3) @ Range(car_to_crash_location[1]-3,car_to_crash_location[1]+3) #0.923356 @ -127.128075   

    final_destination = -40.722378 @ -103.120766
    

    
    ego = Car at 28.924257 @ -131.330811,
        with behavior CrashCar(car_to_crash, final_destination) 
     
    actors_bb = [ego, car_to_crash]
    
    return [],[]
        
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

if globalParameters.cameras_on:
    cameras_on_str = globalParameters.cameras_on.split(',')
    cameras_on = [int(x) for x in cameras_on_str]
else:
    cameras_on = []
    



run_scenario(num_scenario=int(globalParameters.num_scenario),num_extra_pedestrians=int(globalParameters.num_extra_pedestrians), output_dir=globalParameters.output_dir, bind_address=globalParameters.bind_address, cameras_on = cameras_on)




        

