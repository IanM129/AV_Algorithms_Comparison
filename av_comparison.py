import sys
import os
import math
import traci
import traci.constants as tc
import matplotlib.pyplot as plt

# Vars
vehicles = None
platoon = {}
in_front = {}
leaders = {}

# Metrics
arrived = {}



# Parameters
STEP_LENGTH = 0.1
PLATOON_CAPACITY = 6
PLATOON_DETECTDISTANCE = 20.0
PLATOON_GAP = 10.0              # how far to start matching speed
PLATOON_MAX_GAP = 50.0
PLATOON_TLSDISTANCE = 10.0      # TLS distance threshold
PLATOON_SPDTHRESHOLD = 5        # keep accelerating to leader if they
                                # are moving slower than this

SPEEDWAVES_FOLLOWCHECKDISTANCE = 10.0
SPEEDWAVES_FOLLOWERGAP = 5.0
SPEEDWAVES_DISTANCE = 50.0



def drawPolygon(poly_id, pos_a, pos_b, color=(255, 255, 255, 255), fill=False, lineWidth=0.1, layer=1):
    if poly_id in traci.polygon.getIDList():
        traci.polygon.setShape(poly_id, [pos_a, pos_b])
    else:
        traci.polygon.add(poly_id, [pos_a, pos_b],
                        color=color, fill=fill,
                        lineWidth=lineWidth, layer=layer)
def thick_segment_polygon(p0, p1, half_width):
    x0, y0 = p0
    x1, y1 = p1
    dx = x1 - x0
    dy = y1 - y0
    denom = abs(dx) + abs(dy)
    if denom == 0: return None
    scale = half_width / denom
    px = -dy * scale
    py =  dx * scale
    return [
        (x0 + px, y0 + py), (x1 + px, y1 + py),
        (x1 - px, y1 - py), (x0 - px, y0 - py)
    ]




def isEdgeIntersection(edge_id):
    return edge_id.startswith(":");

# Get the front leader of a platoon
def getFrontLeader(cur_id):
    last_id = cur_id
    while (cur_id != None):
        if (cur_id not in platoon):
            return cur_id
        cur_id = platoon[cur_id]
def getChainLength(cur_id, leader):
    length = 0
    while (cur_id != leader and cur_id != None):
        cur_id = in_front[cur_id]
        length += 1
    if cur_id == None: return -1;
    return length;
def getCurrentEdgeInRoute(veh_id):
    route = traci.vehicle.getRoute(veh_id)
    r_index = traci.vehicle.getRouteIndex(veh_id)
    return route[r_index]
def getNextEdge(veh_id, delta=1):
    route = traci.vehicle.getRoute(veh_id)
    r_index = traci.vehicle.getRouteIndex(veh_id)
    if r_index + delta < len(route):
        return route[r_index + delta]
    else: return None
def getPrevEdge(veh_id, delta=1):
    route = traci.vehicle.getRoute(veh_id)
    r_index = traci.vehicle.getRouteIndex(veh_id)
    if r_index - delta >= 0:
        return route[r_index - delta]
    else: return None
def distanceToTLS(veh_id):
    edge = traci.vehicle.getRoadID(veh_id)
    if isEdgeIntersection(edge): return 0;  # already inside intersection
    route = traci.vehicle.getRoute(veh_id)
    index = traci.vehicle.getRouteIndex(veh_id)
    if index + 1 >= len(route): return -1;  # no next intersection
    next_edge = route[index + 1]
    if not isEdgeIntersection(next_edge): return -1;  # next edge is not intersection
    veh_dis = traci.vehicle.getDistance(veh_id)
    edge_len = traci.edge.getLength(edge)
    return edge_len - veh_dis
def nextTLS(veh_id):
    tls_info = traci.vehicle.getNextTLS(veh_id)
    if not tls_info: return None;
def matchSpeedAndAcceleration(veh_id, platoon_leader_id, delta=0):
    speed = traci.vehicle.getSpeed(platoon_leader_id)
    acc = traci.vehicle.getAcceleration(platoon_leader_id);
    #traci.vehicle.setSpeedMode(veh_id, 0)
    traci.vehicle.setSpeed(veh_id, speed) #+ (STEP_LENGTH * acc))
# Platooning
def platooning(veh_id, arrived):
    in_platoon = False
    # Get vehicle in front
    leader = traci.vehicle.getLeader(veh_id, PLATOON_DETECTDISTANCE)
    in_front_of_intersection = False
    if leader:
        lead_id, distance = leader;
        in_front[veh_id] = lead_id
        if distance < PLATOON_DETECTDISTANCE: #and veh_id not in leaders:
            # Platoon only if:
            #   > far from the next traffic light
            #   > traffic light is red
            #   > if traffic light is green check if leader is going the same next edge
            #   > if no upcoming traffic light
            # Check if they are on the same route/edge
            veh_e = traci.vehicle.getRoadID(veh_id);
            lead_e = traci.vehicle.getRoadID(lead_id);
            veh_ce = getCurrentEdgeInRoute(veh_id)
            lead_ce = getCurrentEdgeInRoute(lead_id)
            veh_ne = getNextEdge(veh_id);
            lead_ne = getNextEdge(lead_id);
            veh_pe = getPrevEdge(veh_id);
            lead_pe = getPrevEdge(lead_id);
            #print(veh_id, " -> ", lead_id, ":\n  ", veh_pe, " - ", veh_ce, " - ", veh_ne, "\n  ", lead_pe, " - ", lead_ce, " - ", lead_ne)
            # Check if inside traffic light -> don't change anything
            if (isEdgeIntersection(veh_e)):
                in_platoon = veh_id in platoon;
            else:
                # Get distance to upcoming traffic light
                tlsId = -1; tls_distance = -1; tls_state = "";
                tls_info = traci.vehicle.getNextTLS(veh_id)
                if (tls_info):
                    tlsId, _, tls_distance, tls_state = tls_info[0]
                if tls_distance == -1 or tls_distance > PLATOON_TLSDISTANCE:
                    # Just check if they are on the same edge
                    in_platoon = (veh_ce == lead_ce)
                else:
                    in_front_of_intersection = True
                    # Check if traffic light is red
                    if (tls_state == "r"):
                        # Only switch platoon leader if car in front is at same edge
                        in_platoon = (veh_ce == lead_ce)
                    else:
                        in_platoon = (veh_ce == lead_ce and veh_ne == lead_ne) or (veh_ne == lead_ce and veh_ce == lead_pe);

                if (in_platoon):
                    # Set it as leader of current vehicle
                    platoon_leader = getFrontLeader(lead_id)
                    platoon_size = getChainLength(veh_id, platoon_leader)
                    #if (platoon_leader in leaders): platoon_size = leaders[platoon_leader];
                    #else: platoon_size = 0;
                    if platoon_size >= PLATOON_CAPACITY:
                        in_platoon = False;
                        if (veh_id in platoon): del platoon[veh_id];
                    else:
                        platoon[veh_id] = platoon_leader;
                        #if (platoon_leader not in leaders): leaders[platoon_leader] = 1;
                        #else: leaders[platoon_leader] += 1
    else: in_front[veh_id] = None;

    if in_platoon and veh_id in platoon:
        platoon_leader = platoon[veh_id]
        if platoon_leader not in vehicles:
            del platoon[veh_id];
            traci.vehicle.setSpeedMode(veh_id, 31) # default
            traci.vehicle.setSpeed(veh_id, -1) # default
            traci.vehicle.setLaneChangeMode(veh_id, 1621) # default
            return;
        speed = traci.vehicle.getSpeed(veh_id)
        platoon_leader_speed = traci.vehicle.getSpeed(platoon_leader)
        platoon_leader_accel = traci.vehicle.getAcceleration(platoon_leader)
        platoon_leader_maxdecel = traci.vehicle.getDecel(platoon_leader)
        secure_gap = traci.vehicle.getSecureGap(veh_id, speed, platoon_leader_speed, platoon_leader_maxdecel)
        # Set speed and speed mode:
        pos1 = traci.vehicle.getPosition(veh_id)
        pos2 = traci.vehicle.getPosition(in_front[veh_id])
        distance = pow(pos1[0] - pos2[0], 2) + pow(pos1[1] - pos2[1], 2)
        # if next to car in front follow leader speed
        if distance <= pow(PLATOON_GAP + secure_gap, 2):
            traci.vehicle.setSpeedMode(veh_id, 0)
            traci.vehicle.setLaneChangeMode(veh_id, 0) # Disable lane changing
            matchSpeedAndAcceleration(veh_id, platoon_leader, 0)
        # if far away from the car drive normally
        else:
            traci.vehicle.setSpeedMode(veh_id, 0)
            traci.vehicle.setSpeed(veh_id, -1)
            if in_front_of_intersection: traci.vehicle.setLaneChangeMode(veh_id, 0);
            else: traci.vehicle.setLaneChangeMode(veh_id, 1621);
    else:
        traci.vehicle.setSpeedMode(veh_id, 31) # default
        traci.vehicle.setSpeed(veh_id, -1) # default
        traci.vehicle.setLaneChangeMode(veh_id, 1621) # default
        if veh_id in platoon:
            #platoon_leader = platoon[veh_id];
            del platoon[veh_id];
            #leaders[platoon_leader] -= 1
            #if (leaders[platoon_leader] == 0): del leaders[platoon_leader];


def colorPlatoonMembers(id_list, leader_color = (255, 255, 192, 255), follow_color = (0, 255, 255, 255)):
    leaders = set()
    for l_id in platoon.values():
        leaders.add(l_id)
    for veh_id in id_list:
        if veh_id in platoon:
            traci.vehicle.setColor(veh_id, follow_color)
        elif veh_id in leaders:
            traci.vehicle.setColor(veh_id, leader_color)
        else:
            traci.vehicle.setColor(veh_id, (255, 255, 255, 255))
def drawPlatoonConnections(id_list, arrived, color = (255, 0, 0, 255)):
    for veh_id in arrived:
        poly_id = "platoonConnection" + str(veh_id)
        if poly_id in traci.polygon.getIDList(): traci.polygon.remove(poly_id);
    for veh_id in id_list:
        poly_id = "platoonConnection" + str(veh_id)
        if veh_id in platoon:
            veh_pos = traci.vehicle.getPosition(veh_id)
            lead_pos = traci.vehicle.getPosition(in_front[veh_id]) #platoon[veh_id])
            drawPolygon(poly_id, veh_pos, lead_pos,
                        color=color, fill=False,
                        lineWidth=0.1, layer=1000)
        else:
            if poly_id in traci.polygon.getIDList(): traci.polygon.remove(poly_id);

                        

#### Speed waves
def getRemainingTLSPhaseTime(tls_id):
    next_switch_time = traci.trafficlight.getNextSwitch(tls_id)
    return next_switch_time - traci.simulation.getTime()
def timeUntilNextGreen(tls_id, lane_index):
    phases = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_id)[0].phases
    current_phase_index = traci.trafficlight.getPhase(tls_id)
    time_remaining = getRemainingTLSPhaseTime(tls_id)
    for i in range(1, len(phases)):
        index = (current_phase_index + i) % len(phases)
        if (phases[index].state[lane_index].lower() == "g"): break;
        time_remaining += phases[index].duration
    return time_remaining
def wantsToMergeIntoMyLane(veh_id, other_id):
    my_lane = traci.vehicle.getLaneID(veh_id)
    other_lane = traci.vehicle.getLaneID(other_id)
    if my_lane == other_lane: return False; # same lane
    # Get next lane the other vehicle wants to go to
    other_route = traci.vehicle.getRoute(other_id)
    other_route_index = traci.vehicle.getRouteIndex(other_id)
    if other_route_index + 1 >= len(other_route): return False; # end of route
    next_edge = other_route[other_route_index + 1]
    next_lanes = traci.edge.getLaneIDs(next_edge)
    return my_lane in next_lanes
def isBlockingMergeing(veh_id, min_gap=2.5, look_distance=50):
    neighbors = traci.vehicle.getNeighbors(veh_id, look_distance)
    # check left and right (+ back)
    for side in ["LB", "RB", "L", "R"]:
        if side in neighbors and neighbors[side] is not None:
            other_id, gap, speed = neighbors[side]
            if wantsToMergeIntoMyLane(veh_id, other_id) and gap < min_gap:
                return True, other_id
    return False, None
def speedWaves(veh_id):
    speedWave = False
    # Check if not in intersection
    veh_e = traci.vehicle.getRoadID(veh_id);
    if (not isEdgeIntersection(veh_e)):
        # Get next traffic light
        tls_info = traci.vehicle.getNextTLS(veh_id)
        if (tls_info):
            tls_id, tls_laneIndex, tls_distance, tls_state = tls_info[0]
            traci.vehicle.setSpeedMode(veh_id, 31)
            if (tls_distance <= SPEEDWAVES_DISTANCE):
                if tls_state.lower() == "g":
                    # If green -> see if you can make it at full speed, otherwise wait for next
                    can_make = False
                    lane_max_speed = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(veh_id))
                    cur_speed = traci.vehicle.getSpeed(veh_id)
                    max_accel = traci.vehicle.getAccel(veh_id)
                    green_duration = getRemainingTLSPhaseTime(tls_id)
                    target_speed = tls_distance / (green_duration + 0.01)
                    if (target_speed <= lane_max_speed):
                        accel_time = (lane_max_speed - cur_speed) / max_accel
                        accel_dis = (cur_speed * accel_time) + (0.5 * max_accel * pow(accel_time, 2));
                        if accel_dis >= tls_distance:
                            time_taken = (-cur_speed + math.sqrt(pow(cur_speed, 2) + (2 * max_accel * tls_distance))) / max_accel
                        else:
                            remain_dis = tls_distance - accel_dis
                            time_taken = accel_time + (remain_dis / lane_max_speed)
                        if (time_taken <= green_duration):
                            traci.vehicle.setSpeed(veh_id, lane_max_speed)
                            can_make = True
                    if not can_make:
                        # Go as fast as needed to only make it to the next green
                        time_until_green = timeUntilNextGreen(tls_id, tls_laneIndex) + 1
                        speed = tls_distance / time_until_green
                        traci.vehicle.setSpeed(veh_id, speed)
                else:
                    # Check if blocking someone from behind who's in an intersection
                    follower = traci.vehicle.getFollower(veh_id, SPEEDWAVES_FOLLOWCHECKDISTANCE)
                    if (follower[0] != "" and follower[1] < SPEEDWAVES_FOLLOWERGAP):
                        follower_e = traci.vehicle.getRoadID(follower[0])
                        if (isEdgeIntersection(follower_e)):
                            traci.vehicle.setSpeed(veh_id, -1);
                            return;
                    # Check if blocking someone who wants to merge into my lane
                    if (isBlockingMergeing(veh_id)[0]):
                        traci.vehicle.setSpeed(veh_id, -1);
                        return;
                    # Go as fast as needed to only make it to the next green
                    time_until_green = timeUntilNextGreen(tls_id, tls_laneIndex) + 1
                    speed = tls_distance / time_until_green
                    traci.vehicle.setSpeed(veh_id, speed)
                speedWave = True;
    if (not speedWave):
        traci.vehicle.setSpeed(veh_id, -1)

def fetchOptionalParameters():
    visualize = False
    for i in range(3, len(sys.argv)):
        if sys.argv[i] == "--visualize" or sys.argv[i] == "-v":
            visualize = True
    return (visualize)

def printUsage():
    print("USAGE: av_comparison.py <algorithm> <situation> [--visualize, -v]");
    print("    algorithm: 0 - none")
    print("               1 - platooning")
    print("               2 - speed waves (with shorter TLS phases)")
    print("               c - compare all")
    print("    situation: 01 - crossing, 1 lane");
    print("               02 - crossing, 2 lanes");
    print("               4 - 4 intersections");

#### MAIN
# Square/square
# Cross1/cross1
if __name__ == "__main__":
    prod = True

    if (prod):
        # Command line interface
        if (len(sys.argv) < 4):
            printUsage(); exit(0);
        filepath = ""
        algorithm = sys.argv[1]
        if (algorithm != "0" and algorithm != "1" and algorithm != "2"):
            print("ERROR: Invalid <algorithm> value."); printUsage(); exit(0);
        algorithm = int(algorithm)
        situation = sys.argv[2]
        match (situation):
            case "01":
                filepath = "Cross1/cross1";
            case "02":
                filepath = "Cross2/cross2";
            case "4":
                filepath = "Square/square";
            case _:
                print("ERROR: Invalid <situation> value."); printUsage(); exit(0);
        visualize = fetchOptionalParameters()
    else:
        filepath = "Square/square"
        algorithm = 2
        visualize = False
        
    
    if (algorithm == 2):
        if (os.path.isfile(filepath + "_shortTLS.sumocfg")):
            filepath += "_shortTLS";

    if (visualize):
        traci.start(["sumo-gui", "-c", filepath + ".sumocfg", "--step-length", str(STEP_LENGTH), "--start"])
    else:
        traci.start(["sumo", "-c", filepath + ".sumocfg", "--step-length", str(STEP_LENGTH), "--start"])

    step = 0
    while traci.simulation.getMinExpectedNumber() > 0 and traci.simulation.getTime() < 10:
        traci.simulationStep()
        vehicles = traci.vehicle.getIDList()
        arrived = traci.simulation.getArrivedIDList()

        if algorithm == 1: # PLATOONING
            # Go through despawned
            for veh_id in arrived:
                if veh_id in platoon: del platoon[veh_id];
            # Go through existing vehicles
            for veh_id in vehicles:
                speed = traci.vehicle.getSpeed(veh_id)
                typeId = traci.vehicle.getTypeID(veh_id)
                if typeId == "AV":
                    av_speeds.setdefault(veh_id, []).append(speed)
                    # Apply platooning
                    platooning(veh_id, arrived)
                else:
                    hdv_speeds.setdefault(veh_id, []).append(speed)
            # Colour leaders
            colorPlatoonMembers(vehicles)
            # Draw platoon connections
            drawPlatoonConnections(vehicles, arrived)

        if algorithm == 2: # SPEED WAVES
             for veh_id in vehicles:
                speed = traci.vehicle.getSpeed(veh_id)
                typeId = traci.vehicle.getTypeID(veh_id)
                if typeId == "AV":
                    speedWaves(veh_id)
        step += 1
        
    # --- Collect travel times ---
    for veh_id in traci.vehicle.getIDList():
        travel_times[veh_id] = traci.vehicle.getAccumulatedWaitingTime(veh_id)

    traci.close()

    # --- Simple analysis ---
    avg_av_speed = sum([sum(s)/len(s) for s in av_speeds.values()])/len(av_speeds)
    avg_hdv_speed = sum([sum(s)/len(s) for s in hdv_speeds.values()])/len(hdv_speeds)

    print(f"Average AV speed: {avg_av_speed:.2f} m/s")
    print(f"Average HDV speed: {avg_hdv_speed:.2f} m/s")

    # Plot speed profiles
    plt.figure(figsize=(10,5))
    for veh_id, speeds in av_speeds.items():
        plt.plot(speeds, label=f"{veh_id} (AV)")
    for veh_id, speeds in hdv_speeds.items():
        plt.plot(speeds, label=f"{veh_id} (HDV)", linestyle='--')
    plt.xlabel("Simulation step")
    plt.ylabel("Speed (m/s)")
    plt.title("Vehicle speed profiles")
    plt.legend()
    plt.show()
