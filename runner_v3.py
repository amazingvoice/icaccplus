from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import time
from ortools.linear_solver import pywraplp
from sumolib import checkBinary  # noqa
import traci  # noqa

from get_inter_info import Data


# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


MAX_SPEED = 28.0
TURN_SPEED = 25
Z1_LEN = 100.0
Z2_LEN = 300.0
H_MIN = 10.0
DELTA_TAU = 5.0
duration = 5.0
LARGE_NUM = 99999999
Z2_lanes = ['d1_0','d1_1','d2_0','d2_1','d3_0','d3_1','d4_0','d4_1']
cruise = []
data = Data()


def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
    # demand per second from different directions
    pW = 1. / 3
    pE = 1. / 3
    pN = 1. / 3
    pS = 1. / 3
    with open("data/icacc+.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="carA" accel="100.0" decel="100.0" sigma="0.0" length="{}" minGap="0.0" maxSpeed="30"/>""".format(random.randrange(5,15)), file=routes)
        print("""
        <route id="route01" edges="o1 1 -2 o-2"/>
        <route id="route02" edges="o1 1 -3 o-3"/>
        <route id="route03" edges="o1 1 -4 o-4"/>
        <route id="route04" edges="o2 2 -3 o-3"/>
        <route id="route05" edges="o2 2 -4 o-4"/>
        <route id="route06" edges="o2 2 -1 o-1"/>
        <route id="route07" edges="o3 3 -4 o-4"/>
        <route id="route08" edges="o3 3 -1 o-1"/>
        <route id="route09" edges="o3 3 -2 o-2"/>
        <route id="route10" edges="o4 4 -1 o-1"/>
        <route id="route11" edges="o4 4 -2 o-2"/>
        <route id="route12" edges="o4 4 -3 o-3"/>""", file=routes)
        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < pW:
                r = random.uniform(0, 1)
                if r < 1./3:
                    print('    <vehicle id="R_%i" type="carA" route="route01" depart="%i" />' % (vehNr, i), file=routes)
                elif r > 2./3:
                    print('    <vehicle id="S_%i" type="carA" route="route02" depart="%i" />' % (vehNr, i), file=routes)
                else:
                    print('    <vehicle id="L_%i" type="carA" route="route03" depart="%i" />' % (vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pS:
                r = random.uniform(0, 1)
                if r < 1./3:
                    print('    <vehicle id="R_%i" type="carA" route="route04" depart="%i" />' % (vehNr, i), file=routes)
                elif r > 2./3:
                    print('    <vehicle id="S_%i" type="carA" route="route05" depart="%i" />' % (vehNr, i), file=routes)
                else:
                    print('    <vehicle id="L_%i" type="carA" route="route06" depart="%i" />' % (vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pE:
                r = random.uniform(0, 1)
                if r < 1./3:
                    print('    <vehicle id="R_%i" type="carA" route="route07" depart="%i" />' % (vehNr, i), file=routes)
                elif r > 2./3:
                    print('    <vehicle id="S_%i" type="carA" route="route08" depart="%i" />' % (vehNr, i), file=routes)
                else:
                    print('    <vehicle id="L_%i" type="carA" route="route09" depart="%i" />' % (vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pN:
                r = random.uniform(0, 1)
                if r < 1./3:
                    print('    <vehicle id="R_%i" type="carA" route="route10" depart="%i" />' % (vehNr, i), file=routes)
                elif r > 2./3:
                    print('    <vehicle id="S_%i" type="carA" route="route11" depart="%i" />' % (vehNr, i), file=routes)
                else:
                    print('    <vehicle id="L_%i" type="carA" route="route12" depart="%i" />' % (vehNr, i), file=routes)
                vehNr += 1
        print("</routes>", file=routes)

###################


def run():
    """execute the TraCI control loop"""
    step = 0
    old_c = []

    delay_list = dict()
    
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        new_c = []

    # ######### Cause Collision Deliberately ########
        allCar = traci.vehicle.getIDList()
        for id in allCar:
            traci.vehicle.setSpeedMode(id, 0)
    # ###############################################
        
        all_c = traci.vehicle.getIDList()

        for i in range(len(all_c)):
            # print(all_c[i])
            L = traci.vehicle.getLaneID(all_c[i])

            if L in ('1_0', '2_0', '3_0', '4_0', '1_1', '2_1', '3_1', '4_1'):
                pos = Z2_LEN - traci.vehicle.getLanePosition(all_c[i])
                OT = pos / MAX_SPEED
                lane = int(L[0])*2+int(L[2])+1
                length = traci.vehicle.getLength(all_c[i])
                d = delay_list[all_c[i]]
                direction = all_c[i][0]
                inter_speed = TURN_SPEED
                if direction == "S":
                    inter_speed = MAX_SPEED
                else:
                    inter_speed = TURN_SPEED

                old_c.append({'ID': all_c[i],
                              'pos': pos,
                              'lane': lane,
                              'OT': OT,
                              'D': d,
                              'direction': direction,
                              'inter_speed': inter_speed,
                              'length': length
                              })

            if L in ('o1_0', 'o2_0', 'o3_0', 'o4_0', 'o1_1', 'o2_1', 'o3_1', 'o4_1'):
                pos = Z1_LEN + Z2_LEN - traci.vehicle.getLanePosition(all_c[i])
                OT = pos / MAX_SPEED
                lane = int(L[1])*2+int(L[3])+1
                length = traci.vehicle.getLength(all_c[i])
                print(length)
                direction = all_c[i][0]
                inter_speed = TURN_SPEED
                if direction == "S":
                    inter_speed = MAX_SPEED
                else:
                    inter_speed = TURN_SPEED

                new_c.append({'ID': all_c[i],
                              'pos': pos,
                              'lane': lane,
                              'OT': OT,
                              'direction': direction,
                              'inter_speed': inter_speed,
                              'length': length
                              })

        start = time.time()
        delay_list = Calculate(old_c, new_c, delay_list)

        for i, lane in enumerate(Z2_lanes, 1):
            if traci.inductionloop.getLastStepVehicleNumber(lane) > 0:
                Veh = traci.inductionloop.getLastStepVehicleIDs(lane)
                VehID = Veh[0]
                # print(delay_list[VehID])
                D = float(delay_list[VehID])
                speed = (Z2_LEN - MAX_SPEED*duration) / (Z2_LEN / MAX_SPEED + D - 2*duration)
                traci.vehicle.slowDown(VehID, speed, duration)
                # how to accelertate again? append the time when the car will end it's cruise with VehID
                # const_t = (MAX_SPEED + speed) * duration / 2 / speed
                const_t = Z2_LEN / MAX_SPEED +D - 2 * duration
                tt = time.time() + const_t + duration
                cruise.append({'VehID': VehID, 'cruise end at': tt})
        i = 0
        for j in range(len(cruise)):
            if int(cruise[i]['cruise end at']) == step:
                traci.vehicle.slowDown(cruise[i]['VehID'], MAX_SPEED, duration)
                cruise.pop(i)
                i = i - 1
            i = i + 1
        step += 1
    traci.close()
    sys.stdout.flush()


def Calculate(old_cars, new_cars, d_list):
    # part 1: calculate OT
    for c_idx in range(len(new_cars)):
        OT = new_cars[c_idx]['pos']/MAX_SPEED
        new_cars[c_idx]['OT'] = OT
        
    # part 2: build the solver
    solver = pywraplp.Solver('SolveCarProblem',pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
    
    # part 3: claim parameters
    for c_idx in range(len(new_cars)):
        new_cars[c_idx]['D'] = solver.NumVar(0, solver.infinity(), 'd'+str(c_idx))
        
    # part 4: set objective
    objective = solver.Objective()
    for c_idx in range(len(new_cars)):
        objective.SetCoefficient(new_cars[c_idx]['D'], 1)
    objective.SetMinimization()
        
    # part 5: set constrain (10)
    all_cars = old_cars+new_cars
    for c_idx in range(len(all_cars)):
        for c_jdx in range(c_idx+1, len(all_cars)):
            if all_cars[c_idx]['lane'] == all_cars[c_jdx]['lane']:
                if all_cars[c_idx]['OT'] > all_cars[c_jdx]['OT']:
                    if type(all_cars[c_idx]['D']) != float and type(all_cars[c_jdx]['D']) == float:
                        bound = H_MIN + (all_cars[c_jdx]['OT']+all_cars[c_jdx]['D'])
                        bound = bound - all_cars[c_idx]['OT']
                        tmp_conts = solver.Constraint(bound,solver.infinity())
                        tmp_conts.SetCoefficient(all_cars[c_idx]['D'], 1)
                    elif type(all_cars[c_idx]['D']) == float and type(all_cars[c_jdx]['D']) != float:
                        bound = H_MIN - (all_cars[c_idx]['OT']+all_cars[c_idx]['D'])
                        bound = -bound - all_cars[c_jdx]['OT']
                        tmp_conts = solver.Constraint(-solver.infinity(), bound)
                        tmp_conts.SetCoefficient(all_cars[c_jdx]['D'], 1)
                    elif type(all_cars[c_idx]['D']) != float and type(all_cars[c_jdx]['D']) != float:
                        bound = H_MIN - all_cars[c_idx]['OT']+all_cars[c_jdx]['OT']
                        tmp_conts = solver.Constraint(bound, solver.infinity())
                        tmp_conts.SetCoefficient(all_cars[c_idx]['D'], 1)
                        tmp_conts.SetCoefficient(all_cars[c_jdx]['D'], 1)
        
    # part 6: set constrain (11)
    for c_idx in range(len(new_cars)):
        for c_jdx in range(c_idx+1, len(new_cars)):
        
            if new_cars[c_idx]['lane'] == new_cars[c_jdx]['lane']:
                continue

            ans = data.getConflictRegion(new_cars[c_idx]['lane'], new_cars[c_idx]['direction'], new_cars[c_jdx]['lane'], new_cars[c_jdx]['direction'])
    
            tau_S1_S2 = 0
            tau_S2_S1 = 0
            
            if len(ans) > 0:
                tau_S1_S2 = ans[0] + new_cars[c_idx]['length']/new_cars[c_idx]['inter_speed']
                tau_S2_S1 = ans[1] + new_cars[c_jdx]['length']/new_cars[c_jdx]['inter_speed']

                flag = solver.IntVar(0, 1, 'flag'+str(c_idx)+"_"+str(c_jdx))
                
                if ans[0] > 0:
                    bound = new_cars[c_idx]['OT'] - new_cars[c_jdx]['OT'] - tau_S1_S2 + LARGE_NUM
                    tmp_conts1 = solver.Constraint(-solver.infinity(), bound)
                    tmp_conts1.SetCoefficient(new_cars[c_idx]['D'], -1)
                    tmp_conts1.SetCoefficient(new_cars[c_jdx]['D'], 1)
                    tmp_conts1.SetCoefficient(flag, LARGE_NUM)
                
                if ans[1] > 0:
                    bound = new_cars[c_jdx]['OT'] - new_cars[c_idx]['OT'] - tau_S2_S1
                    tmp_conts2 = solver.Constraint(-solver.infinity(), bound)
                    tmp_conts2.SetCoefficient(new_cars[c_idx]['D'], 1)
                    tmp_conts2.SetCoefficient(new_cars[c_jdx]['D'], -1)
                    tmp_conts2.SetCoefficient(flag, LARGE_NUM)

    # part 7: set constrain (12)
    for nc_idx in range(len(new_cars)):
        for oc_idx in range(len(old_cars)):
            bound = old_cars[oc_idx]['OT']+old_cars[oc_idx]['D']-new_cars[nc_idx]['OT']
            tmp_conts = solver.Constraint(bound, solver.infinity())
            tmp_conts.SetCoefficient(new_cars[nc_idx]['D'], 1)

    # part 8: set constrain (13)
    for nc_idx in range(len(new_cars)):
        tmp_conts = solver.Constraint(0, solver.infinity())
        tmp_conts.SetCoefficient(new_cars[nc_idx]['D'], 1)
    
    # part 9: Solve the problem
    solver.Solve()

    # this is the output information
    for c_idx in range(len(new_cars)):
        d_list['{}'.format(new_cars[c_idx]['ID'])] = new_cars[c_idx]['D'].solution_value()

        new_cars[c_idx]['D'] = new_cars[c_idx]['D'].solution_value()

    print(d_list)
    return d_list

##########################


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/icacc+.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()

