from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import time
from ortools.linear_solver import pywraplp


# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


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
        <vType id="carA" accel="100.0" decel="100.0" sigma="0.0" length="5" minGap="0.0" maxSpeed="30"/>
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
                    print('    <vehicle id="WS_%i" type="carA" route="route01" depart="%i" />' % (vehNr, i), file=routes)
                if r > 2./3:
                    print('    <vehicle id="WE_%i" type="carA" route="route02" depart="%i" />' % (vehNr, i), file=routes)
                else:
                    print('    <vehicle id="WN_%i" type="carA" route="route03" depart="%i" />' % (vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pS:
                r = random.uniform(0, 1)
                if r < 1./3:
                    print('    <vehicle id="SE_%i" type="carA" route="route04" depart="%i" />' % (vehNr, i), file=routes)
                if r > 2./3:
                    print('    <vehicle id="SN_%i" type="carA" route="route05" depart="%i" />' % (vehNr, i), file=routes)
                else:
                    print('    <vehicle id="SW_%i" type="carA" route="route06" depart="%i" />' % (vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pE:
                r = random.uniform(0, 1)
                if r < 1./3:
                    print('    <vehicle id="EN_%i" type="carA" route="route07" depart="%i" />' % (vehNr, i), file=routes)
                if r > 2./3:
                    print('    <vehicle id="EW_%i" type="carA" route="route08" depart="%i" />' % (vehNr, i), file=routes)
                else:
                    print('    <vehicle id="ES_%i" type="carA" route="route09" depart="%i" />' % (vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pN:
                r = random.uniform(0, 1)
                if r < 1./3:
                    print('    <vehicle id="NW_%i" type="carA" route="route10" depart="%i" />' % (vehNr, i), file=routes)
                if r > 2./3:
                    print('    <vehicle id="NS_%i" type="carA" route="route11" depart="%i" />' % (vehNr, i), file=routes)
                else:
                    print('    <vehicle id="NE_%i" type="carA" route="route12" depart="%i" />' % (vehNr, i), file=routes)
                vehNr += 1
        print("</routes>", file=routes)

###################


MAX_SPEED = 28.0
Z1_LEN = 100.0
Z2_LEN = 300.0
H_MIN = 10.0
DELTA_TAU = 5.0
duration = 5.0
Z2_lanes = ['d1_0','d1_1','d2_0','d2_1','d3_0','d3_1','d4_0','d4_1']
cruise = []


def run():
    """execute the TraCI control loop"""
    step = 0
    old_c = []
    
    
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
		
		# #########Cause Collision Deliberately ########
        allCar = traci.vehicle.getIDList()
        for id in allCar:
            traci.vehicle.setSpeedMode(id, 0)
		# ##############################################
		
        new_c = []
        
        #all_c = traci.vehicle.getIDList()
        '''
        

        for i in range(len(all_c)):
            #print(all_c[i])
            L = traci.vehicle.getLaneID(all_c[i])
            #print(L)

            if L in ('1_0','2_0','3_0','4_0','1_1','2_1','3_1','4_1','1_2','2_2','3_2','4_2'):
                print('old car')
                pos = Z2_LEN - traci.vehicle.getLanePosition(all_c[i])
                OT = pos / MAX_SPEED
                old_c.append({'ID': all_c[i], 'pos': pos, 'lane': L, 'OT': OT })
            
            if L in ('o1_0','o2_0','o3_0','o4_0','o1_1','o2_1','o3_1','o4_1','o1_2','o2_2','o3_2','o4_2'):
                #print('new car')
                pos = Z1_LEN + Z2_LEN - traci.vehicle.getLanePosition(all_c[i])
                OT = pos / MAX_SPEED
                new_c.append({'ID': all_c[i], 'pos': pos, 'lane': L, 'OT': OT, 'D': 0 })
        '''

                

        # new car
        
        for i,lane in enumerate(Z2_lanes,1):
            if traci.inductionloop.getLastStepVehicleNumber(lane) > 0:
                print(i)
                print(lane)
                Car = traci.inductionloop.getLastStepVehicleIDs(lane)
                CarID = Car[0]
                pos = Z2_LEN
                new_c.append({'ID': CarID, 'lane': lane, 'pos':pos })
                # print(new_c)
        

                
        start = time.time()
        (delay_list, old_c) = Calculate(new_c, old_c)
		
        for i,lane in enumerate(Z2_lanes, 1):
            if traci.inductionloop.getLastStepVehicleNumber(lane) > 0:
                Veh = traci.inductionloop.getLastStepVehicleIDs(lane)
                VehID = Veh[0]
                # print(delay_list[VehID])
                D = float(delay_list[VehID])
                speed = (Z2_LEN - MAX_SPEED*duration) / (Z2_LEN / MAX_SPEED + D - 2*duration)
                traci.vehicle.slowDown(VehID, speed, duration)
                # how to accelertate again? append the time when the car will end it's cruise with VehID
                const_t = (MAX_SPEED + speed) * duration / 2 / speed
                tt = time.time() + const_t
                cruise.append({'VehID': VehID, 'cruise end at': tt})
        i=0
        for j in range(len(cruise)):
            if int(cruise[i]['cruise end at']) == step:
                traci.vehicle.slowDown(cruise[i]['VehID'], MAX_SPEED, duration)
                cruise.pop(i)
                i = i - 1
            i = i + 1
        step += 1
    traci.close()
    sys.stdout.flush()

    
#######################
'''

def main():    

    if (len(sys.argv) < 2):
        print(sys.argv[0], ' car_num')
        exit()
    
    n_car_num = int(sys.argv[1])
    o_car_num = int(n_car_num*(Z2_LEN/Z1_LEN))
    new_cars = []
    old_cars = []
    
    for c_idx in range(n_car_num):
        pos = randrange(Z1_LEN) + Z2_LEN
        lane = randrange(12)
        new_cars.append({'pos': pos, 'lane': lane})
    for c_idx in range(o_car_num):
        pos = randrange(Z2_LEN)
        OT = pos/MAX_SPEED
        lane = randrange(12)
        old_cars.append({'pos': pos, 'lane': lane, 'OT': OT, 'D': 0})
    
    start = time.time()
    Calculate(old_cars, new_cars, o_car_num, n_car_num)
    end = time.time()
    # print("=== Result ===")
    # print("Ideal time: ", Z1_LEN/MAX_SPEED)
    # print("Computational time: ", end-start)
    print(end-start)
'''
    
def Calculate(new_cars,old_cars):
    

          
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
            if (all_cars[c_idx]['lane'] == all_cars[c_jdx]['lane']):
                if (all_cars[c_idx]['OT'] > all_cars[c_jdx]['OT']):
                    if (type(all_cars[c_idx]['D'])!=int and type(all_cars[c_jdx]['D'])==int):
                        bound = H_MIN + (all_cars[c_jdx]['OT']+all_cars[c_jdx]['D'])
                        bound = bound - all_cars[c_idx]['OT']
                        tmp_conts = solver.Constraint(bound,solver.infinity())
                        tmp_conts.SetCoefficient(all_cars[c_idx]['D'], 1)
                    elif (type(all_cars[c_idx]['D'])==int and type(all_cars[c_jdx]['D'])!=int):
                        bound = H_MIN - (all_cars[c_idx]['OT']+all_cars[c_idx]['D'])
                        bound = -bound - all_cars[c_jdx]['OT']
                        tmp_conts = solver.Constraint(-solver.infinity(), bound)
                        tmp_conts.SetCoefficient(all_cars[c_jdx]['D'], 1)
                    elif (type(all_cars[c_idx]['D'])!=int and type(all_cars[c_jdx]['D'])!=int):
                        bound = H_MIN - all_cars[c_idx]['OT']+all_cars[c_jdx]['OT']
                        tmp_conts = solver.Constraint(bound, solver.infinity())
                        tmp_conts.SetCoefficient(all_cars[c_idx]['D'], 1)
                        tmp_conts.SetCoefficient(all_cars[c_jdx]['D'], 1)
        
    # part 6: set constrain (11)
    for c_idx in range(len(new_cars)):
        for c_jdx in range(c_idx+1, len(new_cars)):
            if (IsConflict(new_cars[c_idx], new_cars[c_jdx])):
                tmp_A = solver.NumVar(0, solver.infinity(), 'A'+str(c_idx)+"_"+str(c_jdx))
                tmp_B = solver.NumVar(0, solver.infinity(), 'B'+str(c_idx)+"_"+str(c_jdx))
                tmp_conts1 = solver.Constraint(0, solver.infinity())
                tmp_conts1.SetCoefficient(tmp_A, 1)
                
                tmp_conts2 = solver.Constraint(0, solver.infinity())
                tmp_conts2.SetCoefficient(tmp_B, 1)
                
                bound = new_cars[c_idx]['OT'] + new_cars[c_jdx]['OT']
                tmp_conts3 = solver.Constraint(bound, bound)
                tmp_conts3.SetCoefficient(tmp_A, 1)
                tmp_conts3.SetCoefficient(tmp_B, -1)
                tmp_conts3.SetCoefficient(new_cars[c_idx]['D'], -1)
                tmp_conts3.SetCoefficient(new_cars[c_jdx]['D'], 1)
                
                tmp_conts4 = solver.Constraint(DELTA_TAU, solver.infinity())
                tmp_conts4.SetCoefficient(tmp_A, 1)
                tmp_conts4.SetCoefficient(tmp_B, 1)
                
                
    # part 7: set constrain (12)
    for nc_idx in range(len(new_cars)):
        for oc_idx in range(len(old_cars)):
            bound = old_cars[oc_idx]['OT']+old_cars[oc_idx]['D']-new_cars[nc_idx]['OT']
            print(bound)
            print(old_cars[oc_idx]['OT'],old_cars[oc_idx]['D'],new_cars[nc_idx]['OT'])
            tmp_conts = solver.Constraint(bound, solver.infinity())
            tmp_conts.SetCoefficient(new_cars[nc_idx]['D'], 1)
            
          
    # part 8: set constrain (13)
    for nc_idx in range(len(new_cars)):
        tmp_conts = solver.Constraint(0, solver.infinity())
        tmp_conts.SetCoefficient(new_cars[nc_idx]['D'], 1)
    
    
    # part 9: Solve the problem
    solver.Solve()
    # print('Solution:')
    # for nc_idx in range(n_car_num):
    # print('D', nc_idx, " = ", new_cars[nc_idx]['D'].solution_value())

        
    # this is the output information
    d_list=dict()
    for c_idx in range(len(new_cars)):
        print(c_idx)
        print(new_cars[c_idx])
        d_list['{}'.format(new_cars[c_idx]['ID'])]='{}'.format(new_cars[c_idx]['D'].solution_value())
        new_cars[c_idx]['D'] = new_cars[c_idx]['D'].solution_value()
    print(d_list)
    

    # refresh OT and position of old cars
    old_cars = old_cars + new_cars
    ii = 0
    for c_idx in range(len(old_cars)):
        R = traci.vehicle.getRoadID(old_cars[ii]['ID'])
        print(R)
        if R in ('1','2','3','4','o1','o2','o3','o4'):
            print('###############')
            pos = Z2_LEN - traci.vehicle.getLanePosition(old_cars[ii]['ID'])
            OT = pos / MAX_SPEED
            old_cars[ii]['pos'] = pos
            old_cars[ii]['OT'] = OT
        else:
            print('$$$$$$$$$$$$$$$$$$$$$$')
            old_cars.pop(ii)
            ii = ii - 1
        ii = ii + 1
    print(old_cars)
        

    return(d_list, old_cars)




def IsConflict(car1, car2):
    is_conflict = False
    if (car1['lane']==1_1):
        if (car2['lane']==4_1):
            is_conflict = True
        elif (car2['lane']==2_2):
            is_conflict = True
        elif (car2['lane']==3_2):
            is_conflict = True
        elif (car2['lane']==2_1):
            is_conflict = True
    elif (car1['lane']==1_2):
        if (car2['lane']==2_2):
            is_conflict = True
        elif (car2['lane']==4_1):
            is_conflict = True
        elif (car2['lane']==3_1):
            is_conflict = True
        elif (car2['lane']==4_2):
            is_conflict = True
    elif (car1['lane']==2_1):
        if (car2['lane']==1_1):
            is_conflict = True
        elif (car2['lane']==3_2):
            is_conflict = True
        elif (car2['lane']==4_2):
            is_conflict = True
        elif (car2['lane']==3_1):
            is_conflict = True
    elif (car1['lane']==2_2):
        if (car2['lane']==3_2):
            is_conflict = True
        elif (car2['lane']==1_1):
            is_conflict = True
        elif (car2['lane']==4_1):
            is_conflict = True
        elif (car2['lane']==1_2):
            is_conflict = True
    elif (car1['lane']==3_1):
        if (car2['lane']==2_1):
            is_conflict = True
        elif (car2['lane']==4_2):
            is_conflict = True
        elif (car2['lane']==1_2):
            is_conflict = True
        elif (car2['lane']==4_1):
            is_conflict = True
    elif (car1['lane']==3_2):
        if (car2['lane']==4_2):
            is_conflict = True
        elif (car2['lane']==2_1):
            is_conflict = True
        elif (car2['lane']==1_1):
            is_conflict = True
        elif (car2['lane']==2_2):
            is_conflict = True
    elif (car1['lane']==4_1):
        if (car2['lane']==3_1):
            is_conflict = True
        elif (car2['lane']==1_2):
            is_conflict = True
        elif (car2['lane']==2_2):
            is_conflict = True
        elif (car2['lane']==1_1):
            is_conflict = True
    elif (car1['lane']==4_2):
        if (car2['lane']==1_2):
            is_conflict = True
        elif (car2['lane']==3_1):
            is_conflict = True
        elif (car2['lane']==2_1):
            is_conflict = True
        elif (car2['lane']==3_2):
            is_conflict = True
    return is_conflict
    
   
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

