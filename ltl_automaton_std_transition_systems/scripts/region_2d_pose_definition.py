#!/usr/bin/env python
import yaml
import math
import region_2d_pose_generator

def extract_point_coord_from_input(start_point_str):
    point_coord_txt = start_point_str.split(',')
    if len(point_coord_txt) == 2:
        point_coord = []
        # x coordinate
        point_coord.append(float(point_coord_txt[0]))
        # y coordinate
        point_coord.append(float(point_coord_txt[1]))
        return point_coord
    else:
        return None

def extract_station_from_input(station_str):
    station_txt = station_str.split(',')
    if len(station_txt) == 7:
        station_data = []
        # station x coordinate
        station_data.append(float(station_txt[0]))
        # station y coordinate
        station_data.append(float(station_txt[1]))
        # station yaw
        station_data.append(float(station_txt[2]))
        # station radius
        station_data.append(float(station_txt[3]))
        # station angle tolerance
        station_data.append(float(station_txt[4]))
        # station distance hysteresis
        station_data.append(float(station_txt[5]))
        # station angle hysteresis
        station_data.append(float(station_txt[6]))
        return station_data
    else:
        return None

# Check if station is inside grid dimension and if radius is not bigger than square
def check_station(grid_data, station_data):
    # Check if station radius is smaller than square side lenght:
    if (station_data[3] > grid_data[1]) or (station_data[3] <=0):
        print "Station radius must be a positive value and inferior than cell side lenght (%f meters)" % grid_data[1]
        return False
    # Check if angle tolerance is inferior or equal to pi radians
    if (station_data[4] > math.pi):
        print "Station angle tolerance must be inferior or equal to pi radians"
        return False
    # Check if distance hysteresis is smaller than square cell side length
    if (station_data[5] > grid_data[1]):
        print "Station distance hysteresis must be inferior than half of the cell side lenght (%f meters)" % (grid_data[1]/2)
        return False
    # Check if angle hysteresis is inferior or equal to pi radians
    if (station_data[6] > math.pi/2):
        print "Station angle hysteresis must be inferior or equal to half-pi radians"
        return False

    # Check if station is in grid dimension
    # Check if x is in grid
    if (station_data[0] > (grid_data[0][0] + grid_data[1]*grid_data[2])) or (station_data[0] < grid_data[0][0]):
        print("Station center point is not in grid, please enter a point inside the grid limits (from (%f, %f) to (%f, %f))"
           % (grid_data[0][0], grid_data[0][1], (grid_data[0][0] + grid_data[1]*grid_data[2]), (grid_data[0][1] + grid_data[1]*grid_data[3])))
        return False
    # Check if y is in grid
    if (station_data[1] > (grid_data[0][1] + grid_data[1]*grid_data[3])) or (station_data[1] < grid_data[0][1]):
        print("Station center point is not in grid, please enter a point inside the grid limits (from (%f, %f) to (%f, %f))"
           % (grid_data[0][0], grid_data[0][1], (grid_data[0][0] + grid_data[1]*grid_data[2]), (grid_data[0][1] + grid_data[1]*grid_data[3])))
        return False

    # Else, station is okay
    return True



#=============================
#         Main Script
#=============================
print "-------------------------------"
print " 2D pose region grid generator"
print "-------------------------------"
print "Please enter grid file name"
prompt = '> '
file_name = raw_input(prompt)

#test_dict = {'state_dim': ['2d_pose_region'], 'state_models': {'2d_pose_region': {'nodes': {'r4': {'attr': {'length': 1.0, 'pose': [[2.5, 2.5], [0]], 'type': 'square', 'hysteresis': 0.05}, 'connected_to': {'r5': 'goto_r5', 's0': 'goto_s0', 'r7': 'goto_r7', 'r1': 'goto_r1', 'r3': 'goto_r3'}}, 'r5': {'attr': {'length': 1.0, 'pose': [[3.5, 2.5], [0]], 'type': 'square', 'hysteresis': 0.05}, 'connected_to': {'r4': 'goto_r4', 'r6': 'goto_r6', 'r8': 'goto_r8', 'r2': 'goto_r2'}}, 'r6': {'attr': {'length': 1.0, 'pose': [[4.5, 2.5], [0]], 'type': 'square', 'hysteresis': 0.05}, 'connected_to': {'r5': 'goto_r5', 'r7': 'goto_r7', 'r9': 'goto_r9', 'r3': 'goto_r3'}}, 'r7': {'attr': {'length': 1.0, 'pose': [[2.5, 3.5], [0]], 'type': 'square', 'hysteresis': 0.05}, 'connected_to': {'r4': 'goto_r4', 'r6': 'goto_r6', 'r8': 'goto_r8', 'r10': 'goto_r10'}}, 'r12': {'attr': {'length': 1.0, 'pose': [[4.5, 4.5], [0]], 'type': 'square', 'hysteresis': 0.05}, 'connected_to': {'r9': 'goto_r9', 'r11': 'goto_r11'}}, 'r1': {'attr': {'length': 1.0, 'pose': [[2.5, 1.5], [0]], 'type': 'square', 'hysteresis': 0.05}, 'connected_to': {'r4': 'goto_r4', 'r2': 'goto_r2'}}, 'r2': {'attr': {'length': 1.0, 'pose': [[3.5, 1.5], [0]], 'type': 'square', 'hysteresis': 0.05}, 'connected_to': {'r5': 'goto_r5', 'r1': 'goto_r1', 'r3': 'goto_r3'}}, 'r3': {'attr': {'length': 1.0, 'pose': [[4.5, 1.5], [0]], 'type': 'square', 'hysteresis': 0.05}, 'connected_to': {'r4': 'goto_r4', 'r6': 'goto_r6', 'r2': 'goto_r2'}}, 's0': {'attr': {'angle_tolerance': 0.2, 'angle_hysteresis': 0.1, 'pose': [[2.0, 2.2], [0.0]], 'radius': 0.2, 'dist_hysteresis': 0.2, 'type': 'station'}, 'connected_to': {'r4': 'goto_r4'}}, 'r8': {'attr': {'length': 1.0, 'pose': [[3.5, 3.5], [0]], 'type': 'square', 'hysteresis': 0.05}, 'connected_to': {'r5': 'goto_r5', 'r7': 'goto_r7', 'r9': 'goto_r9', 'r11': 'goto_r11'}}, 'r9': {'attr': {'length': 1.0, 'pose': [[4.5, 3.5], [0]], 'type': 'square', 'hysteresis': 0.05}, 'connected_to': {'r12': 'goto_r12', 'r6': 'goto_r6', 'r8': 'goto_r8', 'r10': 'goto_r10'}}, 'r11': {'attr': {'length': 1.0, 'pose': [[3.5, 4.5], [0]], 'type': 'square', 'hysteresis': 0.05}, 'connected_to': {'r8': 'goto_r8', 'r12': 'goto_r12', 'r10': 'goto_r10'}}, 'r10': {'attr': {'length': 1.0, 'pose': [[2.5, 4.5], [0]], 'type': 'square', 'hysteresis': 0.05}, 'connected_to': {'r7': 'goto_r7', 'r9': 'goto_r9', 'r11': 'goto_r11'}}}, 'initial': '', 'ts_type': '2d_pose_region'}}, 'actions': {'goto_r5': {'type': 'move', 'attr': {'pose': [[3.5, 2.5, 0], [0, 0, 0, 1]]}, 'weight': 10}, 'goto_r4': {'type': 'move', 'attr': {'pose': [[2.5, 2.5, 0], [0, 0, 0, 1]]}, 'weight': 10}, 'goto_s0': {'type': 'move', 'attr': {'pose': [[2.0, 2.2, 0], [0., 0., 0., 1]]}, 'weight': 10}, 'goto_r6': {'type': 'move', 'attr': {'pose': [[4.5, 2.5, 0], [0, 0, 0, 1]]}, 'weight': 10}, 'goto_r1': {'type': 'move', 'attr': {'pose': [[2.5, 1.5, 0], [0, 0, 0, 1]]}, 'weight': 10}, 'goto_r3': {'type': 'move', 'attr': {'pose': [[4.5, 1.5, 0], [0, 0, 0, 1]]}, 'weight': 10}, 'goto_r2': {'type': 'move', 'attr': {'pose': [[3.5, 1.5, 0], [0, 0, 0, 1]]}, 'weight': 10}, 'goto_r9': {'type': 'move', 'attr': {'pose': [[4.5, 3.5, 0], [0, 0, 0, 1]]}, 'weight': 10}, 'goto_r7': {'type': 'move', 'attr': {'pose': [[2.5, 3.5, 0], [0, 0, 0, 1]]}, 'weight': 10}, 'goto_r8': {'type': 'move', 'attr': {'pose': [[3.5, 3.5, 0], [0, 0, 0, 1]]}, 'weight': 10}, 'goto_r11': {'type': 'move', 'attr': {'pose': [[3.5, 4.5, 0], [0, 0, 0, 1]]}, 'weight': 10}, 'goto_r10': {'type': 'move', 'attr': {'pose': [[2.5, 4.5, 0], [0, 0, 0, 1]]}, 'weight': 10}, 'goto_r12': {'type': 'move', 'attr': {'pose': [[4.5, 4.5, 0], [0, 0, 0, 1]]}, 'weight': 10}}}
#region_2d_pose_generator.write_to_file(file_name, test_dict)


# Base grid definition
print "Enter origin of grid on the format \"x,y\""
prompt = '>'
start_point = raw_input(prompt)
point_coord = extract_point_coord_from_input(start_point)
while not (point_coord):
    print "Entered value is not on the format \"x,y\", please enter origin of grid again"
    prompt = '>'
    start_point = raw_input(prompt)
    point_coord = extract_point_coord_from_input(start_point)

print "Enter cell side lenght (in meters)"
cell_side_length = None
while not cell_side_length:
    prompt = '>'
    cell_side_length = float(raw_input(prompt))
    if cell_side_length <= 0:
        print "Please enter a non-zero positive value"
        cell_side_length = None

print "Enter cell hysteresis (added distance for leaving cell, in meters and needs to be smaller than half cell side length)"
cell_hysteresis = None
while not cell_hysteresis:
    prompt = '>'
    cell_hysteresis = float(raw_input(prompt))
    if (cell_hysteresis <= 0) or (cell_hysteresis >= cell_side_length/2):
        print "Please enter a non-zero positive value"
        cell_hysteresis = None

print "Enter how many cells on x-axis"
number_of_cells_x = None
while not number_of_cells_x:
    prompt = '>'
    number_of_cells_x = int(raw_input(prompt))
    if number_of_cells_x < 1:
        print "Please enter a minimum value of 1 square on x-axis"
        number_of_cells_x = None

print "Enter how many cells on y-axis"
number_of_cells_y = None
while not number_of_cells_y:
    prompt = '>'
    number_of_cells_y = int(raw_input(prompt))
    if number_of_cells_y < 1:
        print "Please enter a minimum value of 1 square on x-axis"
        number_of_cells_y = None

grid_data = [point_coord, cell_side_length, number_of_cells_x, number_of_cells_y, cell_hysteresis]

# Grid recap
print "----------------------------------------------------------"
print("Creating grid of %f meters (x-axis) by %f meters (y-axis) with origin (%f, %f), and %i square cells of side length of %f meters. Hysteresis for leaving cell of %f meters." 
      % ((number_of_cells_x * cell_side_length),
         (number_of_cells_y * cell_side_length),
          point_coord[0], point_coord[1],
          number_of_cells_x * number_of_cells_y,
          cell_side_length,
          cell_hysteresis))
print "----------------------------------------------------------"
# Create grid dictionary
grid_dict = {}
grid_dict.update({'origin': {'x': point_coord[0],
                             'y': point_coord[1]},
                  'cell_side_length': cell_side_length,
                  'cell_hysteresis': cell_hysteresis,
                  'number_of_cells_x': number_of_cells_x,
                  'number_of_cells_y': number_of_cells_y})


# Add stations
print "Enter stations on the format \"x, y, yaw, radius, angle tolerance, distance hysteresis, angle hysteresis\" (in meters and angle in rad), type \"end\" to stop entering stations"
inputing_stations = True
station_list = []
while inputing_stations:
    prompt = '>'
    keyboard_input = raw_input(prompt)
    if not keyboard_input == "end":
        station = extract_station_from_input(keyboard_input)
        if station:
            if check_station(grid_data, station):
                station_list.append(station)
        else:
            print "Entered value is not on the format \"x, y, yaw, radius, angle tolerance, distance hysteresis, angle hysteresis\" (in meters and angle in rad), please enter station again"
    else:
        inputing_stations = False

print "----------------------------------------------------------"
if len(station_list) == 0:
    print " No station added"
else:
    print " Adding the following stations"
    station_dicts = []
    for i in range(len(station_list)):
        print "  - center: (%f, %f), yaw: %f rad, radius: %f meters, angle tolerance: %f, distance hysteresis: %f, angle hysteresis: %f" % (station_list[i][0],
                                                                                                                                            station_list[i][1],
                                                                                                                                            station_list[i][2],
                                                                                                                                            station_list[i][3],
                                                                                                                                            station_list[i][4],
                                                                                                                                            station_list[i][5],
                                                                                                                                            station_list[i][6])
        station_dicts.append({'origin': {'x': station_list[i][0],
                                        'y': station_list[i][1],
                                        'yaw': station_list[i][2]},
                              'radius': station_list[i][3],
                              'angle_tolerance': station_list[i][4],
                              'dist_hysteresis': station_list[i][5],
                              'angle_hysteresis': station_list[i][6],
                              })

print "----------------------------------------------------------"
print " Enter agent initial position on the on the format \"x,y\""
prompt = '>'
initial_position_str = raw_input(prompt)
initial_position = extract_point_coord_from_input(initial_position_str)
while not (initial_position):
    print "Entered value is not on the format \"x,y\", please enter agent initial position again"
    prompt = '>'
    initial_position_str = raw_input(prompt)
    initial_position = extract_point_coord_from_input(initial_position_str)


region_2d_dict = {}
region_2d_dict.update({'grid': grid_dict, 'stations': station_dicts, 'initial_position': initial_position})


# Generate and write ts to file
region_2d_pose_generator.write_to_file(file_name, region_2d_pose_generator.generate_regions_and_actions(region_2d_dict))