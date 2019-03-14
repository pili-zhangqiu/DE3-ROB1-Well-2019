# tf.transformations alternative is not yet available in tf2
from tf.transformations import *
import numpy as np
import math

def calculate_brick_locations():
	overhead_orientation = np.array([-0.0249590815779,0.999649402929,0.00737916180073,0.00486450832011])
	brick_dimensions = [0.2, 0.09, 0.062]

	robot_offset = [0.6, 0.37, 0.145, 0]
	bricks_per_layer = 5
	num_layers = 3
	gap = -0.2

	adjacent_length = 0.1*((brick_dimensions[0]/2+gap)/(math.tan(math.radians(360/(2*bricks_per_layer)))))

	well_centre_to_brick_origin_lenght = adjacent_length + brick_dimensions[1]/2

	angles = np.zeros((bricks_per_layer, 1))

	for i in range(bricks_per_layer):
		theta = 360/bricks_per_layer
		angles[i] = robot_offset[3] + i*theta

	num_bricks = bricks_per_layer * num_layers
	brick_locations = np.zeros(shape=(num_bricks, 7))

	for i in range(num_layers):
		for j in range(bricks_per_layer):
				brick_number = i*bricks_per_layer+j
				brick_locations[brick_number, 0] = 0.1*(math.degrees(math.sin(math.radians(angles[j] + i*(360/(2*bricks_per_layer)) + (robot_offset[3])))) * well_centre_to_brick_origin_lenght) + robot_offset[0] # Fudge factor of 0.1, we dont know why exactly this came in (perhaps degree/radian conversion) but it is accurate.
				brick_locations[brick_number, 1] = 0.1*(math.degrees(math.cos(math.radians(angles[j] + i*(360/(2*bricks_per_layer)) + (robot_offset[3])))) * well_centre_to_brick_origin_lenght) + robot_offset[1]
				brick_locations[brick_number, 2] = (i+0.5)*brick_dimensions[1] + robot_offset[2]
				brick_locations[brick_number, 3:7]= quaternion_multiply(quaternion_from_euler(0,0,-math.radians(angles[j] + i*(360/(2*bricks_per_layer)))),overhead_orientation)
	
	brick_locations_optimised = np.zeros(shape=(num_bricks, 7))

	route = list(range(bricks_per_layer))

	optimised_route = [0]*bricks_per_layer
	flag = 0

	for i in range(bricks_per_layer):
		if flag == 0:
			optimised_route[i] = min(route)
			route.remove(min(route))
			flag = 1
		elif flag == 1:
			optimised_route[i] = max(route)
			route.remove(max(route))
			flag = 0

	for i in range(num_layers):
		for j in range(bricks_per_layer):
			old_brick_number = i*bricks_per_layer+j
			new_brick_number = i*bricks_per_layer+optimised_route[j]
			brick_locations_optimised[old_brick_number] = brick_locations[new_brick_number]
			
	return brick_locations_optimised

def calculate_brick_locations_dual():
	overhead_orientation = np.array([-0.0249590815779,0.999649402929,0.00737916180073,0.00486450832011])
	overhead_orientation_r = np.array([0.0249590815779,0.999649402929,0.00737916180073,-0.00486450832011])
	#overhead_orientation = np.array([0,0,0,0])
	brick_dimensions = [0.2, 0.062, 0.09]  #Lenght, Width, Height. Should width be 0.04 or 0.062??? It was 0.062, Hugo changed it.

	robot_offset = [0.6, 0.1, 0.13, 0] #0.15-0.24
	bricks_per_layer = 4
	num_layers = 3
	gap = -0.1 # Gap between each brick corner and corner of polygon.

	adjacent_length = 0.1*((brick_dimensions[0]/2+gap)/(math.tan(math.radians(360/(2*bricks_per_layer)))))

	well_centre_to_brick_origin_lenght = adjacent_length + brick_dimensions[1]/2

	angles = np.zeros((bricks_per_layer, 1))

	if (bricks_per_layer % 2) == 1:
		print('bricks per layer must be even for dual arms. Please change.')

	for i in range(bricks_per_layer):
		theta = 360/bricks_per_layer
		angles[i] = robot_offset[3] + i*theta-45

	num_bricks = int(bricks_per_layer/2 * num_layers)
	brick_locations_left = np.zeros(shape=(num_bricks, 7))
	brick_locations_right = np.zeros(shape=(num_bricks, 7))

	for i in range(num_layers):
		for j in range(int(bricks_per_layer/2)):
			brick_number = i*int(bricks_per_layer/2)+j
			brick_locations_right[brick_number, 0] = 0.1*(math.degrees(math.sin(math.radians(angles[j] + (i % 2)*(360/(2*bricks_per_layer)) + (robot_offset[3])))) * well_centre_to_brick_origin_lenght) + robot_offset[0]  # Fudge factor of 0.1, we dont know why exactly this came in (perhaps degree/radian conversion) but it is accurate.
			brick_locations_right[brick_number, 1] = 0.1*(math.degrees(math.cos(math.radians(angles[j] + (i % 2)*(360/(2*bricks_per_layer)) + (robot_offset[3])))) * well_centre_to_brick_origin_lenght) + robot_offset[1]
			brick_locations_right[brick_number, 2] = (i+0.5)*brick_dimensions[2] + robot_offset[2]
			brick_locations_right[brick_number, 3:7] = quaternion_multiply(quaternion_from_euler(0, 0, -math.radians(angles[j] + (i%2)*(360/(2*bricks_per_layer)))), overhead_orientation_r) #quaternion_multiply(quaternion_from_euler(0, 0, -math.radians(angles[j] + (i % 2) * (360 / (2 * bricks_per_layer)))), overhead_orientation)  # quaternion_from_euler(0,0,-math.radians(angles[j] + i*(360/(2*bricks_per_layer))))
			# quaternion_multiply(quaternion_from_euler(0,0,math.radians(angles[j] + i*(360/(2*bricks_per_layer)))),overhead_orientation)
		for j in range(int(bricks_per_layer/2)):
			brick_number = i*int(bricks_per_layer/2) + j
			brick_locations_left[brick_number, 0] = 0.1 * (math.degrees(math.sin(math.radians(angles[j + int(bricks_per_layer/2)] + (i % 2) * (360 / (2 * bricks_per_layer)) + (robot_offset[3])))) * well_centre_to_brick_origin_lenght) + robot_offset[0]  # Fudge factor of 0.1, we dont know why exactly this came in (perhaps degree/radian conversion) but it is accurate.
			brick_locations_left[brick_number, 1] = 0.1 * (math.degrees(math.cos(math.radians(angles[j + int(bricks_per_layer/2)] + (i % 2) * (360 / (2 * bricks_per_layer)) + (robot_offset[3])))) * well_centre_to_brick_origin_lenght) + robot_offset[1]
			brick_locations_left[brick_number, 2] = (i + 0.5) * brick_dimensions[2] + robot_offset[2]
			brick_locations_left[brick_number, 3:7] = quaternion_multiply(quaternion_from_euler(0, 0, -math.radians(angles[j] + (i%2)*(360/(2*bricks_per_layer)))), overhead_orientation) #quaternion_multiply(quaternion_from_euler(0, 0, -math.radians(angles[j + int(bricks_per_layer/2)] + (i % 2) * (360 / (2 * bricks_per_layer)))), overhead_orientation)  # quaternion_from_euler(0,0,-math.radians(angles[j] + i*(360/(2*bricks_per_layer))))
			# quaternion_multiply(quaternion_from_euler(0,0,math.radians(angles[j] + i*(360/(2*bricks_per_layer)))),overhead_orientation)
	brick_locations_right_optimised = np.zeros(shape=(num_bricks, 7))
	brick_locations_left_optimised = np.zeros(shape=(num_bricks, 7))

	route = list(range(int(bricks_per_layer/2)))

	optimised_route = [0]*int(bricks_per_layer/2)
	flag = 0

	for i in range(int(bricks_per_layer/2)):
		if flag == 0:
			optimised_route[i] = min(route)
			route.remove(min(route))
			flag = 1
		elif flag == 1:
			optimised_route[i] = max(route)
			route.remove(max(route))
			flag = 0

	for i in range(num_layers):
		for j in range(int(bricks_per_layer/2)):
			old_brick_number = i*int(bricks_per_layer/2)+j
			new_brick_number = i*int(bricks_per_layer/2)+optimised_route[j]
			brick_locations_right_optimised[old_brick_number] = brick_locations_right[new_brick_number]
			brick_locations_left_optimised[old_brick_number] = brick_locations_left[new_brick_number]

	return [brick_locations_right_optimised, brick_locations_left_optimised]

def plan_path(start, end):

	number_of_steps_curve = 2.00
	number_of_steps_drop = 2.00
	lowering_height = 0.04

	path_sum = number_of_steps_curve+number_of_steps_drop
	path = np.zeros((int(path_sum), 3)) # initialise array of zeros
	step_xyz = [0,0,0] #initialise step

	for i in range (0,3):
		path[0, i] = start[i] #set first step as start position
		step_xyz[i] = (end[i]-start[i])/number_of_steps_curve #define step distance

	step_xyz[2] = (((start[2]+ (end[2]+lowering_height)))/float(number_of_steps_curve)) #define z step distance

	for i in range(1,(int(number_of_steps_curve) + int(number_of_steps_drop))): #incrememnt x and y path according to step_xyz

		if (i <= number_of_steps_curve):
			path[i,0] = float(path[i-1,0]+step_xyz[0])
			path[i,1] = path[i-1,1]+step_xyz[1]
			path[i,2] = ((i**(1./3)) / ((number_of_steps_curve-1)**(-(2./3)))) * (step_xyz[2]) #incremment z path according to curve equation

		else:                                                           #keep x and y the same
			path[i,0] = float(path[i-1,0])
			path[i,1] = path[i-1,1]
			path[i,2] = path[i-1,2] - (lowering_height / number_of_steps_drop)      #lower z according to drop height

	path[int(number_of_steps_curve) + int(number_of_steps_drop)-1] = end                      #ensure final value is equal to target (without this for low values of number_steps_drop the final path can have a small z error)

	return path

