# tf.transformations alternative is not yet available in tf2
from tf.transformations import *
import numpy as np
import math
import cv2
from collections import defaultdict
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import Image

def segment_by_angle_kmeans(lines, k=2, **kwargs):
    # Define criteria = (type, max_iter, epsilon)
    default_criteria_type = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER
    criteria = kwargs.get('criteria', (default_criteria_type, 10, 1.0))
    flags = kwargs.get('flags', cv2.KMEANS_RANDOM_CENTERS)
    attempts = kwargs.get('attempts', 10)
    angles = np.array([line[0][1] for line in lines])

    # multiply the angles by two and find coordinates of that angle
    pts = np.array([[np.cos(2*angle), np.sin(2*angle)]
                    for angle in angles], dtype=np.float32)

    # run kmeans on the coords
    labels, centers = cv2.kmeans(pts, k, None, criteria, attempts, flags)[1:]
    labels = labels.reshape(-1)  # transpose to row vec

    # segment lines based on their kmeans label
    segmented = defaultdict(list)
    for i, line in zip(range(len(lines)), lines):
        segmented[labels[i]].append(line)
    segmented = list(segmented.values())
    return segmented

def intersection(line1, line2):
    rho1, theta1 = line1[0]
    rho2, theta2 = line2[0]
    A = np.array([
        [np.cos(theta1), np.sin(theta1)],
        [np.cos(theta2), np.sin(theta2)]
    ])
    b = np.array([[rho1], [rho2]])
    x0, y0 = np.linalg.solve(A, b)
    x0, y0 = int(np.round(x0)), int(np.round(y0))
    return [[x0, y0]]


def segmented_intersections(lines):
    intersections = []
    for i, group in enumerate(lines[:-1]):
        for next_group in lines[i+1:]:
            for line1 in group:
                for line2 in next_group:
                    intersections.append(intersection(line1, line2))

    return intersections

def brick_boi(image):
    try:
        x_offset = -0.05#-0.02-0.06#0.045-0.07
        y_offset = -0.015#0.022-0.03
        Angular_offset = 0
        #img = cv2.imread(image)
        img = image[:, 216:481]
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 5)
        adapt_type = cv2.ADAPTIVE_THRESH_GAUSSIAN_C
        thresh_type = cv2.THRESH_BINARY_INV
        bin_img = cv2.adaptiveThreshold(blur, 255, adapt_type, thresh_type, 11, 2)
        rho, theta, thresh = 1, np.pi/180, 50
        lines = cv2.HoughLines(bin_img, rho, theta, thresh)
        segmented = segment_by_angle_kmeans(lines)
        intersections = segmented_intersections(segmented)
        for i in range (0,len(intersections)):
            array = np.array(intersections[i])
            plt.plot(array[0,0], array[0,1], 'g+')
        newintersections = []
        for i in range (0,len(intersections)):
            newintersections.append(intersections[i][0])
        kmeans = KMeans(n_clusters=4, random_state=0).fit(newintersections)
        centers4=kmeans.cluster_centers_
        centers=centers4[0:3]
        line1 = [centers[0,0]-centers[1,0], centers[0,1]-centers[1,1]]
        line2 = [centers[0,0]-centers[2,0], centers[0,1]-centers[2,1]]
        line3 = [centers[1,0]-centers[2,0], centers[1,1]-centers[2,1]]
        line1len = math.sqrt((line1[0]**2)+(line1[1]**2))
        line2len = math.sqrt((line2[0]**2)+(line2[1]**2))
        line3len = math.sqrt((line3[0]**2)+(line3[1]**2))
        linelen = [line1len, line2len, line3len]

        values = [0,1,2]
        values.remove(linelen.index(max(linelen)))
        values.remove(linelen.index(min(linelen)))
        if (values[0] == 0):
            shortestline = [centers[0,0]-centers[1,0], centers[0,1]-centers[1,1]]
            angle = np.arctan2(shortestline[0], shortestline[1])
            scalefactor = 0.2/line1len

        if (values[0] == 1):
            shortestline = [centers[0,0]-centers[2,0], centers[0,1]-centers[2,1]]
            angle = np.arctan2(shortestline[0], shortestline[1])
            scalefactor = 0.2/line2len

        if (values[0] == 2):
            shortestline = [centers[1,0]-centers[2,0], centers[1,1]-centers[2,1]]
            angle = np.arctan2(shortestline[0], shortestline[1])
            scalefactor = 0.2/line3len

        if (angle >= math.pi/2):
            angle = angle - math.pi

        if (angle <= -math.pi/2):
            angle = angle + math.pi
        averagecenters = [0,0]
        angle = -angle + Angular_offset

        averagecenters[0] = (centers4[0,0] + centers4[1,0]+ centers4[2,0]+ centers4[3,0])/4
        averagecenters[1] = (centers4[0,1] + centers4[1,1]+ centers4[2,1]+ centers4[3,1])/4

        yerror = y_offset -(scalefactor * (averagecenters[0]- img.shape[1]/2))
        xerror = x_offset - scalefactor * (averagecenters[1]- img.shape[0]/2)
        print("+++++++++++++",scalefactor * (averagecenters[1]- img.shape[0]/2))
        plt.plot(averagecenters[0],averagecenters[1], 'ro')
        plt.plot(img.shape[1]/2,img.shape[0]/2, 'r+')

        plt.imshow(img)
        #plt.show()
        plt.savefig('/home/petar/catkin_ws/src/WELL/scripts/final/perception_test/plot.png')
        plt.clf()
        im1=Image.open('/home/petar/catkin_ws/src/WELL/scripts/final/perception_test/plot.png')
        im1=im1.resize((800,600),Image.BICUBIC)
        im1.save('/home/petar/catkin_ws/src/WELL/scripts/final/perception_test/plot.png')
        for i in range (0, len(centers)):
            plt.plot(centers[i][0],centers[i][1], 'g+')
        return(xerror, yerror, angle)

    except:
        return([0,0,0])

def calculate_brick_locations():
	overhead_orientation = np.array([-0.0249590815779,0.999649402929,0.00737916180073,0.00486450832011])
	#overhead_orientation = np.array([0,0,0,0])
	brick_dimensions = [0.2, 0.04, 0.09]  #Lenght, Width, Height. Should width be 0.04 or 0.062??? It was 0.062, Hugo changed it.

	robot_offset = [0.6, 0.4, -0.22, 0]
	bricks_per_layer = 5
	num_layers = 3
	gap = 0.006 # Gap between each brick corner and corner of polygon.

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
				brick_locations[brick_number, 0] = 0.1*(math.degrees(math.sin(math.radians(angles[j] + (i%2)*(360/(2*bricks_per_layer)) + (robot_offset[3])))) * well_centre_to_brick_origin_lenght) + robot_offset[0]  # Fudge factor of 0.1, we dont know why exactly this came in (perhaps degree/radian conversion) but it is accurate.
				brick_locations[brick_number, 1] = 0.1*(math.degrees(math.cos(math.radians(angles[j] + (i%2)*(360/(2*bricks_per_layer)) + (robot_offset[3])))) * well_centre_to_brick_origin_lenght) + robot_offset[1]
				brick_locations[brick_number, 2] = (i+0.5)*brick_dimensions[2] + robot_offset[2]
				brick_locations[brick_number, 3:7]= quaternion_multiply(quaternion_from_euler(0,0,-math.radians(angles[j] + (i%2)*(360/(2*bricks_per_layer)))), overhead_orientation)  #quaternion_from_euler(0,0,-math.radians(angles[j] + i*(360/(2*bricks_per_layer))))
#quaternion_multiply(quaternion_from_euler(0,0,math.radians(angles[j] + i*(360/(2*bricks_per_layer)))),overhead_orientation)

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

	robot_offset = [0.59, 0, 0.13, 0] #0.15-0.24
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
	lowering_height = 0.2

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
