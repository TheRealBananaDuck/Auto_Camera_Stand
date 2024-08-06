import time
import maestro_serial
import math
import numpy as np
import cv2
import matplotlib.pyplot as plt
import serial
import socket
from threading import Timer

def maestro_scale(values, wave_range, degree_range = [0,90]):
	return int((np.interp(values, degree_range, wave_range) * 4))

def find_largest_orange_object(frame):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define the range for orange color in HSV
    lower_orange = np.array([160, 100, 50])
    upper_orange = np.array([180, 255, 255])
    
    # Create a mask for the orange color
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get the bounding box of the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Draw the bounding box on the frame
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    return frame, [x+(w/2),y+h]
    

def dist_calc(vert_pixels, detected_pixel, vert_fov, angle, height):
	angle2 = ((vert_fov/vert_pixels) * (vert_pixels/2 - (detected_pixel)))
	return height * math.tan(math.radians(angle+angle2))
	
def calculate_object_distance(camera_height, tilt_angle, bottom_pixel_position, image_height=600, fov_vertical=40):

    angle2 = ((fov_vertical/image_height) * (image_height/2 - (bottom_pixel_position)))
    return 14.5 * math.tan(math.radians(50 + angle2))

def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size) / window_size, mode='valid')


def distance_est(image, camera_angle, lense_to_mount, base_height_original, vertical_pixels, field_of_view_vertical, lowerbound, upperbound, cameraMatrix, dist, camera_tilt_distance_correction_list, base_height_modified_list):
    frame_with_box, vert_detected_pixel = find_largest_orange_object(image)

    vert_detected_pixel = distortion_Correction_Point(np.array([vert_detected_pixel], dtype=np.float32), cameraMatrix, dist)[0][1]
    
    base_height_modified_list.append(base_height_original - math.sin(math.radians(camera_angle))*lense_to_mount)
    
    if len(base_height_modified_list) > 6:
        base_height_modified = base_height_modified_list[-6]
    else:
        base_height_modified = 13

    camera_tilt_distance_correction_list.append(math.sin(math.radians(camera_angle))*lense_to_mount)
    
    distance = dist_calc(vertical_pixels,vert_detected_pixel, field_of_view_vertical,camera_angle, base_height_modified)
    
    if len(camera_tilt_distance_correction_list) > 1:
        distance = distance + camera_tilt_distance_correction_list[-1]
    
    distance = distance * 1.37441 + 0.225118



    return frame_with_box, distance, camera_tilt_distance_correction_list, base_height_modified_list
#Servo 1 Range is set to 40 and 60 degrees. 

def distortion_Correction_Point(point, cameraMatrix, dist):
    # Reshape the point for undistortPoints
    points = np.array([point], dtype=np.float32)
    undistorted_points = cv2.undistortPoints(points, cameraMatrix, dist, P=cameraMatrix)
    undistorted_points = undistorted_points.get().reshape(-1, 2)  # Reshape back to original form
    return undistorted_points

def update_plot(new_x, new_y):
    xdata.append(new_x)
    ydata.append(new_y)
    
    # Update line data
    graph_line.set_data(xdata, ydata)
    
    # Adjust plot limits if necessary
    ax.set_xlim(0, max(xdata) + 1)
    ax.set_ylim(min(ydata) - 1, max(ydata) + 1)
    
    # Draw only the elements that have changed
    fig.canvas.restore_region(background)
    ax.draw_artist(graph_line)
    fig.canvas.blit(ax.bbox)
    fig.canvas.flush_events()
    
    
ret = 0.17487892034052818 
cameraMatrix_nparray = np.array([[614.9169702, 0, 374.30485171],[0, 614.42926413, 305.12053662],[0, 0, 1]])
# Convert the NumPy array to a cv::UMat object
cameraMatrix = cv2.UMat(cameraMatrix_nparray)
dist_nparray = np.array([[-4.18125759e-01,2.66627964e-01,9.43726035e-05,-9.79700938e-04 ,-1.13138137e-01]])
dist = cv2.UMat(dist_nparray)


cam = cv2.VideoCapture(0)
#Camera resolution 1920 x 1080
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
try:
	maestro_controller = maestro_serial.Maestro(device='/dev/ttyACM0',baudrate=115200)
except:
	maestro_controller = maestro_serial.Maestro(device='/dev/ttyACM1',baudrate=115200)

maestro_controller.set_target(1,maestro_scale(90, [1000, 2000]))
maestro_controller.set_speed(1,20)


time.sleep(6)


if __name__ == '__main__':
	ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
	ser.reset_input_buffer()
	for i in range(0,15):
		line = ser.readline().decode('utf-8').rstrip()
		print(line)


	maestro_controller.set_target(2,8000)
	#was at 8000
	maestro_controller.set_speed(2,18)
	maestro_controller.set_target(1,maestro_scale(30, [1000, 2000]))
	maestro_controller.set_speed(1,7)	

	time.sleep(2)

	base_height_original = 16.5
	#Max height = 16.3
	camera_angle = 1
	field_of_view_vertical = 36
	vertical_pixels = 600
	lense_to_mount = 2
	height = 404

	dist_list = []
	dist_list2 = []
	final_distance_list = []
	final_distance_list2 = []
	final_distance_list3 = []
	base_height_modified_list = []
	camera_tilt_distance_correction_list = []
	dist_list3 = []
	angle_list = []
	angle_buffer = 1
	process_count = -25
	pass_counter = 0
	uncorrected_distance1= 0
	uncorrected_distance2= 0
	uncorrected_distance3 = 0
	uncorrected_dist_list1 = []
	uncorrected_dist_list2 = []
	uncorrected_dist_list3 = []
	message = ""
	xdata, ydata = [], []
	fig, ax = plt.subplots()
	graph_line, = ax.plot([], [], 'r-')
	plt.ion()
	fig.canvas.draw()
	background = fig.canvas.copy_from_bbox(ax.bbox)

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind(('',5000))
	s.listen(5)

	print('Server is now running.')
	clientsocket, address = s.accept()
	print(f"Connection from {address} has been established")


	while(True):


		base_height_original = maestro_scale(maestro_controller.get_position(2), [16.5, 13.5], [4000,8000])/4
		time.sleep(0.001)
		ret, image = cam.read()


		line = ser.readline().decode('utf-8').rstrip()
		process_count += 1
		if line == "":
			print("empty")
			continue
		if abs((float(line)) - angle_buffer) > 5 and process_count > 5:
			print("Passed")
			pass_counter += 1
			if pass_counter > 5:
				camera_angle = float(line)
				angle_buffer = camera_angle
				pass_counter = 0
			continue
		else:
			camera_angle = float(line)
			angle_buffer = camera_angle
			pass_counter = 0

		if len(angle_list) > 2:
			camera_angle = (angle_list[-2])

		ser.reset_input_buffer()
		if (maestro_controller.get_position(2) > 7950):
			maestro_controller.set_target(2,maestro_scale(0, [800, 2200]))
		elif (maestro_controller.get_position(2) < 4050):
			maestro_controller.set_target(2,maestro_scale(90, [800, 2200]))

		if (maestro_controller.get_position(1) > maestro_scale(49, [1000, 2000])):
			maestro_controller.set_target(1,maestro_scale(30, [1000, 2000]))
		elif (maestro_controller.get_position(1) < maestro_scale(31, [1000, 2000])):
			maestro_controller.set_target(1,maestro_scale(50, [1000, 2000]))


		frame_with_box1, distance1, camera_tilt_distance_correction_list, base_height_modified_list = distance_est(image, camera_angle, lense_to_mount, base_height_original, height, field_of_view_vertical, [160, 170, 115], [200, 210, 155], cameraMatrix, dist , camera_tilt_distance_correction_list, base_height_modified_list)



		if process_count > 5:
			dist_list.append(distance1)
			message = dist_list[-1]

			# update_plot(len(dist_list), dist_list[-1]) 
			# plt.pause(0.001)

		#cv2.imshow("Image", frame_with_box1)
		print(message)
		clientsocket.send(bytes(str(message), "utf-8"))


		k = cv2.waitKey(1)
		if k!= -1:
			break
	



	cam.release()
	cv2.destroyAllWindows()

