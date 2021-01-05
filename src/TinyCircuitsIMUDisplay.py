
#################################
#	Imports and module names	#
#################################

#Import python packages for c types and
# os functions
import os
import sys
sys.path.append('..')

# import 3D rendering module
import vtk

# import pyserial tools to list ports
# and find where the arduino is connected
import serial
import serial.tools.list_ports
from time import sleep

# import WiFi socket module for python server
import socket

USB = 1
WIFI = 2

COMMUNICATION = USB		# options: USB, WIFI

# Connection object for USB, if USB is used
serial_connection = 0
vtk_render_window = 0
last_quat = 0

# WiFi socket object for communication to client Arduino
wifi = 0
listen_port = 8090;
client = 0;
client_addr = 0;

# Global variables for vtk in this script
vtk_render_window = 0
vtk_actors = 0


def get_orientation_data():
	if COMMUNICATION == USB:
		return serial_connection.readline().decode().split()
	elif COMMUNICATION == WIFI:
		# Quaterion string and char count: 0 -0.00 -0.00 -0.00 -0.00 [26 chars]
		return client.recv(26).decode().split()


def zero_orientation(self, obj):
	print("Zeroed board orientation!")


def information_callback(self, obj):
	rotate_data = get_orientation_data()

	if len(rotate_data) == 3:
		vtk_actors.InitTraversal()
		vtk_next_actor = vtk_actors.GetNextActor()
		while vtk_next_actor != None:
			vtk_next_actor.SetOrientation(-90, 90, 0)
			vtk_next_actor.RotateX(float(rotate_data[1]))
			vtk_next_actor.RotateY(float(rotate_data[2]))
			vtk_next_actor.SetScale(0.6, 0.6, 0.6)
			vtk_next_actor.GetProperty().LightingOff()
			vtk_next_actor = vtk_actors.GetNextActor()
		vtk_render_window.Render()

	# ensure four elements for four quaterion components
	elif len(rotate_data) == 5:
		global last_quat
		last_quat = vtk.vtkQuaternionf(float(rotate_data[1]), float(rotate_data[2]), float(rotate_data[3]), float(rotate_data[4]))
		t = [ [0]*3 for i in range(3)]
		last_quat.ToMatrix3x3(t)
		temp_matrix = vtk.vtkMatrix4x4()

		temp_matrix.SetElement(0, 0, t[0][0])
		temp_matrix.SetElement(0, 1, t[0][1])
		temp_matrix.SetElement(0, 2, t[0][2])

		temp_matrix.SetElement(1, 0, t[1][0])
		temp_matrix.SetElement(1, 1, t[1][1])
		temp_matrix.SetElement(1, 2, t[1][2])

		temp_matrix.SetElement(2, 0, t[2][0])
		temp_matrix.SetElement(2, 1, t[2][1])
		temp_matrix.SetElement(2, 2, t[2][2])

		temp_matrix.SetElement(3, 0, 0)
		temp_matrix.SetElement(3, 1, 0)
		temp_matrix.SetElement(3, 2, 0)
		temp_matrix.SetElement(3, 3, 1)

		temp_matrix.SetElement(0, 3, 0)
		temp_matrix.SetElement(1, 3, 0)
		temp_matrix.SetElement(2, 3, 0)

		vtk_actors.InitTraversal()
		vtk_next_actor = vtk_actors.GetNextActor()
		while vtk_next_actor != None:
			vtk_next_actor.SetUserMatrix(temp_matrix)
			vtk_next_actor.SetScale(0.6, 0.6, 0.6)
			vtk_next_actor.GetProperty().LightingOff()
			vtk_next_actor = vtk_actors.GetNextActor()
		vtk_render_window.Render()


def connect_usb():
	print("\n*Searching for plugged in Arduino USB ports...")

	arduino_ports = list(serial.tools.list_ports.comports())
	port_number = 0

	print("*Found COM ports:")
	port_index = 0
	for port in arduino_ports:
		print(port_index, "\t", port)
		port_index = port_index + 1

	if not arduino_ports:
		print("*No Arduino boards were found to be plugged in - stopping debugger")
		exit()
	elif len(arduino_ports) > 1:
		print("*More than one COM port connection detected - enter number correlating to port to use (e.g. '0', '1', etc...)")
		try:
			port_number = int(input("*Enter number from above: "))
		except ValueError:
			print("*ERROR: Character entered was not a number - stopping debugger")
			exit()

	print("*Using board port \"", arduino_ports[port_number], "\" for IMU data stream")
	print("*Attempting to open port", arduino_ports[port_number].device)

	try:
		global serial_connection
		serial_connection = serial.Serial(arduino_ports[port_number].device, 115200)
		print("*Successfully opened device port\n")
		return 1
	except:
		print("\n*Could not open port: either the device was unplugged, another process is using the port, or permission denied - stopping debugger\n")
		exit()


def connect_wifi():
	print("\n*Connecting using WiFi...\n")
	wifi = socket.socket()
	wifi.bind(('0.0.0.0', listen_port))	# Bind to address of this computer
	wifi.listen(0)						# Only allow one connection, refuse the rest
	print("*Waiting for Arduino to connect through WiFi...")
	global client
	global client_addr
	client, client_addr = wifi.accept()	# Wait for the client Arduino to connect
	print("*Arduino client WiFi connection established!", client_addr, "\n")
	return 1


def init_3D_scene(board_file_name):
	data_root = os.path.join(os.path.dirname(__file__), 'data')
	importer = vtk.vtkGLTFImporter()
	importer.SetFileName(data_root + board_file_name)
	importer.Read()

	vtk.vtkButtonWidget()

	global vtk_render_window
	vtk_renderer = importer.GetRenderer()
	vtk_render_window = importer.GetRenderWindow()
	vtk_render_window_interactor = vtk.vtkRenderWindowInteractor()
	vtk_render_window_interactor.SetRenderWindow(vtk_render_window)

	vtk_zero_button = vtk.vtkButtonWidget()
	vtk_zero_button_representation = vtk.vtkTexturedButtonRepresentation2D()

	vtk_zero_button_representation.SetNumberOfStates(1)
	vtk_png_reader = vtk.vtkPNGReader()
	vtk_png_reader.SetFileName("zero.png")
	vtk_png_reader.Update()
	image = vtk_png_reader.GetOutput()
	vtk_zero_button_representation.SetButtonTexture(0, image)
	vtk_zero_button.SetRepresentation(vtk_zero_button_representation)
	vtk_zero_button.SetInteractor(vtk_render_window_interactor)
	vtk_zero_button.SetCurrentRenderer(vtk_renderer)
	vtk_zero_button.AddObserver("StateChangedEvent", zero_orientation)
	vtk_zero_button.On()

	width, height, _ = image.GetDimensions()
	bounds = (0, 0 + width, 0, height, 0, 0)
	vtk_zero_button_representation.SetPlaceFactor(1)
	vtk_zero_button_representation.PlaceWidget(bounds)

	vtk_renderer.GradientBackgroundOn()
	vtk_renderer.SetBackground(0.2, 0.2, 0.2)
	vtk_renderer.SetBackground2(0.3, 0.3, 0.3)
	vtk_render_window.SetSize(600, 600)
	vtk_render_window.SetWindowName('TinyCircuits: IMU 3D Visualizer')

	vtk_render_window_interactor.Initialize()
	vtk_renderer.GetActiveCamera().Zoom(1.0)
	vtk_renderer.GetActiveCamera().SetRoll(90)
	vtk_renderer.GetActiveCamera().SetClippingRange(0.01, 100)
	vtk_renderer.GetActiveCamera().SetViewAngle(40)
	vtk_renderer.SetClippingRangeExpansion(0.1)					# Adjust so clipping of polygons doesn't show
	vtk_renderer.TwoSidedLightingOn()
	vtk_renderer.SetAmbient([1, 1, 1])
	vtk_renderer.ResetCamera()
	vtk_render_window.Render()

	# Add callback for getting data from Arduino
	vtk_render_window_interactor.CreateRepeatingTimer(1)
	vtk_render_window_interactor.AddObserver("TimerEvent", information_callback)

	global vtk_actors
	vtk_actors = vtk_renderer.GetActors()
	vtk_render_window_interactor.Start()


def main():
	if COMMUNICATION == USB:
		print("\n*Looking for USB connection...\n")
		connect_usb()
	elif COMMUNICATION == WIFI:
		print("\n*Looking for WiFi connection...\n")
		connect_wifi()

	# Let wifi connect if it was used
	sleep(0.25)

	# Every string sent is appended with the board id that is sending the string
	board_id = get_orientation_data()[0]
	if board_id == "1":
		init_3D_scene("/TinyZero.glb")
	elif board_id == "2":
		init_3D_scene("/RobotZero.glb")
	elif board_id == "3":
		init_3D_scene("/Wireling9Axis.glb")
	elif board_id == "4":
		init_3D_scene("/Wireling3Axis.glb")


if __name__ == '__main__':
	main()