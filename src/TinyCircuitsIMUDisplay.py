
#################################
#	Imports and module names	#
#################################

#Import python packages for c types and
# os functions
import ctypes
import os
import sys
import glob
import math
import struct

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
vtkRenderWindow = 0
last_quat = 0

# WiFi socket object for communication to client Arduino
wifi = 0
listen_port = 8090;
client = 0;
client_addr = 0;


def GetOrientationData():
	if COMMUNICATION == USB:
		return serial_connection.readline().decode().split()
	elif COMMUNICATION == WIFI:
		# Quaterion string and char count: 0 -0.00 -0.00 -0.00 -0.00 [26 chars]
		return client.recv(26).decode().split()


def information_callback(self, obj):

	rotate_data = GetOrientationData()
	
	if len(rotate_data) == 3:
		vtkActors.InitTraversal()
		vtkNextActor = vtkActors.GetNextActor()
		while vtkNextActor != None:
			vtkNextActor.SetOrientation(-90, 90,0)
			vtkNextActor.RotateX(float(rotate_data[1]))
			vtkNextActor.RotateY(float(rotate_data[2]))
			vtkNextActor.SetScale(0.6, 0.6, 0.6)
			vtkNextActor.GetProperty().LightingOff()
			vtkNextActor = vtkActors.GetNextActor()
		vtkRenderWindow.Render()

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

		vtkActors.InitTraversal()
		vtkNextActor = vtkActors.GetNextActor()
		while vtkNextActor != None:
			vtkNextActor.SetUserMatrix(temp_matrix)
			vtkNextActor.SetScale(0.6, 0.6, 0.6)
			vtkNextActor.GetProperty().LightingOff()
			vtkNextActor = vtkActors.GetNextActor()
		vtkRenderWindow.Render()


def ConnectUSB():
	print("\n*Searching for plugged in Arduino USB ports...")

	arduino_ports = list(serial.tools.list_ports.comports())

	if not arduino_ports:
		print("*No Arduino boards were found to be plugged in - stopping debugger")
		exit()
	elif len(arduino_ports) > 1:
		print("More than one Arduino board is plugged in - using the first board")

	print("*Found USB Arduino boards:")
	for port in arduino_ports:
		print("\t", port)
		
	print("*Using board port \"", arduino_ports[0], "\" for IMU data stream")
	print("*Attempting to open port", arduino_ports[0].device)

	try:
		global serial_connection
		serial_connection = serial.Serial(arduino_ports[0].device, 115200)
		print("*Successfully opened device port\n")
		connection_established = 1
		return 1
	except:
		print("\n*Could not open port: either the device was unplugged, another process is using the port, or permission denied - stopping debugger\n")
		exit()


def ConnentWiFi():
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


def Init3DScene(board_file_name):
	data_root = os.path.join(os.path.dirname(__file__), 'data')
	importer = vtk.vtkGLTFImporter()
	importer.SetFileName( data_root + board_file_name )
	importer.Read()
	
	global vtkRenderWindow
	vtkRenderer = importer.GetRenderer()
	vtkRenderWindow = importer.GetRenderWindow()
	vtkRenderWindowInteractor = vtk.vtkRenderWindowInteractor()
	vtkRenderWindowInteractor.SetRenderWindow(vtkRenderWindow)

	vtkRenderer.GradientBackgroundOn()
	vtkRenderer.SetBackground(0.2, 0.2, 0.2)
	vtkRenderer.SetBackground2(0.3, 0.3, 0.3)
	vtkRenderWindow.SetSize(600, 600)
	vtkRenderWindow.SetWindowName('TinyCircuits: IMU 3D Visualizer')

	vtkRenderWindowInteractor.Initialize()
	vtkRenderer.GetActiveCamera().Zoom(1.0)
	vtkRenderer.GetActiveCamera().SetRoll(90)
	vtkRenderer.GetActiveCamera().SetClippingRange(0.01, 100)
	vtkRenderer.GetActiveCamera().SetViewAngle(40)
	vtkRenderer.SetClippingRangeExpansion(0.1)					# Adjust so clipping of ploygons doesn't show
	vtkRenderer.TwoSidedLightingOn()
	vtkRenderer.SetAmbient([1, 1, 1])
	vtkRenderer.ResetCamera()
	vtkRenderWindow.Render()

	# Add callback for getting data from Arduino
	vtkRenderWindowInteractor.CreateRepeatingTimer(1)
	vtkRenderWindowInteractor.AddObserver("TimerEvent", information_callback)

	global vtkActors
	vtkActors = vtkRenderer.GetActors()
	vtkRenderWindowInteractor.Start()


def main():
	if COMMUNICATION == USB:
		print("\n*Looking for USB connection...\n")
		ConnectUSB()
	elif COMMUNICATION == WIFI:
		print("\n*Looking for WiFi connection...\n")
		ConnentWiFi()

	sleep(0.25)

	board_id = GetOrientationData()[0]
	print(board_id)
	if board_id == "1":
		Init3DScene("/TinyZero.glb")
	elif board_id == "2":
		Init3DScene("/RobotZero.glb")


if __name__ == '__main__':
	main()