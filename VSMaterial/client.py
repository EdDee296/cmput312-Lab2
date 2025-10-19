#!/usr/bin/python3       
# RUN ON BRICK
    
import socket
import os
import time
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B
from ev3dev2.motor import SpeedPercent
from fk import calibrate_zero

# This class handles the client side of communication. It has a set of predefined messages to send to the server as well as functionality to poll and decode data.
class Client:
    def __init__(self, host, port):
        # We need to use the ipv4 address that shows up in ipconfig in the computer for the USB. Ethernet adapter handling the connection to the EV3
        print("Setting up client\nAddress: " + host + "\nPort: " + str(port))
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.s.connect((host, port))
        print("Connected successfully!")
        
        # Initialize motors
        self.base_motor = LargeMotor(OUTPUT_A)   # Base motor (Joint 1)
        self.joint_motor = LargeMotor(OUTPUT_B)  # Joint motor (Joint 2)
        
        # Set initial stop action to hold
        self.base_motor.stop_action = 'hold'
        self.joint_motor.stop_action = 'hold'
        
        # Track safety mode state
        self.safety_mode = False
        
    # Block until a message from the server is received. When the message is received it will be decoded and returned as a string.
    # Output: UTF-8 decoded string containing the instructions from server.
    def pollData(self):
        print("Waiting for Data")
        data = self.s.recv(128).decode("UTF-8")
        print("Data Received: " + data)
        return data
    
    # Sends a message to the server letting it know that the movement of the motors was executed without any inconvenience.
    def sendDone(self):
        self.s.send("DONE".encode("UTF-8"))

    # Sends a message to the server letting it know that there was an issue during the execution of the movement (obstacle avoided) and that the initial jacobian should be recomputed (Visual servoing started from scratch)
    def sendReset(self):
        self.s.send("RESET".encode("UTF-8"))
    
    # Move the motors based on received angles
    def moveMotors(self, base_angle, joint_angle):
        print("Moving base: " + str(base_angle) + " degrees")
        print("Moving joint: " + str(joint_angle) + " degrees")
        
        # Move base motor normally
        self.base_motor.on_for_degrees(SpeedPercent(20), base_angle, brake=True, block=False)
        
        # Move joint motor with inverted angle (since it's upside down)
        self.joint_motor.on_for_degrees(SpeedPercent(20), -joint_angle, brake=True, block=False)
        
        # Wait for both motors to complete
        self.base_motor.wait_until_not_moving()
        self.joint_motor.wait_until_not_moving()
    
        if not self.safety_mode:
            self.base_motor.stop_action = 'coast'
            self.joint_motor.stop_action = 'coast'
            self.base_motor.stop()
            self.joint_motor.stop()

        print("Movement complete")
    
    # Enable safety mode - motors locked
    def enableSafetyMode(self):
        print("Safety mode ENABLED - motors locked")
        self.safety_mode = True
        self.base_motor.stop_action = 'hold'
        self.joint_motor.stop_action = 'hold'
        self.base_motor.stop()
        self.joint_motor.stop()

    
    # Disable safety mode - motors are free to move
    def disableSafetyMode(self):
        print("Safety mode DISABLED - motors are free to move")
        self.safety_mode = False
        self.base_motor.stop_action = 'coast'
        self.joint_motor.stop_action = 'coast'
        self.base_motor.stop()
        self.joint_motor.stop()


host = "169.254.207.188"
port = 9999
client = Client(host, port)
calibrate_zero()
while True:
    data = client.pollData()
    
    # Check for special commands
    if data == "EXIT":
        print("Received EXIT command")
        client.base_motor.stop()
        client.joint_motor.stop()
        break
    elif data == "SAFETY_ON":
        client.enableSafetyMode()
        client.sendDone()
        continue
    elif data == "SAFETY_OFF":
        client.disableSafetyMode()
        client.sendDone()
        continue
    
    # Parse the angles
    try:
        angles = data.split(",")
        base_angle = float(angles[0])
        joint_angle = float(angles[1])
        
        # Move the motors
        client.moveMotors(base_angle, joint_angle)
        
        # Send DONE back to server
        client.sendDone()
    except Exception as e:
        print("Error processing data: " + str(e))
        client.sendReset()