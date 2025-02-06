import serial 
import time 
def begin_serial_connection(DEVICENAME= '/dev/ttyUSB0'):
	serial_inst = serial.Serial()
	serial_inst.baudrate= 115200
	serial_inst.port = DEVICENAME
	serial_inst.open()
	return serial_inst

def call_arduino(command, serial_inst, ack_time_out = 10, func_time_out= 120):
	# if command is a string, sends message to Arduino
	if isinstance(command, str): # to check if the command is a string or not. 
		command= command.encode('utf-8')
		serial_inst.write(command) # writes the comand to arduino
	# waits for Arduino to acknowledge that python command was received
	ack_received = False #initiate flag
	start_time= time.time() # record start time, ack_time_out is in seconds. 
	while time.time()-start_time < ack_time_out: # 
		if serial_inst.in_waiting: # there is bytes sent from Arduino
			response= serial_inst.readline().decode('utf-8').strip() # reads the Serial.print() in ardunino
			if response == "ACK":
				ack_received = True
	if ack_received: # 
		print(f"Initiated {command.decode('utf-8')} processing...")
	else:
		print("Acknowledgment not received.")
		return
	# outputs Arduino serial monitor outputs onto command line 
	# until Arduino function terminates or timeout is reached 
	func_complete = False
	start_time= time.time()
	while not func_complete and time.time() - start_time < func_time_out:
		response= serial_inst.readline().decode('utf-8').strip()
		print(response)
		print('in while loop')
		if response.endswith("complete!"):
			func_complete = True 
	if func_complete:
		print(f"{command.decode('utf-8')} complete!")
	else:
		print(f"{command.decode('utf-8')} did not complete within timeout.")


def end_serial_connection(serial_inst):
	serial_inst.close()


	