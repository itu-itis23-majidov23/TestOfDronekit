from dronekit import connect, VehicleMode
import time

connection_string = 'COM5'  

try:
    print(f"Connecting to vehicle on: {connection_string}")
    vehicle = connect(connection_string, baud=57600, wait_ready=True)
    print(" Type: %s" % vehicle._vehicle_type)
    print(" Armed: %s" % vehicle.armed)
    print(" System status: %s" % vehicle.system_status.state)
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED") 
    timeout = 30  
    start_time = time.time()

    while not vehicle.armed:
        if time.time() - start_time > timeout:
            raise Exception("Timeout waiting for arming")

        vehicle.armed = True
        time.sleep(1)  

    print("Vehicle armed and ready!")
    time.sleep(5) 
    print("Disarming motors...")
    vehicle.armed = False

    while vehicle.armed:
        print(" Waiting for disarming...")
        time.sleep(1)

    print("Vehicle disarmed.")

except Exception as e:
    print(f"Error: {e}")

finally:
    if 'vehicle' in locals():
        vehicle.close()
        print("Connection closed")

print("Completed")
