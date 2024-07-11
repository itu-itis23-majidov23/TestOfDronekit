from dronekit import connect, VehicleMode
import time

connection_string = 'COM5'  

def set_motor_speed(vehicle, motor_number, speed):
    """
    Sets the speed of a specific motor.
    motor_number: 1-4 for quadcopters
    speed: 0 to 1 (0% to 100%)
    """
    motor_channels = {
        1: '5',  
        2: '6',  
        3: '7',  
        4: '8',  
    }
    
    if motor_number in motor_channels:
        channel = motor_channels[motor_number]
        pwm_value = 1000 + int(speed * 1000)  
        vehicle.channels.overrides[channel] = pwm_value
    else:
        print(f"Invalid motor number: {motor_number}")

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

    while True:
        motor_input = input("Enter motor number to run (1-4) or 'all' for all motors: ").strip().lower()
        
        if motor_input == 'all':
            print("Running all motors at 7% speed for 7 seconds...")
            for motor_num in range(1, 5):
                set_motor_speed(vehicle, motor_num, 0.07)
            time.sleep(7)
            for motor_num in range(1, 5):
                set_motor_speed(vehicle, motor_num, 0.00)
            print("All motors stopped.")
        elif motor_input in ['1', '2', '3', '4']:
            motor_num = int(motor_input)
            print(f"Running motor {motor_num} at 7% speed for 7 seconds...")
            set_motor_speed(vehicle, motor_num, 0.07)
            time.sleep(7)
            set_motor_speed(vehicle, motor_num, 0.00)
            print(f"Motor {motor_num} stopped.")
        else:
            print("Invalid input. Please enter a valid motor number (1-4) or 'all'.")

        disarm_input = input("Do you want to disarm the vehicle? (yes/no): ").strip().lower()
        if disarm_input == 'yes':
            break

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
