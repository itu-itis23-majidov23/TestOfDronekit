from dronekit import connect, VehicleMode
import time

# Replace 'COM5' with the correct port for your telemetry module
connection_string = 'COM5'  # Adjust as needed for your setup

def set_motor_speed(vehicle, motor_number, speed):
    """
    Sets the speed of a specific motor.
    motor_number: 1-4 for quadcopters
    speed: 0 to 1 (0% to 100%)
    """
    # Define the channel mappings for motors
    motor_channels = {
        1: '5',  # Motor 1
        2: '6',  # Motor 2
        3: '7',  # Motor 3
        4: '8',  # Motor 4
    }
    
    if motor_number in motor_channels:
        channel = motor_channels[motor_number]
        pwm_value = 1000 + int(speed * 1000)  # PWM value between 1000 (0%) and 2000 (100%)
        vehicle.channels.overrides[channel] = pwm_value
    else:
        print(f"Invalid motor number: {motor_number}")

try:
    # Connect to the Vehicle
    print(f"Connecting to vehicle on: {connection_string}")
    vehicle = connect(connection_string, baud=57600, wait_ready=True)

    # Display basic vehicle state
    print(" Type: %s" % vehicle._vehicle_type)
    print(" Armed: %s" % vehicle.armed)
    print(" System status: %s" % vehicle.system_status.state)

    # Arm the vehicle without waiting for GPS HDOP
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")  # Set mode to guided for arming

    # Wait for arming with a timeout
    timeout = 30  # Timeout in seconds
    start_time = time.time()
    while not vehicle.armed:
        if time.time() - start_time > timeout:
            raise Exception("Timeout waiting for arming")

        # Attempt to arm without GPS check
        vehicle.armed = True
        time.sleep(1)  # Wait for a second to allow the vehicle to change state

    # Vehicle is armed, perform operations here
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

    # Disarm the vehicle
    print("Disarming motors...")
    vehicle.armed = False

    # Wait for disarming to complete
    while vehicle.armed:
        print(" Waiting for disarming...")
        time.sleep(1)

    print("Vehicle disarmed.")

except Exception as e:
    print(f"Error: {e}")

finally:
    # Close vehicle object before exiting script
    if 'vehicle' in locals():
        vehicle.close()
        print("Connection closed")

print("Completed")
