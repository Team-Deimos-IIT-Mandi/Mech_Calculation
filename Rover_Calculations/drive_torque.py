def calculate_torque(friction_coefficient, num_motors, wheel_diameter, rover_weight):
    """
    Calculate the torque needed per motor for a 4-wheel drive rover on plane ground.

    Parameters:
        friction_coefficient (float): Coefficient of friction between the wheels and the ground.
        num_motors (int): Number of driving motors.
        wheel_diameter (float): Diameter of the wheels in meters.
        rover_weight (float): Weight of the rover in kilograms.

    Returns:
        float: Torque needed per motor in Nm.
    """
    # Gravitational acceleration (m/s^2)
    g = 9.81

    # Normal force (N)
    normal_force = rover_weight * g

    # Total traction force required (N)
    traction_force = friction_coefficient * normal_force

    # Torque per motor (Nm)
    torque_per_motor = (traction_force * (wheel_diameter / 2)) / num_motors

    return torque_per_motor


if __name__ == "__main__":
    print("Debug: Starting program")  # Add this line

    # Fixed parameters
    friction_coefficient = 0.1  # friction coefficient
    num_motors = 4    # number of drive motors
    wheel_diameter = 0.25  # wheel diameter in meters (25 cm)
    rover_weight = 50.0  # rover weight in kg

    # Calculate torque
    torque = calculate_torque(friction_coefficient, num_motors, wheel_diameter, rover_weight)

    print(f"Parameters used:")
    print(f"Friction coefficient: {friction_coefficient}")
    print(f"Number of motors: {num_motors}")
    print(f"Wheel diameter: {wheel_diameter} m")
    print(f"Rover weight: {rover_weight} kg")
    print(f"\nTorque needed per motor: {torque:.2f} Nm")
