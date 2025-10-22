"""
Robotic Arm Torque Calculator for Last Three Joints (J4, J5, J6)
================================================================
J4: Axial rotation of Link 3 (tube)
J5: Lifting joint (at end of Link 3)
J6: Axial rotation at end (gripper attached)
Note: All motors are at the back of Link 3
"""

import math

# ===========================
# CONFIGURATION PARAMETERS
# ===========================

# Link 3 (tube) parameters
LINK_3_LENGTH = 0.5  # Length of link 3 (meters)
LINK_3_MASS = 1.5  # Total mass of link 3 including motors (kg)
LINK_3_RADIUS = 0.035  # Outer radius for axial rotation inertia calculation (meters)

# Motor parameters (two identical motors mounted on the tube, opposite sides)
MOTOR_MASS = 0.3  # Mass of each motor (kg)
MOTOR_RADIAL_OFFSET = 0.05  # Radial distance from J4 axis to motor COM (meters)

# Gripper parameters
MASS_GRIPPER = 0.4  # Mass of gripper (kg)
GRIPPER_DISTANCE_FROM_J5 = 0.1  # Distance from J5 axis (meters)
GRIPPER_RADIUS = 0.1  # Radius for axial rotation calculation (meters)

# Payload parameters
MASS_PAYLOAD = 5.0  # Payload mass (kg)
PAYLOAD_DISTANCE_FROM_J5 = 0.2  # Distance from J5 axis (meters)
PAYLOAD_RADIUS = 0.05  # Radius for axial rotation calculation (meters)

# J4 Radial offsets (perpendicular distance from J4 rotation axis)
GRIPPER_RADIAL_OFFSET = 0.1  # How far gripper COM is from J4 axis (meters)
PAYLOAD_RADIAL_OFFSET = 0.2  # How far payload COM is from J4 axis (meters)

# Holding torque: when the gripper is offset from J4 axis there will be a
# steady gravitational moment that the J4 motor(s) must resist to hold
# the orientation (this is not part of dynamic inertia but a static
# holding torque). Set to None to compute from gripper weight and offset,
# or set a numeric torque (N·m) to override.
GRIPPER_HOLDING_TORQUE_OVERRIDE = None  # N·m or None to auto-calc

# Safety factor
SAFETY_FACTOR = 2.0

# Gravity
g = 9.81  # m/s^2

# Friction / bearing estimates
BEARING_FRICTION_COEFF = 0.005  # coefficient for bearing friction (unitless, estimate)
BEARING_EFFECTIVE_RADIUS = 0.02  # effective bearing radius where friction acts (m)
MOTOR_FRICTION_PER_MOTOR = 0.02  # static/friction torque each motor exhibits (N·m), estimate

# Angular acceleration (rad/s^2) - for dynamic torque calculation
ALPHA_J4 = 8.0  # Angular acceleration for J4
ALPHA_J5 = 5.0  # Angular acceleration for J5
ALPHA_J6 = 15.0  # Angular acceleration for J6

# ===========================
# HELPER FUNCTIONS
# ===========================

def calculate_tube_inertia(mass, radius):
    """Calculate moment of inertia for a tube about its central axis"""
    return mass * radius**2

def calculate_point_mass_inertia(mass, distance):
    """Calculate moment of inertia for a point mass at distance from axis"""
    return mass * distance**2

def calculate_disc_inertia(mass, radius):
    """Moment of inertia for a solid disc about its central axis: (1/2) m r^2"""
    return 0.5 * mass * radius**2

def calculate_solid_sphere_inertia(mass, radius):
    """Moment of inertia for a solid sphere about its central axis: (2/5) m r^2"""
    return (2.0/5.0) * mass * radius**2

# ===========================
# TORQUE CALCULATIONS
# ===========================

def calculate_j6_torque():
    """
    J6: Axial rotation joint (spins gripper and payload)
    Only needs to overcome rotational inertia
    """
    print("\n" + "="*60)
    print("J6 TORQUE CALCULATION (Axial Rotation)")
    print("="*60)
    
    # Moment of inertia for gripper (disc) and payload (solid sphere)
    # For J6 (axial spin) the rotation axis goes through their centers, so
    # use their geometric inertias about their own centers
    I_gripper = calculate_disc_inertia(MASS_GRIPPER, GRIPPER_RADIUS)
    I_payload = calculate_solid_sphere_inertia(MASS_PAYLOAD, PAYLOAD_RADIUS)

    total_inertia = I_gripper + I_payload
    
    # Dynamic torque
    torque_dynamic = total_inertia * ALPHA_J6
    
    # Static torque (friction)
    torque_static = 0.1  # Estimate for friction
    
    torque_required = torque_dynamic + torque_static
    torque_with_safety = torque_required * SAFETY_FACTOR
    
    print(f"Total rotational inertia: {total_inertia:.6f} kg·m²")
    print(f"Dynamic torque: {torque_dynamic:.3f} N·m")
    print(f"Static torque (friction): {torque_static:.3f} N·m")
    print(f"Required torque: {torque_required:.3f} N·m")
    print(f"With safety factor ({SAFETY_FACTOR}x): {torque_with_safety:.3f} N·m")
    
    return torque_with_safety

def calculate_j5_torque():
    """
    J5: Lifting joint (at end of Link 3)
    """
    print("\n" + "="*60)
    print("J5 TORQUE CALCULATION (Lifting)")
    print("="*60)
    
    # Gravitational torques (worst case: horizontal arm)
    torque_gripper = MASS_GRIPPER * g * GRIPPER_DISTANCE_FROM_J5
    torque_payload = MASS_PAYLOAD * g * PAYLOAD_DISTANCE_FROM_J5
    
    torque_gravity_total = torque_gripper + torque_payload
    
    # Rotational inertia for dynamic torque (treat as point masses about J5)
    I_gripper = calculate_point_mass_inertia(MASS_GRIPPER, GRIPPER_DISTANCE_FROM_J5)
    I_payload = calculate_point_mass_inertia(MASS_PAYLOAD, PAYLOAD_DISTANCE_FROM_J5)
    
    total_inertia = I_gripper + I_payload
    
    torque_dynamic = total_inertia * ALPHA_J5
    
    torque_required = torque_gravity_total + torque_dynamic
    torque_with_safety = torque_required * SAFETY_FACTOR
    
    print(f"Gravitational torque breakdown:")
    print(f"  Gripper: {torque_gripper:.3f} N·m")
    print(f"  Payload: {torque_payload:.3f} N·m")
    print(f"Total gravitational torque: {torque_gravity_total:.3f} N·m")
    print(f"Rotational inertia: {total_inertia:.6f} kg·m²")
    print(f"Dynamic torque: {torque_dynamic:.3f} N·m")
    print(f"Required torque: {torque_required:.3f} N·m")
    print(f"With safety factor ({SAFETY_FACTOR}x): {torque_with_safety:.3f} N·m")
    
    return torque_with_safety

def calculate_j4_torque():
    """
    J4: Axial rotation of Link 3 (tube)
    Must rotate Link 3 (with motors) and gripper for dynamics. Payload is
    excluded from the dynamic inertia per requirement. However, payload
    still contributes to bearing loads/friction.
    """
    print("\n" + "="*60)
    print("J4 TORQUE CALCULATION (Axial Rotation of Tube)")
    print("="*60)

    # Moment of inertia for Link 3 (tube) rotating about its axis
    I_tube = calculate_tube_inertia(LINK_3_MASS, LINK_3_RADIUS)

    # Motors are mounted on opposite sides of the tube; treat each as a point mass
    I_motor_each = calculate_point_mass_inertia(MOTOR_MASS, MOTOR_RADIAL_OFFSET)
    I_motors_total = 2 * I_motor_each

    # Gripper inertia (include parallel-axis term)
    I_gripper_center = calculate_disc_inertia(MASS_GRIPPER, GRIPPER_RADIUS)
    I_gripper = I_gripper_center + calculate_point_mass_inertia(MASS_GRIPPER, GRIPPER_RADIAL_OFFSET)

    # Payload inertia is computed for reporting only and is NOT included in dynamics
    I_payload_center = calculate_solid_sphere_inertia(MASS_PAYLOAD, PAYLOAD_RADIUS)
    I_payload = I_payload_center + calculate_point_mass_inertia(MASS_PAYLOAD, PAYLOAD_RADIAL_OFFSET)

    # Total inertia used for dynamics excludes payload
    total_inertia = I_tube + I_motors_total + I_gripper

    # Dynamic torque (only inertia * angular acceleration)
    torque_dynamic = total_inertia * ALPHA_J4

    # Estimate bearing friction torque using a simple model: T = mu * F_radial * r_eff
    radial_load = MASS_GRIPPER * g + MASS_PAYLOAD * g
    torque_bearing_friction = BEARING_FRICTION_COEFF * radial_load * BEARING_EFFECTIVE_RADIUS

    # Motor internal friction (two motors)
    torque_motor_friction_total = 2 * MOTOR_FRICTION_PER_MOTOR

    # Total static/friction torque is bearing + motor friction
    torque_static = torque_bearing_friction + torque_motor_friction_total

    # Holding torque due to gripper gravity offset: static moment that J4 must resist
    if GRIPPER_HOLDING_TORQUE_OVERRIDE is None:
        gripper_holding_torque = MASS_GRIPPER * g * GRIPPER_RADIAL_OFFSET
    else:
        gripper_holding_torque = float(GRIPPER_HOLDING_TORQUE_OVERRIDE)

    # Add holding torque to static/friction term (gravity-induced steady torque)
    torque_static += gripper_holding_torque

    # Final required torque includes dynamic + static (with holding)
    torque_required = torque_dynamic + torque_static
    torque_with_safety = torque_required * SAFETY_FACTOR

    # Per-component dynamic torque contributions (inertia * alpha) for reporting
    torque_tube_dynamic = I_tube * ALPHA_J4
    torque_motors_dynamic = I_motors_total * ALPHA_J4
    torque_gripper_dynamic = I_gripper * ALPHA_J4
    torque_payload_dynamic = I_payload * ALPHA_J4  # reported but not used in dynamics

    print(f"Rotational inertia breakdown:")
    print(f"  Link 3 (tube): {I_tube:.6f} kg·m² -> dynamic: {torque_tube_dynamic:.3f} N·m")
    print(f"  Each motor (at {MOTOR_RADIAL_OFFSET:.3f}m): {I_motor_each:.6f} kg·m² -> each dynamic: {(I_motor_each*ALPHA_J4):.3f} N·m")
    print(f"  Motors total: {I_motors_total:.6f} kg·m² -> dynamic: {torque_motors_dynamic:.3f} N·m")
    print(f"  Gripper center inertia (disc): {I_gripper_center:.6f} kg·m²")
    print(f"  Gripper (parallel-axis at {GRIPPER_RADIAL_OFFSET:.3f}m): {I_gripper:.6f} kg·m² -> dynamic: {torque_gripper_dynamic:.3f} N·m")
    print(f"  Payload center inertia (sphere) [NOT included in J4 dynamics]: {I_payload_center:.6f} kg·m²")
    print(f"  Payload (parallel-axis at {PAYLOAD_RADIAL_OFFSET:.3f}m) [NOT included]: {I_payload:.6f} kg·m² -> dynamic(if included): {torque_payload_dynamic:.3f} N·m")
    print(f"Total rotational inertia (used for dynamics): {total_inertia:.6f} kg·m² -> dynamic total: {torque_dynamic:.3f} N·m")
    print(f"Estimated bearing friction torque: {torque_bearing_friction:.3f} N·m")
    print(f"Motor friction total: {torque_motor_friction_total:.3f} N·m")
    print(f"Gripper holding torque (static due to gravity offset): {gripper_holding_torque:.3f} N·m")
    print(f"Static torque (bearing + motor friction + holding): {torque_static:.3f} N·m")
    print(f"Required torque (dynamic + static): {torque_required:.3f} N·m")
    print(f"With safety factor ({SAFETY_FACTOR}x): {torque_with_safety:.3f} N·m")

    return torque_with_safety

# ===========================
# ADDITIONAL USEFUL CALCULATIONS
# ===========================

def calculate_power_requirements(torque, rpm):
    """Calculate power requirement in Watts"""
    omega = rpm * 2 * math.pi / 60  # Convert RPM to rad/s
    power = torque * omega
    return power

def calculate_gear_ratio(motor_torque, required_torque):
    """Calculate required gear ratio"""
    return required_torque / motor_torque

# ===========================
# MAIN EXECUTION
# ===========================

if __name__ == "__main__":
    print("="*60)
    print("ROBOTIC ARM TORQUE CALCULATOR")
    print("Last Three Joints (J4, J5, J6)")
    print("="*60)
    # RUNTIME WARNING: The robot should NOT rely on J4 or J5 to lift the payload
    # with the gripper. In actual use, J4 will NOT be used to lift the payload
    # carried by the gripper — payload lift loads must be carried by the
    # primary upstream lifting joint or an appropriate mechanism. This script
    # computes torques for analysis only.
    print("WARNING: In actual use J4 will NOT lift the payload with the gripper; use upstream lifting joints.")
    
    print("\nCONFIGURATION:")
    print(f"Link 3 length: {LINK_3_LENGTH} m, mass: {LINK_3_MASS} kg")
    print(f"Gripper mass: {MASS_GRIPPER} kg")
    print(f"Payload mass: {MASS_PAYLOAD} kg")
    print(f"Safety factor: {SAFETY_FACTOR}")
    
    # Calculate torques
    torque_j6 = calculate_j6_torque()
    torque_j5 = calculate_j5_torque()
    torque_j4 = calculate_j4_torque()
    
    # Summary
    print("\n" + "="*60)
    print("SUMMARY - REQUIRED TORQUES (with safety factor)")
    print("="*60)
    print(f"J6 (Axial rotation, gripper): {torque_j6:.3f} N·m")
    print(f"J5 (Lifting joint): {torque_j5:.3f} N·m")
    print(f"J4 (Axial rotation, tube): {torque_j4:.3f} N·m")
    
    # Power requirements (example at 60 RPM)
    print("\n" + "="*60)
    print("POWER REQUIREMENTS @ 60 RPM")
    print("="*60)
    rpm = 60
    print(f"J6: {calculate_power_requirements(torque_j6, rpm):.2f} W")
    print(f"J5: {calculate_power_requirements(torque_j5, rpm):.2f} W")
    print(f"J4: {calculate_power_requirements(torque_j4, rpm):.2f} W")
    
    print("\n" + "="*60)
    print("Note: Adjust parameters at top of file for your specific design")
    print("="*60)
