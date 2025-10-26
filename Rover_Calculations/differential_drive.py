import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass
class RoverGeometry:
    """Physical parameters of the rocker-bogie rover."""
    mass: float  # kg
    chassis_length: float  # m (wheelbase)
    chassis_width: float  # m (track width)
    wheel_radius: float  # m
    rocker_length: float  # m
    cg_offset_x: float = 0.0  # m (positive = forward)
    cg_offset_y: float = 0.0  # m (positive = left)
    cg_height: float = 0.15  # m (above ground)


@dataclass
class TerrainParams:
    """Terrain and traction parameters."""
    mu: float  # coefficient of friction
    rolling_resistance: float = 0.08  # dimensionless
    terrain_slope_deg: float = 0.0  # degrees
    

@dataclass
class MotionCommand:
    """Desired motion of the rover."""
    velocity: float  # m/s (forward positive)
    yaw_rate: float  # rad/s (CCW positive)


class RockerRoverDynamics:
    """
    Physics-based torque calculator for 4-wheel rocker-bogie rover.
    
    Coordinate system:
        - Origin at chassis geometric center
        - X-axis: forward (positive)
        - Y-axis: left (positive)
        - Z-axis: up (positive)
        - Yaw: CCW positive (right-hand rule)
    
    Wheel naming convention:
        FL: Front Left,  FR: Front Right
        RL: Rear Left,   RR: Rear Right
    """
    
    def __init__(self, geometry: RoverGeometry, terrain: TerrainParams):
        self.geo = geometry
        self.terrain = terrain
        self.g = 9.81  # m/s^2
        
        # Compute wheel positions relative to chassis center
        L, W = geometry.chassis_length, geometry.chassis_width
        self.wheel_pos = {
            "FL": np.array([L/2, W/2]),
            "FR": np.array([L/2, -W/2]),
            "RL": np.array([-L/2, W/2]),
            "RR": np.array([-L/2, -W/2])
        }
        
    def compute_normal_loads(self) -> Dict[str, float]:
        """
        Calculate normal force distribution across wheels.
        
        Accounts for:
        - Terrain slope
        - CG position (longitudinal and lateral offset)
        - Static weight distribution
        
        Returns:
            Dict of normal forces [N] per wheel
        """
        slope_rad = np.deg2rad(self.terrain.terrain_slope_deg)
        W_total = self.geo.mass * self.g * np.cos(slope_rad)
        
        L = self.geo.chassis_length
        W = self.geo.chassis_width
        
        # Longitudinal weight transfer due to CG offset and slope
        long_bias = self.geo.cg_offset_x / L if L > 0 else 0
        slope_bias = np.tan(slope_rad) * self.geo.cg_height / L if L > 0 else 0
        front_fraction = 0.5 - long_bias - slope_bias
        rear_fraction = 1.0 - front_fraction
        
        # Lateral weight transfer due to CG offset
        lat_bias = self.geo.cg_offset_y / W if W > 0 else 0
        left_fraction = 0.5 + lat_bias
        right_fraction = 1.0 - left_fraction
        
        # Distribute to individual wheels
        return {
            "FL": W_total * front_fraction * left_fraction,
            "FR": W_total * front_fraction * right_fraction,
            "RL": W_total * rear_fraction * left_fraction,
            "RR": W_total * rear_fraction * right_fraction
        }
    
    def compute_icc_geometry(self, cmd: MotionCommand) -> Tuple[float, Dict[str, float]]:
        """
        Calculate Instantaneous Center of Curvature (ICC) and wheel radii.
        
        Args:
            cmd: Motion command
            
        Returns:
            Tuple of (R_center, wheel_distances) where:
            - R_center: radius from vehicle center to ICC [m]
            - wheel_distances: distance from each wheel to ICC [m]
        """
        if np.abs(cmd.yaw_rate) < 1e-9:
            # Straight line motion
            R_center = np.inf
            r_wheels = {k: np.inf for k in self.wheel_pos.keys()}
        else:
            # Turning motion: ICC is lateral to vehicle center
            R_center = cmd.velocity / cmd.yaw_rate
            
            r_wheels = {}
            for wheel, pos in self.wheel_pos.items():
                x, y = pos[0], pos[1]
                # ICC at (0, R_center) in vehicle frame
                dx = -x
                dy = R_center - y
                r_wheels[wheel] = np.hypot(dx, dy)
        
        return R_center, r_wheels
    
    def compute_wheel_velocities(self, cmd: MotionCommand, 
                                r_wheels: Dict[str, float]) -> Dict[str, Tuple[float, float]]:
        """
        Calculate required linear and angular velocity for each wheel.
        
        Args:
            cmd: Motion command
            r_wheels: Distance from each wheel to ICC
            
        Returns:
            Dict mapping wheel name to (linear_vel [m/s], angular_vel [rad/s])
        """
        velocities = {}
        
        for wheel, r_i in r_wheels.items():
            if np.isinf(r_i):
                # Straight motion
                v_linear = cmd.velocity
            else:
                # Curved motion: v = ω × r
                v_linear = cmd.yaw_rate * r_i
            
            omega = v_linear / self.geo.wheel_radius
            velocities[wheel] = (v_linear, omega)
        
        return velocities
    
    def compute_longitudinal_forces(self, normal_loads: Dict[str, float],
                                   cmd: MotionCommand, R_center: float) -> Dict[str, float]:
        """
        Calculate longitudinal force requirements for each wheel.
        
        Includes:
        - Rolling resistance
        - Yaw moment generation
        - Traction limits
        
        Returns:
            Dict of longitudinal forces [N] per wheel
        """
        # Base rolling resistance (proportional to normal load)
        F_base = {k: self.terrain.rolling_resistance * N 
                 for k, N in normal_loads.items()}
        
        # Yaw moment requirement
        if np.abs(cmd.yaw_rate) < 1e-9:
            # No turning
            return F_base
        
        # Required yaw moment: M_z ≈ m·v·ω·(L/2)
        # More accurate: M_z = I_z·α + m·v²/R·(xcg) for steady turn
        L = self.geo.chassis_length
        W = self.geo.chassis_width
        
        # Simplified yaw moment (quasi-static approximation)
        M_z = self.geo.mass * np.abs(cmd.velocity * cmd.yaw_rate) * (L / 2)
        
        # Convert to force differential across track width
        # ΔF·(W/2) = M_z  =>  ΔF = 2·M_z/W
        if W > 1e-6:
            delta_F = 2 * M_z / W
        else:
            delta_F = 0
        
        # Distribute force differential based on longitudinal position
        # Wheels farther from CG contribute more to yaw moment
        x_positions = {k: np.abs(self.wheel_pos[k][0]) for k in self.wheel_pos}
        x_sum_per_side = x_positions["FL"] + x_positions["RL"]
        
        if x_sum_per_side > 1e-6:
            x_weight = {
                "FL": x_positions["FL"] / x_sum_per_side,
                "RL": x_positions["RL"] / x_sum_per_side,
                "FR": x_positions["FR"] / x_sum_per_side,
                "RR": x_positions["RR"] / x_sum_per_side
            }
        else:
            x_weight = {k: 0.5 for k in self.wheel_pos}
        
        # Apply differential (positive yaw_rate = right turn = right wheels faster)
        # Right side adds force, left side reduces
        turn_sign = np.sign(cmd.yaw_rate)
        F_yaw = {}
        
        for wheel in self.wheel_pos.keys():
            if wheel in ("FL", "RL"):  # Left side
                F_yaw[wheel] = -turn_sign * delta_F * x_weight[wheel] / 2
            else:  # Right side
                F_yaw[wheel] = turn_sign * delta_F * x_weight[wheel] / 2
        
        # Combine base and yaw forces
        F_total = {k: F_base[k] + F_yaw[k] for k in self.wheel_pos}
        
        # Add centripetal force effect (increases outer wheel load)
        if not np.isinf(R_center):
            F_centripetal = self.geo.mass * cmd.velocity**2 / np.abs(R_center)
            
            # Distribute to outer wheels (70%) and reduce inner wheels (30%)
            outer_frac = 0.35
            inner_frac = 0.15
            
            if turn_sign > 0:  # Right turn
                F_total["FR"] += F_centripetal * outer_frac
                F_total["RR"] += F_centripetal * outer_frac
                F_total["FL"] -= F_centripetal * inner_frac
                F_total["RL"] -= F_centripetal * inner_frac
            else:  # Left turn
                F_total["FL"] += F_centripetal * outer_frac
                F_total["RL"] += F_centripetal * outer_frac
                F_total["FR"] -= F_centripetal * inner_frac
                F_total["RR"] -= F_centripetal * inner_frac
        
        return F_total
    
    def compute_torques(self, cmd: MotionCommand) -> Dict:
        """
        Main calculation: compute required torques for all wheels.
        
        Returns:
            Dict containing:
            - T_req: Required torques [N·m]
            - F_req: Required forces [N]
            - F_max: Maximum traction forces [N]
            - N: Normal loads [N]
            - v_i: Linear velocities [m/s]
            - omega_i: Angular velocities [rad/s]
            - R_center: Turning radius [m]
        """
        # Step 1: Normal loads
        N = self.compute_normal_loads()
        
        # Step 2: ICC geometry
        R_center, r_wheels = self.compute_icc_geometry(cmd)
        
        # Step 3: Wheel velocities
        velocities = self.compute_wheel_velocities(cmd, r_wheels)
        v_i = {k: v[0] for k, v in velocities.items()}
        omega_i = {k: v[1] for k, v in velocities.items()}
        
        # Step 4: Traction limits
        F_max = {k: self.terrain.mu * N[k] for k in N}
        
        # Step 5: Required longitudinal forces
        F_req = self.compute_longitudinal_forces(N, cmd, R_center)
        
        # Step 6: Convert to torques
        T_req = {k: F_req[k] * self.geo.wheel_radius for k in F_req}
        
        # Check traction limits and warn if exceeded
        traction_margin = {k: F_max[k] - abs(F_req[k]) for k in F_req}
        
        return {
            "T_req": T_req,
            "F_req": F_req,
            "F_max": F_max,
            "N": N,
            "v_i": v_i,
            "omega_i": omega_i,
            "R_center": R_center,
            "traction_margin": traction_margin
        }


def plot_torque_analysis(geometry: RoverGeometry, terrain: TerrainParams,
                        velocity: float = 0.5, R_min: float = 0.5, 
                        R_max: float = 5.0, steps: int = 50):
    """
    Generate plots and analysis of torque vs turning radius.
    
    Args:
        geometry: Rover geometry parameters
        terrain: Terrain parameters
        velocity: Forward velocity [m/s]
        R_min, R_max: Min/max turning radius [m]
        steps: Number of points to plot
    """
    dynamics = RockerRoverDynamics(geometry, terrain)
    R_values = np.linspace(R_min, R_max, steps)
    
    # Storage for results
    results = {wheel: {"T": [], "omega": [], "F": [], "margin": []} 
              for wheel in ["FL", "FR", "RL", "RR"]}
    
    print("\n" + "="*110)
    print(f"Torque Analysis: v = {velocity:.2f} m/s, mass = {geometry.mass:.1f} kg, μ = {terrain.mu:.2f}")
    print("="*110)
    print(f"{'R (m)':>8} | {'T_FL':>8} {'T_FR':>8} {'T_RL':>8} {'T_RR':>8} | "
          f"{'ω_FL':>7} {'ω_FR':>7} {'ω_RL':>7} {'ω_RR':>7} | {'Min Margin':>11}")
    print("-"*110)
    
    for R in R_values:
        yaw_rate = velocity / R
        cmd = MotionCommand(velocity=velocity, yaw_rate=yaw_rate)
        res = dynamics.compute_torques(cmd)
        
        min_margin = min(res["traction_margin"].values())
        
        print(f"{R:8.3f} | ", end="")
        print(f"{res['T_req']['FL']:8.4f} {res['T_req']['FR']:8.4f} "
              f"{res['T_req']['RL']:8.4f} {res['T_req']['RR']:8.4f} | ", end="")
        print(f"{res['omega_i']['FL']:7.2f} {res['omega_i']['FR']:7.2f} "
              f"{res['omega_i']['RL']:7.2f} {res['omega_i']['RR']:7.2f} | ", end="")
        print(f"{min_margin:11.2f} N")
        
        for wheel in results:
            results[wheel]["T"].append(res["T_req"][wheel])
            results[wheel]["omega"].append(res["omega_i"][wheel])
            results[wheel]["F"].append(res["F_req"][wheel])
            results[wheel]["margin"].append(res["traction_margin"][wheel])
    
    # Create plots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Plot 1: Torques
    ax = axes[0, 0]
    for wheel, style in [("FL", "r-o"), ("FR", "b-o"), ("RL", "g-s"), ("RR", "m-s")]:
        ax.plot(R_values, results[wheel]["T"], style, label=f'T_{wheel}', markersize=4)
    ax.set_xlabel("Turning Radius (m)")
    ax.set_ylabel("Torque (N·m)")
    ax.set_title("Motor Torque Requirements")
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Plot 2: Angular velocities
    ax = axes[0, 1]
    for wheel, style in [("FL", "r-o"), ("FR", "b-o"), ("RL", "g-s"), ("RR", "m-s")]:
        ax.plot(R_values, results[wheel]["omega"], style, label=f'ω_{wheel}', markersize=4)
    ax.set_xlabel("Turning Radius (m)")
    ax.set_ylabel("Angular Velocity (rad/s)")
    ax.set_title("Motor Speed Requirements")
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Plot 3: Forces
    ax = axes[1, 0]
    for wheel, style in [("FL", "r-o"), ("FR", "b-o"), ("RL", "g-s"), ("RR", "m-s")]:
        ax.plot(R_values, results[wheel]["F"], style, label=f'F_{wheel}', markersize=4)
    ax.set_xlabel("Turning Radius (m)")
    ax.set_ylabel("Longitudinal Force (N)")
    ax.set_title("Wheel Force Distribution")
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Plot 4: Traction margins
    ax = axes[1, 1]
    for wheel, style in [("FL", "r-o"), ("FR", "b-o"), ("RL", "g-s"), ("RR", "m-s")]:
        ax.plot(R_values, results[wheel]["margin"], style, label=f'{wheel}', markersize=4)
    ax.axhline(y=0, color='k', linestyle='--', linewidth=1, label='Slip threshold')
    ax.set_xlabel("Turning Radius (m)")
    ax.set_ylabel("Traction Margin (N)")
    ax.set_title("Available Traction (F_max - F_req)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    plt.tight_layout()
    plt.show()


# Example usage
if __name__ == "__main__":
    # Define rover parameters
    geometry = RoverGeometry(
        mass=55.0,              # kg
        chassis_length=0.85,    # m
        chassis_width=0.45,     # m
        wheel_radius=0.075,     # m
        rocker_length=0.9,      # m
        cg_offset_x=0.0,       # m (0 = centered)
        cg_height=0.15          # m
    )
    
    terrain = TerrainParams(
        mu=0.6,                    # dry soil/sand
        rolling_resistance=0.08,   # typical for soft terrain
        terrain_slope_deg=0.0
    )
    
    # Run analysis
    plot_torque_analysis(
        geometry=geometry,
        terrain=terrain,
        velocity=0.5,      # m/s
        R_min=0.5,         # m
        R_max=5.0,         # m
        steps=50
    )
