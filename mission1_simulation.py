#!/usr/bin/env python3
"""
ITU Robotics for Good Youth Challenge 2025-2026
Mission 1: Cultivation and Irrigation Simulator

This simulation helps optimize the robot's navigation strategy for Mission 1,
including route planning, timing analysis, and feasibility testing.

Author: GitHub Copilot
Date: December 26, 2025
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend to avoid display issues
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.animation import FuncAnimation
import yaml
import csv
import os
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from enum import Enum
import itertools
from datetime import datetime


class MissionPhase(Enum):
    """Enumeration of mission phases for tracking and visualization."""
    LOADING = "Loading Seeds"
    TRAVELING_TO_PLOT = "Traveling to Crop Plot"
    PLANTING = "Planting Seeds"
    TRAVELING_TO_GATE = "Traveling to Irrigation Gate"
    IRRIGATING = "Activating Irrigation"
    RETURNING = "Returning to Start"
    COMPLETED = "Mission Completed"


@dataclass
class Waypoint:
    """Represents a waypoint in the mission path."""
    name: str
    position: Tuple[float, float]
    stop_duration: float = 0.0  # Time to spend at this waypoint (seconds)
    phase: MissionPhase = MissionPhase.TRAVELING_TO_PLOT


@dataclass
class RobotState:
    """Current state of the robot during simulation."""
    position: np.ndarray  # [x, y]
    velocity: float
    time: float
    phase: MissionPhase
    distance_traveled: float


class Mission1Simulator:
    """
    Simulates Mission 1: Cultivation and Irrigation for the ITU Robotics Competition.
    
    Features:
    - Waypoint-based navigation with acceleration/deceleration profiles
    - Route optimization comparing different visiting orders
    - Real-time timing analysis against 120-second limit
    - Visualization with trajectory tracking
    - Data export for analysis
    """
    
    def __init__(self, config_file: str = "robotics_competition.yaml"):
        """
        Initialize the simulator with configuration from YAML file.
        
        Args:
            config_file: Path to the YAML configuration file
        """
        self.config = self._load_config(config_file)
        self._setup_parameters()
        self._define_waypoints()
        
        # Simulation state
        self.trajectory = []
        self.time_log = []
        self.phase_log = []
        self.velocity_log = []
        
    def _load_config(self, config_file: str) -> Dict:
        """Load configuration from YAML file."""
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)
    
    def _setup_parameters(self):
        """Extract and setup parameters from configuration."""
        # World parameters
        self.field_width = self.config['world']['width']
        self.field_height = self.config['world']['height']
        self.dt = self.config['world']['sample_time']
        
        # Robot parameters
        robot_cfg = self.config['robot']
        self.robot_radius = robot_cfg['radius']
        self.max_velocity = robot_cfg['velocity']['max_linear']
        self.acceleration = robot_cfg['acceleration']['linear']
        self.start_position = np.array(robot_cfg['position'][:2])
        
        # Mission parameters
        mission_cfg = self.config['mission']
        self.time_limit = mission_cfg['time_limit']
        self.seed_placement_time = mission_cfg['seed_placement_time']
        self.irrigation_time = mission_cfg['irrigation_activation_time']
        
        # Extract goal positions
        self.goals = {goal['name']: np.array(goal['position']) 
                     for goal in self.config['goals']}
        
    def _define_waypoints(self):
        """Define waypoints for crop plot cells and irrigation gate."""
        # New field layout: single 2×3 crop plot with 6 individual cells
        self.crop_plots = {
            'cell_1_1': self.goals['crop_cell_1_1'],  # Bottom-left
            'cell_1_2': self.goals['crop_cell_1_2'],  # Bottom-right
            'cell_2_1': self.goals['crop_cell_2_1'],  # Middle-left
            'cell_2_2': self.goals['crop_cell_2_2'],  # Middle-right
            'cell_3_1': self.goals['crop_cell_3_1'],  # Top-left
            'cell_3_2': self.goals['crop_cell_3_2']   # Top-right
        }
        self.crop_plot_center = self.goals['crop_plot_center']
        self.irrigation_gate_pos = self.goals['irrigation_gate_entrance']
        
        # Colored plot sections for two-trip strategy
        self.colored_plots = {
            'orange': self.goals.get('orange_plot', self.goals['crop_cell_1_1']),  # Bottom section
            'gray': self.goals.get('gray_plot', self.goals['crop_cell_2_1']),      # Middle section
            'green': self.goals.get('green_plot', self.goals['crop_cell_3_1'])     # Top section
        }
        self.trip1_midpoint = self.goals.get('trip1_midpoint', np.array([0.175, 0.877]))
        
    def calculate_travel_time(self, start: np.ndarray, end: np.ndarray, 
                             current_velocity: float = 0.0) -> Tuple[float, float]:
        """
        Calculate travel time between two points with acceleration/deceleration profile.
        
        Uses trapezoidal velocity profile:
        1. Accelerate from current velocity to max velocity
        2. Cruise at max velocity
        3. Decelerate to zero
        
        Args:
            start: Starting position [x, y]
            end: Ending position [x, y]
            current_velocity: Initial velocity (m/s)
            
        Returns:
            Tuple of (travel_time, distance)
        """
        distance = np.linalg.norm(end - start)
        
        # Time to accelerate from current velocity to max velocity
        t_accel = (self.max_velocity - current_velocity) / self.acceleration
        d_accel = current_velocity * t_accel + 0.5 * self.acceleration * t_accel**2
        
        # Time to decelerate from max velocity to zero
        t_decel = self.max_velocity / self.acceleration
        d_decel = 0.5 * self.acceleration * t_decel**2
        
        # Check if we reach max velocity
        if d_accel + d_decel <= distance:
            # Trapezoidal profile: accelerate, cruise, decelerate
            d_cruise = distance - d_accel - d_decel
            t_cruise = d_cruise / self.max_velocity
            total_time = t_accel + t_cruise + t_decel
        else:
            # Triangular profile: only accelerate and decelerate
            # Solve for peak velocity: v_peak^2 = v0^2 + 2*a*d/2 + v_peak^2 = 2*a*d/2
            # Simplified: v_peak = sqrt(v0^2 + a*d)
            v_peak = np.sqrt(current_velocity**2 + self.acceleration * distance)
            t_accel = (v_peak - current_velocity) / self.acceleration
            t_decel = v_peak / self.acceleration
            total_time = t_accel + t_decel
            
        return total_time, distance
    
    def calculate_route_metrics(self, route_order: List[str], 
                                include_irrigation: bool = True,
                                include_return: bool = False) -> Dict:
        """
        Calculate total time and distance for a given route order.
        
        Args:
            route_order: List of crop plot names in visiting order (e.g., ['orange', 'gray', 'green'])
            include_irrigation: Whether to include irrigation gate visit
            include_return: Whether to include return to start zone
            
        Returns:
            Dictionary with route metrics
        """
        total_time = 0.0
        total_distance = 0.0
        current_pos = self.start_position.copy()
        current_velocity = 0.0
        
        segments = []
        
        # Visit each crop plot in order
        for plot_name in route_order:
            # Check both crop_plots and colored_plots for the position
            if plot_name in self.crop_plots:
                plot_pos = self.crop_plots[plot_name]
            elif hasattr(self, 'colored_plots') and plot_name in self.colored_plots:
                plot_pos = self.colored_plots[plot_name]
            else:
                raise KeyError(f"Unknown plot name: {plot_name}")
            
            travel_time, distance = self.calculate_travel_time(current_pos, plot_pos, current_velocity)
            
            total_time += travel_time + self.seed_placement_time
            total_distance += distance
            
            segments.append({
                'from': 'start' if current_pos is self.start_position else 'previous_plot',
                'to': plot_name,
                'distance': distance,
                'travel_time': travel_time,
                'stop_time': self.seed_placement_time
            })
            
            current_pos = plot_pos
            current_velocity = 0.0  # Stop at each plot
        
        # Visit irrigation gate
        if include_irrigation:
            travel_time, distance = self.calculate_travel_time(current_pos, 
                                                               self.irrigation_gate_pos, 
                                                               current_velocity)
            total_time += travel_time + self.irrigation_time
            total_distance += distance
            
            segments.append({
                'from': 'last_plot',
                'to': 'irrigation_gate',
                'distance': distance,
                'travel_time': travel_time,
                'stop_time': self.irrigation_time
            })
            
            current_pos = self.irrigation_gate_pos
            current_velocity = 0.0
        
        # Return to start zone
        if include_return:
            travel_time, distance = self.calculate_travel_time(current_pos, 
                                                               self.start_position, 
                                                               current_velocity)
            total_time += travel_time
            total_distance += distance
            
            segments.append({
                'from': 'irrigation_gate' if include_irrigation else 'last_plot',
                'to': 'start_zone',
                'distance': distance,
                'travel_time': travel_time,
                'stop_time': 0.0
            })
        
        return {
            'route_order': route_order,
            'total_time': total_time,
            'total_distance': total_distance,
            'segments': segments,
            'time_remaining': self.time_limit - total_time,
            'feasible': total_time <= self.time_limit
        }
    
    def optimize_route(self) -> Dict:
        """
        Compare all possible route orders and find the optimal one.
        Note: With 6 crop cells, we'll test a subset of strategic routes
        instead of all 720 permutations for practicality.
        
        Returns:
            Dictionary with comparison of all routes and optimal route
        """
        plot_names = list(self.crop_plots.keys())
        
        # For 6 cells (720 permutations), test strategic patterns instead
        # Pattern 1: Row-by-row (bottom to top)
        # Pattern 2: Column-by-column (left to right)
        # Pattern 3: Zigzag pattern
        strategic_routes = [
            ['cell_1_1', 'cell_1_2', 'cell_2_1', 'cell_2_2', 'cell_3_1', 'cell_3_2'],  # Row-by-row
            ['cell_1_1', 'cell_2_1', 'cell_3_1', 'cell_3_2', 'cell_2_2', 'cell_1_2'],  # Column-by-column
            ['cell_1_1', 'cell_1_2', 'cell_2_2', 'cell_2_1', 'cell_3_1', 'cell_3_2'],  # Zigzag
            ['cell_3_1', 'cell_3_2', 'cell_2_1', 'cell_2_2', 'cell_1_1', 'cell_1_2'],  # Reverse row-by-row
        ]
        
        print("\n" + "="*70)
        print("ROUTE OPTIMIZATION ANALYSIS")
        print("="*70)
        print(f"Testing {len(strategic_routes)} strategic route patterns...\n")
        
        results = []
        for route_order in strategic_routes:
            metrics = self.calculate_route_metrics(route_order, 
                                                   include_irrigation=True, 
                                                   include_return=False)
            results.append(metrics)
        
        # Sort by total time
        results.sort(key=lambda x: x['total_time'])
        
        # Print comparison table
        print(f"{'Rank':<6} {'Route Order':<25} {'Distance (m)':<15} {'Time (s)':<12} {'Remaining (s)':<15} {'Feasible':<10}")
        print("-" * 95)
        
        for i, result in enumerate(results, 1):
            route_str = ' → '.join(result['route_order'])
            feasible_str = "✓ YES" if result['feasible'] else "✗ NO"
            print(f"{i:<6} {route_str:<25} {result['total_distance']:>8.3f}      "
                  f"{result['total_time']:>8.2f}    {result['time_remaining']:>10.2f}      {feasible_str:<10}")
        
        optimal = results[0]
        print("\n" + "="*70)
        print(f"OPTIMAL ROUTE: {' → '.join(optimal['route_order'])}")
        print(f"Total Distance: {optimal['total_distance']:.3f} m")
        print(f"Total Time: {optimal['total_time']:.2f} s")
        print(f"Time Remaining: {optimal['time_remaining']:.2f} s")
        print("="*70 + "\n")
        
        return {
            'all_routes': results,
            'optimal_route': optimal,
            'worst_route': results[-1]
        }
    
    def simulate_trajectory(self, route_order: List[str], 
                           include_irrigation: bool = True,
                           include_return: bool = False,
                           save_trajectory: bool = True) -> RobotState:
        """
        Simulate the complete robot trajectory for a given route.
        
        Args:
            route_order: List of crop plot names in visiting order
            include_irrigation: Whether to include irrigation gate visit
            include_return: Whether to include return to start zone
            save_trajectory: Whether to save trajectory data
            
        Returns:
            Final robot state
        """
        print("\n" + "="*70)
        print("TRAJECTORY SIMULATION")
        print("="*70)
        print(f"Route: {' → '.join(route_order)}")
        print(f"Include Irrigation: {include_irrigation}")
        print(f"Include Return to Start: {include_return}")
        print("="*70 + "\n")
        
        # Reset trajectory data
        self.trajectory = []
        self.time_log = []
        self.phase_log = []
        self.velocity_log = []
        
        current_pos = self.start_position.copy()
        current_time = 0.0
        total_distance = 0.0
        
        # Build waypoint sequence
        waypoints = []
        
        # Add crop plots
        for plot_name in route_order:
            waypoints.append(Waypoint(
                name=f"{plot_name}_plot",
                position=tuple(self.crop_plots[plot_name]),
                stop_duration=self.seed_placement_time,
                phase=MissionPhase.PLANTING
            ))
        
        # Add irrigation gate
        if include_irrigation:
            waypoints.append(Waypoint(
                name="irrigation_gate",
                position=tuple(self.irrigation_gate_pos),
                stop_duration=self.irrigation_time,
                phase=MissionPhase.IRRIGATING
            ))
        
        # Add return to start
        if include_return:
            waypoints.append(Waypoint(
                name="start_zone",
                position=tuple(self.start_position),
                stop_duration=0.0,
                phase=MissionPhase.RETURNING
            ))
        
        # Simulate each segment
        for i, waypoint in enumerate(waypoints):
            print(f"Phase {i+1}: Traveling to {waypoint.name}")
            
            target_pos = np.array(waypoint.position)
            distance = np.linalg.norm(target_pos - current_pos)
            
            # Generate trajectory points with acceleration profile
            points, times, velocities = self._generate_segment_trajectory(
                current_pos, target_pos, current_time
            )
            
            # Append to trajectory
            self.trajectory.extend(points)
            self.time_log.extend(times)
            self.velocity_log.extend(velocities)
            self.phase_log.extend([MissionPhase.TRAVELING_TO_PLOT] * len(points))
            
            travel_time = times[-1] - current_time
            current_time = times[-1]
            total_distance += distance
            
            print(f"  Distance: {distance:.3f} m | Travel Time: {travel_time:.2f} s | "
                  f"Cumulative Time: {current_time:.2f} s")
            
            # Stop at waypoint
            if waypoint.stop_duration > 0:
                print(f"  Stopping for {waypoint.stop_duration:.1f} s ({waypoint.phase.value})")
                
                # Add stationary points during stop
                num_stop_points = int(waypoint.stop_duration / self.dt)
                for _ in range(num_stop_points):
                    self.trajectory.append(target_pos.copy())
                    current_time += self.dt
                    self.time_log.append(current_time)
                    self.velocity_log.append(0.0)
                    self.phase_log.append(waypoint.phase)
                
                print(f"  Cumulative Time: {current_time:.2f} s")
            
            current_pos = target_pos
            print()
        
        # Final state
        final_state = RobotState(
            position=current_pos,
            velocity=0.0,
            time=current_time,
            phase=MissionPhase.COMPLETED,
            distance_traveled=total_distance
        )
        
        print("="*70)
        print("SIMULATION SUMMARY")
        print("="*70)
        print(f"Total Distance Traveled: {total_distance:.3f} m")
        print(f"Total Time: {current_time:.2f} s")
        print(f"Time Limit: {self.time_limit} s")
        print(f"Time Remaining: {self.time_limit - current_time:.2f} s")
        print(f"Mission Feasibility: {'✓ FEASIBLE' if current_time <= self.time_limit else '✗ EXCEEDS TIME LIMIT'}")
        print(f"Average Velocity: {total_distance / current_time:.3f} m/s")
        print(f"Trajectory Points: {len(self.trajectory)}")
        print("="*70 + "\n")
        
        # Save trajectory to CSV
        if save_trajectory:
            self._save_trajectory_csv(route_order)
        
        return final_state
    
    def _generate_segment_trajectory(self, start: np.ndarray, end: np.ndarray, 
                                    start_time: float) -> Tuple[List, List, List]:
        """
        Generate trajectory points for a segment with acceleration/deceleration profile.
        
        Args:
            start: Starting position
            end: Ending position
            start_time: Starting time
            
        Returns:
            Tuple of (positions, times, velocities)
        """
        distance = np.linalg.norm(end - start)
        direction = (end - start) / distance if distance > 0 else np.array([0, 0])
        
        # Calculate time phases
        t_accel = self.max_velocity / self.acceleration
        d_accel = 0.5 * self.acceleration * t_accel**2
        d_decel = d_accel
        
        positions = []
        times = []
        velocities = []
        
        current_pos = start.copy()
        current_time = start_time
        
        if d_accel + d_decel <= distance:
            # Trapezoidal profile
            d_cruise = distance - d_accel - d_decel
            t_cruise = d_cruise / self.max_velocity
            t_decel_actual = t_accel  # Same as acceleration time
            
            # Acceleration phase
            t = 0
            while t < t_accel:
                v = self.acceleration * t
                d = 0.5 * self.acceleration * t**2
                pos = start + direction * d
                positions.append(pos)
                times.append(current_time + t)
                velocities.append(v)
                t += self.dt
            
            # Cruise phase
            t = 0
            while t < t_cruise:
                d = d_accel + self.max_velocity * t
                pos = start + direction * d
                positions.append(pos)
                times.append(current_time + t_accel + t)
                velocities.append(self.max_velocity)
                t += self.dt
            
            # Deceleration phase
            t = 0
            while t < t_decel_actual:
                v = self.max_velocity - self.acceleration * t
                d = d_accel + d_cruise + self.max_velocity * t - 0.5 * self.acceleration * t**2
                pos = start + direction * d
                positions.append(pos)
                times.append(current_time + t_accel + t_cruise + t)
                velocities.append(v)
                t += self.dt
        else:
            # Triangular profile
            v_peak = np.sqrt(self.acceleration * distance)
            t_accel_tri = v_peak / self.acceleration
            
            # Acceleration phase
            t = 0
            while t < t_accel_tri:
                v = self.acceleration * t
                d = 0.5 * self.acceleration * t**2
                pos = start + direction * d
                positions.append(pos)
                times.append(current_time + t)
                velocities.append(v)
                t += self.dt
            
            # Deceleration phase
            t = 0
            while t < t_accel_tri:
                v = v_peak - self.acceleration * t
                d = distance / 2 + v_peak * t - 0.5 * self.acceleration * t**2
                pos = start + direction * d
                positions.append(pos)
                times.append(current_time + t_accel_tri + t)
                velocities.append(v)
                t += self.dt
        
        # Ensure we reach the end
        if len(positions) == 0 or np.linalg.norm(positions[-1] - end) > 0.01:
            positions.append(end)
            times.append(times[-1] + self.dt if times else current_time)
            velocities.append(0.0)
        
        return positions, times, velocities
    
    def _save_trajectory_csv(self, route_order: List[str]):
        """Save trajectory data to CSV file."""
        route_name = '_'.join(route_order)
        filename = f"results/trajectory_{len(route_order)}-trip_{route_name}.csv"
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time (s)', 'X (m)', 'Y (m)', 'Velocity (m/s)', 'Phase'])
            
            for i in range(len(self.trajectory)):
                writer.writerow([
                    f"{self.time_log[i]:.3f}",
                    f"{self.trajectory[i][0]:.4f}",
                    f"{self.trajectory[i][1]:.4f}",
                    f"{self.velocity_log[i]:.4f}",
                    self.phase_log[i].value
                ])
        
        print(f"Trajectory data saved to: {filename}")
    
    def visualize_trajectory(self, save_plot: bool = True, animate: bool = True):
        """
        Visualize the robot trajectory and timing profile with optional animation.
        
        Args:
            save_plot: Whether to save the final plot
            animate: Whether to show real-time animation of robot movement
        """
        if animate and self.trajectory:
            self._animate_robot_movement(save_animation=save_plot)
        else:
            self._plot_static_trajectory(save_plot=save_plot)
    
    def _animate_robot_movement(self, save_animation: bool = False):
        """Create animated visualization of robot movement."""
        print("\n" + "="*70)
        print("GENERATING ANIMATION - Robot Movement Visualization")
        print("="*70)
        print("Creating animated GIF of robot movement...")
        print("(Animation will be saved to file - no interactive window on macOS)")
        print("="*70 + "\n")
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))
        fig.suptitle('Mission 1: Real-Time Robot Simulation', 
                    fontsize=16, fontweight='bold')
        
        # Setup field visualization (ax1)
        ax1.set_xlim(0, self.field_width)
        ax1.set_ylim(0, self.field_height)
        ax1.set_aspect('equal')
        ax1.set_xlabel('X Position (m)', fontsize=12)
        ax1.set_ylabel('Y Position (m)', fontsize=12)
        ax1.set_title('Robot Navigation Path', fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        
        # Draw field elements
        self._draw_field_elements(ax1)
        
        # Initialize animated elements
        robot_circle = Circle((0, 0), self.robot_radius, 
                            color='red', alpha=0.7, zorder=10)
        ax1.add_patch(robot_circle)
        
        path_line, = ax1.plot([], [], 'r-', linewidth=2, alpha=0.5, label='Path Traveled')
        current_pos_marker, = ax1.plot([], [], 'ro', markersize=12, 
                                      markeredgecolor='darkred', markeredgewidth=2)
        
        # Text displays
        time_text = ax1.text(0.02, 0.98, '', transform=ax1.transAxes, 
                           fontsize=11, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        phase_text = ax1.text(0.02, 0.88, '', transform=ax1.transAxes,
                            fontsize=11, verticalalignment='top',
                            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        ax1.legend(loc='upper right', fontsize=10)
        
        # Setup velocity/distance plot (ax2)
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('Velocity (m/s)', fontsize=12, color='b')
        ax2.tick_params(axis='y', labelcolor='b')
        ax2.set_title('Velocity Profile', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.set_xlim(0, max(self.time_log) if self.time_log else 120)
        ax2.set_ylim(0, max(self.velocity_log) * 1.2 if self.velocity_log else 0.5)
        
        velocity_line, = ax2.plot([], [], 'b-', linewidth=2, label='Velocity')
        ax2.axhline(y=self.max_velocity, color='green', linestyle='--', 
                   linewidth=1.5, alpha=0.5, label='Max Velocity')
        ax2.axvline(x=self.time_limit, color='red', linestyle='--', 
                   linewidth=2, alpha=0.7, label='Time Limit')
        ax2.legend(loc='upper right', fontsize=10)
        
        # Animation state
        path_history_x = []
        path_history_y = []
        time_history = []
        velocity_history = []
        
        # Animation speed control (skip frames for faster playback)
        frame_skip = max(1, len(self.trajectory) // 500)  # Limit to ~500 frames
        
        def init():
            """Initialize animation."""
            robot_circle.center = (self.start_position[0], self.start_position[1])
            path_line.set_data([], [])
            current_pos_marker.set_data([], [])
            velocity_line.set_data([], [])
            time_text.set_text('')
            phase_text.set_text('')
            return robot_circle, path_line, current_pos_marker, velocity_line, time_text, phase_text
        
        def animate(frame):
            """Update animation frame."""
            idx = frame * frame_skip
            if idx >= len(self.trajectory):
                idx = len(self.trajectory) - 1
            
            # Update robot position
            pos = self.trajectory[idx]
            robot_circle.center = (pos[0], pos[1])
            current_pos_marker.set_data([pos[0]], [pos[1]])
            
            # Update path history
            path_history_x.append(pos[0])
            path_history_y.append(pos[1])
            path_line.set_data(path_history_x, path_history_y)
            
            # Update velocity plot
            if idx < len(self.time_log):
                time_history.append(self.time_log[idx])
                velocity_history.append(self.velocity_log[idx])
                velocity_line.set_data(time_history, velocity_history)
            
            # Update text displays
            current_time = self.time_log[idx] if idx < len(self.time_log) else 0
            current_phase = self.phase_log[idx] if idx < len(self.phase_log) else MissionPhase.LOADING
            
            time_text.set_text(f'Time: {current_time:.1f}s / {self.time_limit}s\n'
                             f'Position: ({pos[0]:.3f}, {pos[1]:.3f}) m')
            phase_text.set_text(f'Phase: {current_phase.value}')
            
            return robot_circle, path_line, current_pos_marker, velocity_line, time_text, phase_text
        
        # Calculate total frames
        total_frames = (len(self.trajectory) + frame_skip - 1) // frame_skip
        
        # Create animation
        anim = FuncAnimation(fig, animate, init_func=init,
                           frames=total_frames, interval=20,  # 20ms between frames
                           blit=True, repeat=False)
        
        plt.tight_layout()
        
        if save_animation:
            try:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"results/mission1_animation_{timestamp}.gif"
                print(f"Saving animation to: {filename}")
                print("Please wait, rendering animation frames...")
                anim.save(filename, writer='pillow', fps=30, dpi=100)
                print(f"\n✓ SUCCESS: Animation saved to {filename}")
                print(f"  Open this file to watch the robot simulation!")
                print(f"  File size: ~{os.path.getsize(filename) / 1024 / 1024:.1f} MB")
            except Exception as e:
                print(f"✗ ERROR: Could not save animation: {e}")
        
        plt.close(fig)  # Close figure without displaying
        print("\n✓ Animation generation complete!")
    
    def _draw_field_elements(self, ax):
        """Draw static field elements on the axis."""
        # Draw boundaries
        ax.add_patch(Rectangle((0, 0), self.field_width, self.field_height,
                              linewidth=3, edgecolor='black', facecolor='white'))
        
        # Draw start zone (bottom-right)
        start_zone_x = self.field_width - 0.320
        start_zone = Rectangle((start_zone_x, 0.020), 0.300, 0.580,
                               linewidth=2, edgecolor='blue', facecolor='lightblue', alpha=0.3)
        ax.add_patch(start_zone)
        ax.text(start_zone_x + 0.150, 0.300, 'START\nZONE', 
               ha='center', va='center', fontsize=9, fontweight='bold', color='darkblue')
        
        # Draw crop plot area
        ax.add_patch(Rectangle((0.025, 0.743), 0.300, 0.400,
                              linewidth=3, edgecolor='darkgreen', facecolor='lightgreen', alpha=0.2))
        ax.text(0.175, 0.720, 'CROP PLOT', 
               ha='center', va='top', fontsize=9, fontweight='bold', color='darkgreen')
        
        # Draw crop cells
        for cell_pos in self.crop_plots.values():
            ax.add_patch(Rectangle((cell_pos[0] - 0.04, cell_pos[1] - 0.055), 
                                  0.08, 0.11, linewidth=1, edgecolor='green', 
                                  facecolor='lightgreen', alpha=0.4))
            ax.plot(cell_pos[0], cell_pos[1], 'o', color='green', markersize=5)
        
        # Draw sorting zone
        ax.add_patch(Rectangle((0.020, 0.020), 0.275, 0.446,
                              linewidth=2, edgecolor='orange', facecolor='yellow', alpha=0.2))
        ax.text(0.1575, 0.480, 'SORTING', ha='center', va='top', 
               fontsize=8, fontweight='bold', color='darkorange')
        
        # Draw irrigation gate
        ax.plot(self.irrigation_gate_pos[0], self.irrigation_gate_pos[1], 's', 
               color='blue', markersize=10, markeredgecolor='black', markeredgewidth=1.5)
        ax.text(self.irrigation_gate_pos[0] + 0.08, self.irrigation_gate_pos[1], 
               'GATE', ha='left', va='center', fontsize=7, fontweight='bold')
    
    def _plot_static_trajectory(self, save_plot: bool = True):
        """
        Create visualization of the robot trajectory with field layout.
        
        Args:
            save_plot: Whether to save the plot to file
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
        
        # === Left plot: Field layout and trajectory ===
        ax1.set_xlim(-0.05, self.field_width + 0.05)
        ax1.set_ylim(-0.05, self.field_height + 0.05)
        ax1.set_aspect('equal')
        ax1.set_xlabel('X Position (m)', fontsize=12)
        ax1.set_ylabel('Y Position (m)', fontsize=12)
        ax1.set_title('Mission 1: Robot Trajectory on Competition Field', fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        
        # Draw field boundaries
        field_rect = Rectangle((0, 0), self.field_width, self.field_height,
                               linewidth=3, edgecolor='black', facecolor='lightgray', alpha=0.2)
        ax1.add_patch(field_rect)
        
        # Draw start zone (bottom-right corner: 320mm × 600mm)
        start_zone_x = self.field_width - 0.320
        start_zone = Rectangle((start_zone_x, 0), 0.320, 0.600,
                               linewidth=2, edgecolor='blue', facecolor='lightblue', alpha=0.3)
        ax1.add_patch(start_zone)
        ax1.text(start_zone_x + 0.160, 0.300, 'START\nZONE', 
                ha='center', va='center', fontsize=10, fontweight='bold', color='darkblue')
        
        # Draw crop plot area (top-left: 300mm × 400mm with 2×3 grid)
        crop_plot_outer = Rectangle((0.025, 0.743), 0.300, 0.400,
                                   linewidth=3, edgecolor='darkgreen', facecolor='lightgreen', alpha=0.2)
        ax1.add_patch(crop_plot_outer)
        ax1.text(0.175, 0.720, 'CROP PLOT\n(2×3 Grid)', 
                ha='center', va='top', fontsize=9, fontweight='bold', color='darkgreen')
        
        # Draw individual crop cells
        for i, (cell_name, cell_pos) in enumerate(self.crop_plots.items()):
            cell_rect = Rectangle((cell_pos[0] - 0.04, cell_pos[1] - 0.055), 
                                 0.08, 0.11,
                                 linewidth=1.5, edgecolor='green', 
                                 facecolor='lightgreen', alpha=0.5)
            ax1.add_patch(cell_rect)
            ax1.plot(cell_pos[0], cell_pos[1], 'o', color='green', 
                    markersize=6, markeredgecolor='darkgreen', markeredgewidth=1.5)
        
        # Draw sorting zone (bottom-left: 275mm × 446mm)
        sorting_zone = Rectangle((0.020, 0.020), 0.275, 0.446,
                                linewidth=2, edgecolor='orange', facecolor='yellow', alpha=0.2)
        ax1.add_patch(sorting_zone)
        ax1.text(0.1575, 0.480, 'SORTING\nZONE', 
                ha='center', va='top', fontsize=9, fontweight='bold', color='darkorange')
        
        # Draw irrigation corridor (left side, vertical)
        corridor_rect = Rectangle((0.020, 0.250), 0.155, 0.600,
                                 linewidth=2, edgecolor='cyan', facecolor='lightcyan', alpha=0.2)
        ax1.add_patch(corridor_rect)
        
        # Draw irrigation gate
        gate_rect = Rectangle((self.irrigation_gate_pos[0] - 0.075, 
                               self.irrigation_gate_pos[1] - 0.04),
                              0.150, 0.080,
                              linewidth=2, edgecolor='darkblue', facecolor='cyan', alpha=0.5)
        ax1.add_patch(gate_rect)
        ax1.plot(self.irrigation_gate_pos[0], self.irrigation_gate_pos[1], 's', 
                color='blue', markersize=10, markeredgecolor='black', markeredgewidth=2)
        ax1.text(self.irrigation_gate_pos[0] + 0.10, self.irrigation_gate_pos[1], 
                'IRRIGATION\nGATE', ha='left', va='center', fontsize=8, fontweight='bold')
        
        # Draw trajectory
        if self.trajectory:
            traj_array = np.array(self.trajectory)
            ax1.plot(traj_array[:, 0], traj_array[:, 1], 'r-', 
                    linewidth=2, label='Robot Path', alpha=0.7)
            
            # Mark start and end positions
            ax1.plot(traj_array[0, 0], traj_array[0, 1], 'go', 
                    markersize=15, label='Start', markeredgecolor='black', markeredgewidth=2)
            ax1.plot(traj_array[-1, 0], traj_array[-1, 1], 'r*', 
                    markersize=20, label='End', markeredgecolor='black', markeredgewidth=2)
            
            # Add direction arrows
            arrow_indices = np.linspace(0, len(traj_array) - 1, 15, dtype=int)
            for idx in arrow_indices[1:-1]:  # Skip start and end
                if idx + 1 < len(traj_array):
                    dx = traj_array[idx + 1, 0] - traj_array[idx, 0]
                    dy = traj_array[idx + 1, 1] - traj_array[idx, 1]
                    ax1.arrow(traj_array[idx, 0], traj_array[idx, 1], dx, dy,
                            head_width=0.03, head_length=0.02, fc='red', ec='red', alpha=0.6)
        
        ax1.legend(loc='upper right', fontsize=10)
        
        # === Right plot: Velocity and timing profile ===
        if self.time_log:
            ax2_twin = ax2.twinx()
            
            # Plot velocity
            ax2.plot(self.time_log, self.velocity_log, 'b-', linewidth=2, label='Velocity')
            ax2.set_xlabel('Time (s)', fontsize=12)
            ax2.set_ylabel('Velocity (m/s)', fontsize=12, color='b')
            ax2.tick_params(axis='y', labelcolor='b')
            ax2.grid(True, alpha=0.3)
            ax2.set_title('Velocity Profile and Distance Over Time', fontsize=14, fontweight='bold')
            
            # Plot cumulative distance
            distances = [0]
            for i in range(1, len(self.trajectory)):
                dist = distances[-1] + np.linalg.norm(self.trajectory[i] - self.trajectory[i-1])
                distances.append(dist)
            
            ax2_twin.plot(self.time_log, distances, 'g-', linewidth=2, label='Distance Traveled')
            ax2_twin.set_ylabel('Distance Traveled (m)', fontsize=12, color='g')
            ax2_twin.tick_params(axis='y', labelcolor='g')
            
            # Mark time limit
            ax2.axvline(x=self.time_limit, color='red', linestyle='--', linewidth=2, label='Time Limit (120s)')
            
            # Add phase transitions
            phase_changes = []
            current_phase = self.phase_log[0]
            for i, phase in enumerate(self.phase_log):
                if phase != current_phase:
                    phase_changes.append((self.time_log[i], phase))
                    current_phase = phase
            
            for time, phase in phase_changes:
                ax2.axvline(x=time, color='gray', linestyle=':', alpha=0.5)
            
            # Legend
            lines1, labels1 = ax2.get_legend_handles_labels()
            lines2, labels2 = ax2_twin.get_legend_handles_labels()
            ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize=10)
        
        plt.tight_layout()
        
        if save_plot:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"results/mission1_visualization_{timestamp}.png"
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Visualization saved to: {filename}")
        
        plt.show()
    
    def compare_strategies(self):
        """
        Compare different mission strategies (2-trip vs 3-trip).
        """
        print("\n" + "="*70)
        print("STRATEGY COMPARISON: 2-TRIP vs 3-TRIP")
        print("="*70)
        
        # Compare different strategies for visiting 6 crop cells
        # Strategy 1: Single trip visiting all 6 cells
        # Strategy 2: Two trips (3 cells each)
        # Strategy 3: With/without return to start
        
        strategies = {
            'Single trip (row-by-row)': {
                'route': ['cell_1_1', 'cell_1_2', 'cell_2_1', 'cell_2_2', 'cell_3_1', 'cell_3_2'],
                'include_return': False,
                'description': 'Visit all 6 cells in one trip, row-by-row pattern'
            },
            'Single trip (with return)': {
                'route': ['cell_1_1', 'cell_1_2', 'cell_2_1', 'cell_2_2', 'cell_3_1', 'cell_3_2'],
                'include_return': True,
                'description': 'Visit all 6 cells and return to start zone'
            },
            'Zigzag pattern': {
                'route': ['cell_1_1', 'cell_1_2', 'cell_2_2', 'cell_2_1', 'cell_3_1', 'cell_3_2'],
                'include_return': False,
                'description': 'Zigzag pattern to minimize turning'
            }
        }
        
        results = []
        for strategy_name, strategy_config in strategies.items():
            metrics = self.calculate_route_metrics(
                strategy_config['route'],
                include_irrigation=True,
                include_return=strategy_config.get('include_return', False)
            )
            metrics['strategy_name'] = strategy_name
            metrics['description'] = strategy_config['description']
            results.append(metrics)
        
        print(f"\n{'Strategy':<30} {'Distance (m)':<15} {'Time (s)':<12} {'Feasible':<10}")
        print("-" * 70)
        
        for result in results:
            feasible_str = "✓ YES" if result['feasible'] else "✗ NO"
            print(f"{result['strategy_name']:<30} {result['total_distance']:>8.3f}      "
                  f"{result['total_time']:>8.2f}    {feasible_str:<10}")
        
        print("="*70 + "\n")
        
        return results
    
    def parameter_sweep(self, velocities: List[float] = None):
        """
        Test different maximum velocities to analyze performance.
        
        Args:
            velocities: List of velocities to test (m/s)
        """
        if velocities is None:
            velocities = [0.3, 0.35, 0.4, 0.45, 0.5]
        
        print("\n" + "="*70)
        print("PARAMETER SWEEP: VELOCITY ANALYSIS")
        print("="*70)
        
        original_velocity = self.max_velocity
        optimal_route = ['cell_1_1', 'cell_1_2', 'cell_2_1', 'cell_2_2', 'cell_3_1', 'cell_3_2']  # Row-by-row pattern
        
        results = []
        for velocity in velocities:
            self.max_velocity = velocity
            metrics = self.calculate_route_metrics(optimal_route, 
                                                   include_irrigation=True,
                                                   include_return=False)
            metrics['velocity'] = velocity
            results.append(metrics)
        
        print(f"\n{'Velocity (m/s)':<15} {'Distance (m)':<15} {'Time (s)':<12} {'Time Margin (s)':<18} {'Feasible':<10}")
        print("-" * 75)
        
        for result in results:
            feasible_str = "✓ YES" if result['feasible'] else "✗ NO"
            print(f"{result['velocity']:>8.2f}       {result['total_distance']:>8.3f}      "
                  f"{result['total_time']:>8.2f}    {result['time_remaining']:>10.2f}        {feasible_str:<10}")
        
        print("="*70 + "\n")
        
        # Restore original velocity
        self.max_velocity = original_velocity
        
        return results

    def simulate_two_trip_strategy(self, 
                                   trip1_plots: List[str] = None,
                                   trip2_plots: List[str] = None,
                                   seed_reload_time: float = 2.5,
                                   include_irrigation: bool = True,
                                   save_trajectory: bool = True) -> Dict:
        """
        Simulate a two-trip seed planting strategy.
        
        Trip 1: Start Zone → first two crop sections (orange, gray) → Return to Start Zone
        Trip 2: Start Zone → remaining section (green) → Return to Start Zone
        Optional: Visit irrigation gate after both trips
        
        Args:
            trip1_plots: List of plot names for Trip 1 (default: ['orange', 'gray'])
            trip2_plots: List of plot names for Trip 2 (default: ['green'])
            seed_reload_time: Time to wait at Start Zone between trips (seconds)
            include_irrigation: Whether to include irrigation gate visit after Trip 2
            save_trajectory: Whether to save trajectory data
            
        Returns:
            Dictionary with detailed metrics for both trips
        """
        if trip1_plots is None:
            trip1_plots = ['orange', 'gray']
        if trip2_plots is None:
            trip2_plots = ['green']
        
        print("\n" + "="*70)
        print("TWO-TRIP SEED PLANTING STRATEGY SIMULATION")
        print("="*70)
        print(f"Trip 1 Targets: {' → '.join(trip1_plots)}")
        print(f"Trip 2 Targets: {' → '.join(trip2_plots)}")
        print(f"Seed Reload Time: {seed_reload_time} s")
        print(f"Include Irrigation: {include_irrigation}")
        print("="*70 + "\n")
        
        # Reset trajectory data
        self.trajectory = []
        self.time_log = []
        self.phase_log = []
        self.velocity_log = []
        
        current_pos = self.start_position.copy()
        current_time = 0.0
        total_distance = 0.0
        trip_metrics = []
        
        # =====================================================================
        # TRIP 1: Start Zone → Orange Plot → Gray Plot → Return to Start Zone
        # =====================================================================
        print("=" * 50)
        print("TRIP 1: First Two Crop Sections")
        print("=" * 50)
        
        trip1_start_time = current_time
        trip1_distance = 0.0
        
        # Visit each plot in Trip 1
        for plot_name in trip1_plots:
            if plot_name in self.colored_plots:
                plot_pos = self.colored_plots[plot_name]
            elif plot_name in self.crop_plots:
                plot_pos = self.crop_plots[plot_name]
            else:
                print(f"  Warning: Unknown plot '{plot_name}', skipping")
                continue
            
            print(f"\n  → Traveling to {plot_name} plot...")
            
            # Generate trajectory to plot
            points, times, velocities = self._generate_segment_trajectory(
                current_pos, plot_pos, current_time
            )
            
            self.trajectory.extend(points)
            self.time_log.extend(times)
            self.velocity_log.extend(velocities)
            self.phase_log.extend([MissionPhase.TRAVELING_TO_PLOT] * len(points))
            
            distance = np.linalg.norm(plot_pos - current_pos)
            travel_time = times[-1] - current_time
            current_time = times[-1]
            trip1_distance += distance
            total_distance += distance
            
            print(f"    Distance: {distance:.3f} m | Travel Time: {travel_time:.2f} s")
            
            # Planting seeds at plot
            print(f"    Planting seeds at {plot_name} plot ({self.seed_placement_time:.1f} s)...")
            num_stop_points = int(self.seed_placement_time / self.dt)
            for _ in range(num_stop_points):
                self.trajectory.append(plot_pos.copy())
                current_time += self.dt
                self.time_log.append(current_time)
                self.velocity_log.append(0.0)
                self.phase_log.append(MissionPhase.PLANTING)
            
            current_pos = plot_pos
            print(f"    Cumulative Time: {current_time:.2f} s")
        
        # Return to Start Zone after Trip 1
        print(f"\n  ← Returning to Start Zone...")
        points, times, velocities = self._generate_segment_trajectory(
            current_pos, self.start_position, current_time
        )
        
        self.trajectory.extend(points)
        self.time_log.extend(times)
        self.velocity_log.extend(velocities)
        self.phase_log.extend([MissionPhase.RETURNING] * len(points))
        
        return_distance = np.linalg.norm(self.start_position - current_pos)
        return_time = times[-1] - current_time
        current_time = times[-1]
        trip1_distance += return_distance
        total_distance += return_distance
        
        trip1_total_time = current_time - trip1_start_time
        
        print(f"    Return Distance: {return_distance:.3f} m | Return Time: {return_time:.2f} s")
        print(f"\n  ✓ TRIP 1 COMPLETE")
        print(f"    Trip 1 Distance: {trip1_distance:.3f} m")
        print(f"    Trip 1 Time: {trip1_total_time:.2f} s")
        print(f"    Cumulative Time: {current_time:.2f} s")
        
        trip_metrics.append({
            'trip': 1,
            'plots_visited': trip1_plots,
            'distance': trip1_distance,
            'time': trip1_total_time
        })
        
        # =====================================================================
        # SEED RELOAD AT START ZONE
        # =====================================================================
        print(f"\n  ⏳ Reloading seeds at Start Zone ({seed_reload_time:.1f} s)...")
        num_reload_points = int(seed_reload_time / self.dt)
        for _ in range(num_reload_points):
            self.trajectory.append(self.start_position.copy())
            current_time += self.dt
            self.time_log.append(current_time)
            self.velocity_log.append(0.0)
            self.phase_log.extend([MissionPhase.LOADING])
        
        current_pos = self.start_position.copy()
        print(f"    Cumulative Time: {current_time:.2f} s")
        
        # =====================================================================
        # TRIP 2: Start Zone → Green Plot → Return to Start Zone
        # =====================================================================
        print("\n" + "=" * 50)
        print("TRIP 2: Remaining Crop Section")
        print("=" * 50)
        
        trip2_start_time = current_time
        trip2_distance = 0.0
        
        # Visit each plot in Trip 2
        for plot_name in trip2_plots:
            if plot_name in self.colored_plots:
                plot_pos = self.colored_plots[plot_name]
            elif plot_name in self.crop_plots:
                plot_pos = self.crop_plots[plot_name]
            else:
                print(f"  Warning: Unknown plot '{plot_name}', skipping")
                continue
            
            print(f"\n  → Traveling to {plot_name} plot...")
            
            # Generate trajectory to plot
            points, times, velocities = self._generate_segment_trajectory(
                current_pos, plot_pos, current_time
            )
            
            self.trajectory.extend(points)
            self.time_log.extend(times)
            self.velocity_log.extend(velocities)
            self.phase_log.extend([MissionPhase.TRAVELING_TO_PLOT] * len(points))
            
            distance = np.linalg.norm(plot_pos - current_pos)
            travel_time = times[-1] - current_time
            current_time = times[-1]
            trip2_distance += distance
            total_distance += distance
            
            print(f"    Distance: {distance:.3f} m | Travel Time: {travel_time:.2f} s")
            
            # Planting seeds at plot
            print(f"    Planting seeds at {plot_name} plot ({self.seed_placement_time:.1f} s)...")
            num_stop_points = int(self.seed_placement_time / self.dt)
            for _ in range(num_stop_points):
                self.trajectory.append(plot_pos.copy())
                current_time += self.dt
                self.time_log.append(current_time)
                self.velocity_log.append(0.0)
                self.phase_log.append(MissionPhase.PLANTING)
            
            current_pos = plot_pos
            print(f"    Cumulative Time: {current_time:.2f} s")
        
        # Return to Start Zone after Trip 2
        print(f"\n  ← Returning to Start Zone...")
        points, times, velocities = self._generate_segment_trajectory(
            current_pos, self.start_position, current_time
        )
        
        self.trajectory.extend(points)
        self.time_log.extend(times)
        self.velocity_log.extend(velocities)
        self.phase_log.extend([MissionPhase.RETURNING] * len(points))
        
        return_distance = np.linalg.norm(self.start_position - current_pos)
        return_time = times[-1] - current_time
        current_time = times[-1]
        trip2_distance += return_distance
        total_distance += return_distance
        
        trip2_total_time = current_time - trip2_start_time
        
        print(f"    Return Distance: {return_distance:.3f} m | Return Time: {return_time:.2f} s")
        print(f"\n  ✓ TRIP 2 COMPLETE")
        print(f"    Trip 2 Distance: {trip2_distance:.3f} m")
        print(f"    Trip 2 Time: {trip2_total_time:.2f} s")
        print(f"    Cumulative Time: {current_time:.2f} s")
        
        trip_metrics.append({
            'trip': 2,
            'plots_visited': trip2_plots,
            'distance': trip2_distance,
            'time': trip2_total_time
        })
        
        current_pos = self.start_position.copy()
        
        # =====================================================================
        # OPTIONAL: IRRIGATION GATE VISIT
        # =====================================================================
        irrigation_time_spent = 0.0
        irrigation_distance = 0.0
        
        if include_irrigation:
            print("\n" + "=" * 50)
            print("IRRIGATION GATE VISIT (Optional)")
            print("=" * 50)
            
            print(f"\n  → Traveling to irrigation gate...")
            points, times, velocities = self._generate_segment_trajectory(
                current_pos, self.irrigation_gate_pos, current_time
            )
            
            self.trajectory.extend(points)
            self.time_log.extend(times)
            self.velocity_log.extend(velocities)
            self.phase_log.extend([MissionPhase.TRAVELING_TO_GATE] * len(points))
            
            distance = np.linalg.norm(self.irrigation_gate_pos - current_pos)
            travel_time = times[-1] - current_time
            current_time = times[-1]
            irrigation_distance += distance
            total_distance += distance
            
            print(f"    Distance: {distance:.3f} m | Travel Time: {travel_time:.2f} s")
            
            # Activate irrigation
            print(f"    Activating irrigation ({self.irrigation_time:.1f} s)...")
            num_stop_points = int(self.irrigation_time / self.dt)
            for _ in range(num_stop_points):
                self.trajectory.append(self.irrigation_gate_pos.copy())
                current_time += self.dt
                self.time_log.append(current_time)
                self.velocity_log.append(0.0)
                self.phase_log.append(MissionPhase.IRRIGATING)
            
            current_pos = self.irrigation_gate_pos
            irrigation_time_spent = travel_time + self.irrigation_time
            
            # Return to Start Zone (final position)
            print(f"\n  ← Returning to Start Zone (final position)...")
            points, times, velocities = self._generate_segment_trajectory(
                current_pos, self.start_position, current_time
            )
            
            self.trajectory.extend(points)
            self.time_log.extend(times)
            self.velocity_log.extend(velocities)
            self.phase_log.extend([MissionPhase.RETURNING] * len(points))
            
            return_distance = np.linalg.norm(self.start_position - current_pos)
            return_time = times[-1] - current_time
            current_time = times[-1]
            irrigation_distance += return_distance
            total_distance += return_distance
            irrigation_time_spent += return_time
            
            print(f"    Return Distance: {return_distance:.3f} m | Return Time: {return_time:.2f} s")
            print(f"\n  ✓ IRRIGATION COMPLETE")
            print(f"    Irrigation Phase Distance: {irrigation_distance:.3f} m")
            print(f"    Irrigation Phase Time: {irrigation_time_spent:.2f} s")
        
        # =====================================================================
        # FINAL SUMMARY
        # =====================================================================
        print("\n" + "="*70)
        print("TWO-TRIP STRATEGY SIMULATION SUMMARY")
        print("="*70)
        
        within_time_limit = current_time <= self.time_limit
        
        print(f"\n  TIMING BREAKDOWN:")
        print(f"  {'─' * 45}")
        print(f"  Trip 1 (orange + gray):     {trip_metrics[0]['time']:>8.2f} s")
        print(f"  Seed Reload:                {seed_reload_time:>8.2f} s")
        print(f"  Trip 2 (green):             {trip_metrics[1]['time']:>8.2f} s")
        if include_irrigation:
            print(f"  Irrigation Phase:           {irrigation_time_spent:>8.2f} s")
        print(f"  {'─' * 45}")
        print(f"  TOTAL TIME:                 {current_time:>8.2f} s")
        print(f"  Time Limit:                 {self.time_limit:>8.2f} s")
        print(f"  Time Remaining:             {self.time_limit - current_time:>8.2f} s")
        
        print(f"\n  DISTANCE BREAKDOWN:")
        print(f"  {'─' * 45}")
        print(f"  Trip 1:                     {trip_metrics[0]['distance']:>8.3f} m")
        print(f"  Trip 2:                     {trip_metrics[1]['distance']:>8.3f} m")
        if include_irrigation:
            print(f"  Irrigation Phase:           {irrigation_distance:>8.3f} m")
        print(f"  {'─' * 45}")
        print(f"  TOTAL DISTANCE:             {total_distance:>8.3f} m")
        
        print(f"\n  MISSION STATUS:")
        print(f"  {'─' * 45}")
        if within_time_limit:
            print(f"  ✓ FEASIBLE - Completes within 120-second limit!")
            print(f"  ✓ Time buffer available: {self.time_limit - current_time:.2f} s")
        else:
            print(f"  ✗ EXCEEDS TIME LIMIT by {current_time - self.time_limit:.2f} s")
            print(f"  ✗ Consider reducing seed placement time or increasing speed")
        
        print("="*70 + "\n")
        
        # Save trajectory to CSV with two-trip naming
        if save_trajectory:
            route_name = f"2trip_{'-'.join(trip1_plots)}_then_{'-'.join(trip2_plots)}"
            if include_irrigation:
                route_name += "_with_irrigation"
            filename = f"results/trajectory_{route_name}.csv"
            
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Time (s)', 'X (m)', 'Y (m)', 'Velocity (m/s)', 'Phase'])
                
                for i in range(len(self.trajectory)):
                    writer.writerow([
                        f"{self.time_log[i]:.3f}",
                        f"{self.trajectory[i][0]:.4f}",
                        f"{self.trajectory[i][1]:.4f}",
                        f"{self.velocity_log[i]:.4f}",
                        self.phase_log[i].value
                    ])
            
            print(f"Trajectory data saved to: {filename}")
        
        return {
            'total_time': current_time,
            'total_distance': total_distance,
            'time_remaining': self.time_limit - current_time,
            'feasible': within_time_limit,
            'seed_reload_time': seed_reload_time,
            'trips': trip_metrics,
            'include_irrigation': include_irrigation,
            'irrigation_time': irrigation_time_spent if include_irrigation else 0,
            'irrigation_distance': irrigation_distance if include_irrigation else 0
        }

    def compare_single_vs_two_trip(self, seed_reload_time: float = 2.5) -> Dict:
        """
        Compare single-trip vs two-trip strategies.
        
        Args:
            seed_reload_time: Time to reload seeds at Start Zone between trips
            
        Returns:
            Dictionary with comparison results
        """
        print("\n" + "="*70)
        print("STRATEGY COMPARISON: SINGLE-TRIP vs TWO-TRIP")
        print("="*70)
        
        # Single-trip strategy (visit all 3 colored plots in one go)
        single_trip_route = ['orange', 'gray', 'green']
        single_trip_metrics = self.calculate_route_metrics(
            single_trip_route, 
            include_irrigation=True,
            include_return=True
        )
        
        # Two-trip strategy calculation
        # Trip 1: orange → gray → return
        # Trip 2: green → return
        # Optional: irrigation
        
        trip1_distance = 0.0
        trip1_time = 0.0
        
        # Trip 1: Start → orange
        d1 = np.linalg.norm(self.colored_plots['orange'] - self.start_position)
        t1, _ = self.calculate_travel_time(self.start_position, self.colored_plots['orange'])
        trip1_distance += d1
        trip1_time += t1 + self.seed_placement_time
        
        # Trip 1: orange → gray
        d2 = np.linalg.norm(self.colored_plots['gray'] - self.colored_plots['orange'])
        t2, _ = self.calculate_travel_time(self.colored_plots['orange'], self.colored_plots['gray'])
        trip1_distance += d2
        trip1_time += t2 + self.seed_placement_time
        
        # Trip 1: gray → Start
        d3 = np.linalg.norm(self.start_position - self.colored_plots['gray'])
        t3, _ = self.calculate_travel_time(self.colored_plots['gray'], self.start_position)
        trip1_distance += d3
        trip1_time += t3
        
        # Seed reload
        reload_time = seed_reload_time
        
        # Trip 2: Start → green → Start
        trip2_distance = 0.0
        trip2_time = 0.0
        
        d4 = np.linalg.norm(self.colored_plots['green'] - self.start_position)
        t4, _ = self.calculate_travel_time(self.start_position, self.colored_plots['green'])
        trip2_distance += d4
        trip2_time += t4 + self.seed_placement_time
        
        d5 = np.linalg.norm(self.start_position - self.colored_plots['green'])
        t5, _ = self.calculate_travel_time(self.colored_plots['green'], self.start_position)
        trip2_distance += d5
        trip2_time += t5
        
        # Irrigation (from start zone)
        irrigation_distance = 0.0
        irrigation_time = 0.0
        
        d6 = np.linalg.norm(self.irrigation_gate_pos - self.start_position)
        t6, _ = self.calculate_travel_time(self.start_position, self.irrigation_gate_pos)
        irrigation_distance += d6
        irrigation_time += t6 + self.irrigation_time
        
        d7 = np.linalg.norm(self.start_position - self.irrigation_gate_pos)
        t7, _ = self.calculate_travel_time(self.irrigation_gate_pos, self.start_position)
        irrigation_distance += d7
        irrigation_time += t7
        
        two_trip_total_time = trip1_time + reload_time + trip2_time + irrigation_time
        two_trip_total_distance = trip1_distance + trip2_distance + irrigation_distance
        
        print(f"\n{'Strategy':<35} {'Distance (m)':<15} {'Time (s)':<12} {'Remaining (s)':<15} {'Feasible':<10}")
        print("-" * 90)
        
        single_feasible = "✓ YES" if single_trip_metrics['feasible'] else "✗ NO"
        two_trip_feasible = "✓ YES" if two_trip_total_time <= self.time_limit else "✗ NO"
        
        print(f"{'Single-Trip (all 3 plots)':<35} {single_trip_metrics['total_distance']:>8.3f}      "
              f"{single_trip_metrics['total_time']:>8.2f}    {single_trip_metrics['time_remaining']:>10.2f}      {single_feasible:<10}")
        print(f"{'Two-Trip (2+1 with reload)':<35} {two_trip_total_distance:>8.3f}      "
              f"{two_trip_total_time:>8.2f}    {self.time_limit - two_trip_total_time:>10.2f}      {two_trip_feasible:<10}")
        
        time_diff = two_trip_total_time - single_trip_metrics['total_time']
        print(f"\n  Two-Trip adds {time_diff:.2f} s overhead (reload + extra return trips)")
        print(f"  Two-Trip adds {two_trip_total_distance - single_trip_metrics['total_distance']:.3f} m extra distance")
        
        print("="*70 + "\n")
        
        return {
            'single_trip': single_trip_metrics,
            'two_trip': {
                'total_time': two_trip_total_time,
                'total_distance': two_trip_total_distance,
                'time_remaining': self.time_limit - two_trip_total_time,
                'feasible': two_trip_total_time <= self.time_limit,
                'trip1_time': trip1_time,
                'trip2_time': trip2_time,
                'reload_time': reload_time,
                'irrigation_time': irrigation_time
            },
            'time_difference': time_diff
        }


def main():
    """Main simulation entry point."""
    print("\n")
    print("╔" + "="*68 + "╗")
    print("║" + " "*68 + "║")
    print("║" + "  ITU Robotics for Good Youth Challenge 2025-2026".center(68) + "║")
    print("║" + "  Mission 1: Cultivation and Irrigation Simulator".center(68) + "║")
    print("║" + "  TWO-TRIP SEED PLANTING STRATEGY".center(68) + "║")
    print("║" + " "*68 + "║")
    print("╚" + "="*68 + "╝")
    print()
    
    # Initialize simulator
    simulator = Mission1Simulator("robotics_competition.yaml")
    
    # =========================================================================
    # 1. COMPARE SINGLE-TRIP vs TWO-TRIP STRATEGIES
    # =========================================================================
    comparison_results = simulator.compare_single_vs_two_trip(seed_reload_time=2.5)
    
    # =========================================================================
    # 2. SIMULATE TWO-TRIP STRATEGY (Primary Mission Strategy)
    # =========================================================================
    # Trip 1: Start Zone → Orange Plot → Gray Plot → Return to Start Zone
    # Trip 2: Start Zone → Green Plot → Return to Start Zone
    # Optional: Irrigation gate visit after both trips
    
    two_trip_results = simulator.simulate_two_trip_strategy(
        trip1_plots=['orange', 'gray'],  # First two crop sections
        trip2_plots=['green'],           # Remaining crop section
        seed_reload_time=2.5,            # 2.5 seconds reload time at Start Zone
        include_irrigation=True,         # Visit irrigation gate after Trip 2
        save_trajectory=True
    )
    
    # =========================================================================
    # 3. VISUALIZE TWO-TRIP TRAJECTORY
    # =========================================================================
    print("\n" + "="*70)
    print("VISUALIZATION")
    print("="*70)
    print("Generating animated visualization of two-trip robot movement...")
    print("• Animation shows real-time robot path on the competition field")
    print("• Shows Trip 1, return to Start Zone, reload, Trip 2, and irrigation")
    print("• Velocity profile displayed with acceleration/deceleration")
    print("• Saved as GIF file for playback and analysis")
    print("="*70 + "\n")
    
    simulator.visualize_trajectory(save_plot=True, animate=True)
    
    # =========================================================================
    # 4. FINAL SUMMARY FOR TWO-TRIP STRATEGY
    # =========================================================================
    print("\n" + "="*70)
    print("TWO-TRIP MISSION STRATEGY - FINAL SUMMARY")
    print("="*70)
    print("\nSTRATEGY OVERVIEW:")
    print("  Trip 1: Start Zone → Orange Plot → Gray Plot → Start Zone")
    print("  [Reload]: Wait at Start Zone for seed reload (~2.5 seconds)")
    print("  Trip 2: Start Zone → Green Plot → Start Zone")
    print("  [Optional]: Start Zone → Irrigation Gate → Start Zone")
    
    print("\nKEY FINDINGS:")
    print(f"  • Total mission time: {two_trip_results['total_time']:.2f} seconds")
    print(f"  • Time remaining: {two_trip_results['time_remaining']:.2f} seconds")
    print(f"  • Total distance: {two_trip_results['total_distance']:.3f} meters")
    print(f"  • Mission feasibility: {'✓ FEASIBLE within 120s limit' if two_trip_results['feasible'] else '✗ EXCEEDS time limit'}")
    
    print("\nTRIP BREAKDOWN:")
    for trip in two_trip_results['trips']:
        plots_str = ', '.join(trip['plots_visited'])
        print(f"  • Trip {trip['trip']}: {plots_str}")
        print(f"    Distance: {trip['distance']:.3f} m | Time: {trip['time']:.2f} s")
    print(f"  • Seed Reload Time: {two_trip_results['seed_reload_time']:.1f} s")
    if two_trip_results['include_irrigation']:
        print(f"  • Irrigation Phase: {two_trip_results['irrigation_distance']:.3f} m | {two_trip_results['irrigation_time']:.2f} s")
    
    print("\nCOMPARISON TO SINGLE-TRIP:")
    time_diff = comparison_results['time_difference']
    if time_diff > 0:
        print(f"  • Two-trip takes {time_diff:.2f} s longer than single-trip")
    else:
        print(f"  • Two-trip is {-time_diff:.2f} s faster than single-trip")
    print(f"  • Both strategies are {'feasible' if two_trip_results['feasible'] and comparison_results['single_trip']['feasible'] else 'NOT both feasible'}")
    
    print("\nRECOMMENDATIONS:")
    print("  1. Use two-trip strategy for realistic seed capacity constraints")
    print("  2. Optimize path within each trip for minimal travel time")
    print(f"  3. Target velocity: {simulator.max_velocity} m/s for smooth motion")
    print("  4. Practice quick seed reload at Start Zone (target: 2-3 seconds)")
    print("  5. Monitor time during mission - abort irrigation if behind schedule")
    
    print("\nOUTPUT FILES:")
    print("  • Trajectory CSV: results/trajectory_2trip_*.csv")
    print("  • Animation GIF: results/mission1_animation_*.gif")
    print("="*70 + "\n")
    
    print("✓ Two-trip strategy simulation complete! Review the saved files for detailed results.")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
