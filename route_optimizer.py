#!/usr/bin/env python3
"""
Route Optimizer Module
Provides advanced route planning and optimization utilities for Mission 1.

This module can be imported by the main simulation or used standalone
to analyze different routing strategies.
"""

import numpy as np
from typing import List, Tuple, Dict
import itertools


class RouteOptimizer:
    """
    Advanced route optimization for multi-waypoint navigation.
    Implements various algorithms including nearest neighbor and optimal TSP-like solutions.
    """
    
    def __init__(self, waypoints: Dict[str, np.ndarray], start_position: np.ndarray):
        """
        Initialize route optimizer.
        
        Args:
            waypoints: Dictionary mapping waypoint names to positions [x, y]
            start_position: Starting position [x, y]
        """
        self.waypoints = waypoints
        self.start_position = start_position
        
    def euclidean_distance(self, pos1: np.ndarray, pos2: np.ndarray) -> float:
        """Calculate Euclidean distance between two positions."""
        return np.linalg.norm(pos2 - pos1)
    
    def nearest_neighbor(self) -> List[str]:
        """
        Greedy nearest neighbor algorithm.
        Always visits the closest unvisited waypoint next.
        
        Returns:
            List of waypoint names in visiting order
        """
        current_pos = self.start_position
        unvisited = set(self.waypoints.keys())
        route = []
        
        while unvisited:
            # Find nearest unvisited waypoint
            nearest = min(unvisited, 
                         key=lambda wp: self.euclidean_distance(current_pos, self.waypoints[wp]))
            route.append(nearest)
            current_pos = self.waypoints[nearest]
            unvisited.remove(nearest)
        
        return route
    
    def brute_force_optimal(self) -> Tuple[List[str], float]:
        """
        Find optimal route by testing all permutations (brute force).
        Only practical for small number of waypoints (< 10).
        
        Returns:
            Tuple of (optimal_route, total_distance)
        """
        waypoint_names = list(self.waypoints.keys())
        all_routes = itertools.permutations(waypoint_names)
        
        best_route = None
        best_distance = float('inf')
        
        for route in all_routes:
            distance = self._calculate_route_distance(list(route))
            if distance < best_distance:
                best_distance = distance
                best_route = route
        
        return list(best_route), best_distance
    
    def _calculate_route_distance(self, route: List[str]) -> float:
        """Calculate total distance for a given route."""
        total_distance = 0.0
        current_pos = self.start_position
        
        for waypoint_name in route:
            waypoint_pos = self.waypoints[waypoint_name]
            total_distance += self.euclidean_distance(current_pos, waypoint_pos)
            current_pos = waypoint_pos
        
        return total_distance
    
    def left_to_right_sweep(self) -> List[str]:
        """
        Sort waypoints by x-coordinate (left to right sweep).
        Useful for field layouts where waypoints are arranged horizontally.
        
        Returns:
            List of waypoint names sorted by x-coordinate
        """
        sorted_waypoints = sorted(self.waypoints.items(), 
                                 key=lambda item: item[1][0])
        return [name for name, _ in sorted_waypoints]
    
    def bottom_to_top_sweep(self) -> List[str]:
        """
        Sort waypoints by y-coordinate (bottom to top sweep).
        Useful for field layouts where waypoints are arranged vertically.
        
        Returns:
            List of waypoint names sorted by y-coordinate
        """
        sorted_waypoints = sorted(self.waypoints.items(), 
                                 key=lambda item: item[1][1])
        return [name for name, _ in sorted_waypoints]
    
    def zigzag_pattern(self) -> List[str]:
        """
        Create zigzag pattern (sweep left-right alternating with vertical movement).
        Good for systematic coverage of grid-like layouts.
        
        Returns:
            List of waypoint names in zigzag order
        """
        # Sort by y first, then x
        sorted_waypoints = sorted(self.waypoints.items(), 
                                 key=lambda item: (item[1][1], item[1][0]))
        
        # Group by similar y-coordinates (tolerance of 0.1m)
        rows = []
        current_row = []
        prev_y = None
        
        for name, pos in sorted_waypoints:
            if prev_y is None or abs(pos[1] - prev_y) < 0.1:
                current_row.append((name, pos))
            else:
                rows.append(current_row)
                current_row = [(name, pos)]
            prev_y = pos[1]
        
        if current_row:
            rows.append(current_row)
        
        # Alternate direction for each row
        route = []
        for i, row in enumerate(rows):
            row_sorted = sorted(row, key=lambda item: item[1][0], reverse=(i % 2 == 1))
            route.extend([name for name, _ in row_sorted])
        
        return route
    
    def compare_all_strategies(self) -> Dict[str, Dict]:
        """
        Compare all routing strategies and return comprehensive results.
        
        Returns:
            Dictionary mapping strategy names to their results
        """
        strategies = {
            'Nearest Neighbor': self.nearest_neighbor(),
            'Left-to-Right Sweep': self.left_to_right_sweep(),
            'Bottom-to-Top Sweep': self.bottom_to_top_sweep(),
            'Zigzag Pattern': self.zigzag_pattern()
        }
        
        # Add brute force optimal if feasible (< 8 waypoints)
        if len(self.waypoints) <= 7:
            optimal_route, optimal_distance = self.brute_force_optimal()
            strategies['Optimal (Brute Force)'] = optimal_route
        
        results = {}
        for strategy_name, route in strategies.items():
            distance = self._calculate_route_distance(route)
            results[strategy_name] = {
                'route': route,
                'distance': distance,
                'route_string': ' → '.join(route)
            }
        
        return results
    
    def print_comparison_table(self):
        """Print formatted comparison table of all strategies."""
        results = self.compare_all_strategies()
        
        print("\n" + "="*70)
        print("ROUTE STRATEGY COMPARISON")
        print("="*70)
        print(f"{'Strategy':<25} {'Distance (m)':<15} {'Route Order':<30}")
        print("-" * 70)
        
        # Sort by distance
        sorted_results = sorted(results.items(), key=lambda x: x[1]['distance'])
        
        for strategy_name, data in sorted_results:
            print(f"{strategy_name:<25} {data['distance']:>8.3f}       {data['route_string']:<30}")
        
        print("="*70 + "\n")
        
        # Identify best strategy
        best_strategy = sorted_results[0]
        print(f"BEST STRATEGY: {best_strategy[0]}")
        print(f"Distance: {best_strategy[1]['distance']:.3f} m")
        print(f"Route: {best_strategy[1]['route_string']}\n")


def main():
    """Standalone demonstration of route optimizer."""
    print("\nRoute Optimizer - Standalone Demo")
    print("="*70)
    
    # Example: ITU Competition crop plots
    crop_plots = {
        'orange': np.array([0.25, 0.995]),
        'gray': np.array([0.585, 0.995]),
        'green': np.array([0.92, 0.995])
    }
    
    start_position = np.array([0.585, 0.15])
    
    optimizer = RouteOptimizer(crop_plots, start_position)
    optimizer.print_comparison_table()
    
    # Demonstrate individual strategies
    print("\nINDIVIDUAL STRATEGY RESULTS:")
    print("-" * 70)
    
    nn_route = optimizer.nearest_neighbor()
    print(f"Nearest Neighbor: {' → '.join(nn_route)}")
    print(f"  Distance: {optimizer._calculate_route_distance(nn_route):.3f} m\n")
    
    lr_route = optimizer.left_to_right_sweep()
    print(f"Left-to-Right Sweep: {' → '.join(lr_route)}")
    print(f"  Distance: {optimizer._calculate_route_distance(lr_route):.3f} m\n")
    
    optimal_route, optimal_dist = optimizer.brute_force_optimal()
    print(f"Optimal (Brute Force): {' → '.join(optimal_route)}")
    print(f"  Distance: {optimal_dist:.3f} m\n")


if __name__ == "__main__":
    main()
