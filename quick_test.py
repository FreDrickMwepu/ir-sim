#!/usr/bin/env python3
"""
Quick Scenario Tester
Rapidly test different mission scenarios without editing the main simulation.

Usage:
    python3 quick_test.py --velocity 0.35 --route orange,gray,green
    python3 quick_test.py --scenario conservative
    python3 quick_test.py --compare-all
"""

import argparse
import sys
from mission1_simulation import Mission1Simulator


def test_velocity_range(simulator, min_vel=0.25, max_vel=0.5, steps=6):
    """Test a range of velocities."""
    print("\n" + "="*70)
    print(f"VELOCITY RANGE TEST: {min_vel} to {max_vel} m/s")
    print("="*70)
    
    velocities = [min_vel + i * (max_vel - min_vel) / (steps - 1) 
                  for i in range(steps)]
    
    original_velocity = simulator.max_velocity
    optimal_route = ['orange', 'gray', 'green']
    
    print(f"\n{'Velocity':<12} {'Time':<10} {'Margin':<12} {'Feasible':<10}")
    print("-" * 50)
    
    for velocity in velocities:
        simulator.max_velocity = velocity
        metrics = simulator.calculate_route_metrics(optimal_route)
        feasible = "✓" if metrics['feasible'] else "✗"
        print(f"{velocity:>6.2f} m/s   {metrics['total_time']:>6.2f}s   "
              f"{metrics['time_remaining']:>7.2f}s    {feasible}")
    
    simulator.max_velocity = original_velocity
    print()


def test_acceleration_range(simulator, min_acc=0.3, max_acc=1.2, steps=5):
    """Test a range of accelerations."""
    print("\n" + "="*70)
    print(f"ACCELERATION RANGE TEST: {min_acc} to {max_acc} m/s²")
    print("="*70)
    
    accelerations = [min_acc + i * (max_acc - min_acc) / (steps - 1) 
                     for i in range(steps)]
    
    original_accel = simulator.acceleration
    optimal_route = ['orange', 'gray', 'green']
    
    print(f"\n{'Acceleration':<15} {'Time':<10} {'Margin':<12} {'Feasible':<10}")
    print("-" * 55)
    
    for accel in accelerations:
        simulator.acceleration = accel
        metrics = simulator.calculate_route_metrics(optimal_route)
        feasible = "✓" if metrics['feasible'] else "✗"
        print(f"{accel:>6.2f} m/s²    {metrics['total_time']:>6.2f}s   "
              f"{metrics['time_remaining']:>7.2f}s    {feasible}")
    
    simulator.acceleration = original_accel
    print()


def test_scenario(simulator, scenario_name):
    """Test predefined scenarios."""
    scenarios = {
        'conservative': {
            'velocity': 0.3,
            'acceleration': 0.5,
            'seed_time': 3.0,
            'description': 'Conservative: Slower, more time for placement'
        },
        'aggressive': {
            'velocity': 0.5,
            'acceleration': 1.0,
            'seed_time': 2.0,
            'description': 'Aggressive: Fast movement, quick placement'
        },
        'balanced': {
            'velocity': 0.4,
            'acceleration': 0.7,
            'seed_time': 2.5,
            'description': 'Balanced: Default recommended settings'
        },
        'cautious': {
            'velocity': 0.35,
            'acceleration': 0.6,
            'seed_time': 3.5,
            'description': 'Cautious: Extra time for precision'
        }
    }
    
    if scenario_name not in scenarios:
        print(f"Unknown scenario: {scenario_name}")
        print(f"Available scenarios: {', '.join(scenarios.keys())}")
        return
    
    scenario = scenarios[scenario_name]
    
    print("\n" + "="*70)
    print(f"SCENARIO TEST: {scenario_name.upper()}")
    print("="*70)
    print(f"Description: {scenario['description']}")
    print(f"Velocity: {scenario['velocity']} m/s")
    print(f"Acceleration: {scenario['acceleration']} m/s²")
    print(f"Seed placement time: {scenario['seed_time']} s")
    print()
    
    # Save original values
    orig_vel = simulator.max_velocity
    orig_acc = simulator.acceleration
    orig_seed = simulator.seed_placement_time
    
    # Apply scenario settings
    simulator.max_velocity = scenario['velocity']
    simulator.acceleration = scenario['acceleration']
    simulator.seed_placement_time = scenario['seed_time']
    
    # Run optimization
    optimization_results = simulator.optimize_route()
    optimal_route = optimization_results['optimal_route']['route_order']
    
    # Restore original values
    simulator.max_velocity = orig_vel
    simulator.acceleration = orig_acc
    simulator.seed_placement_time = orig_seed


def test_custom_route(simulator, route_order):
    """Test a specific route order."""
    print("\n" + "="*70)
    print(f"CUSTOM ROUTE TEST: {' → '.join(route_order)}")
    print("="*70)
    
    metrics = simulator.calculate_route_metrics(route_order)
    
    print(f"\nTotal Distance: {metrics['total_distance']:.3f} m")
    print(f"Total Time: {metrics['total_time']:.2f} s")
    print(f"Time Remaining: {metrics['time_remaining']:.2f} s")
    print(f"Feasible: {'✓ YES' if metrics['feasible'] else '✗ NO'}")
    
    print("\nSegment Breakdown:")
    print(f"{'From':<20} {'To':<20} {'Distance':<12} {'Time':<10}")
    print("-" * 65)
    
    for segment in metrics['segments']:
        print(f"{segment['from']:<20} {segment['to']:<20} "
              f"{segment['distance']:>6.3f} m     {segment['travel_time']:>6.2f} s")
    print()


def compare_all_scenarios(simulator):
    """Compare all predefined scenarios."""
    scenarios = ['conservative', 'balanced', 'aggressive', 'cautious']
    
    print("\n" + "="*70)
    print("COMPARING ALL SCENARIOS")
    print("="*70)
    
    results = []
    optimal_route = ['orange', 'gray', 'green']
    
    # Save original
    orig_vel = simulator.max_velocity
    orig_acc = simulator.acceleration
    orig_seed = simulator.seed_placement_time
    
    scenario_configs = {
        'conservative': {'velocity': 0.3, 'acceleration': 0.5, 'seed_time': 3.0},
        'balanced': {'velocity': 0.4, 'acceleration': 0.7, 'seed_time': 2.5},
        'aggressive': {'velocity': 0.5, 'acceleration': 1.0, 'seed_time': 2.0},
        'cautious': {'velocity': 0.35, 'acceleration': 0.6, 'seed_time': 3.5}
    }
    
    for scenario_name, config in scenario_configs.items():
        simulator.max_velocity = config['velocity']
        simulator.acceleration = config['acceleration']
        simulator.seed_placement_time = config['seed_time']
        
        metrics = simulator.calculate_route_metrics(optimal_route)
        results.append({
            'name': scenario_name,
            'metrics': metrics,
            'config': config
        })
    
    # Print comparison
    print(f"\n{'Scenario':<15} {'Velocity':<10} {'Time':<10} {'Margin':<12} {'Feasible':<10}")
    print("-" * 60)
    
    for result in results:
        feasible = "✓" if result['metrics']['feasible'] else "✗"
        print(f"{result['name']:<15} {result['config']['velocity']:>5.2f} m/s  "
              f"{result['metrics']['total_time']:>6.2f}s   "
              f"{result['metrics']['time_remaining']:>7.2f}s    {feasible}")
    
    # Restore original
    simulator.max_velocity = orig_vel
    simulator.acceleration = orig_acc
    simulator.seed_placement_time = orig_seed
    
    print("\nRECOMMENDATION:")
    best = min(results, key=lambda x: x['metrics']['total_time'])
    print(f"  Fastest: {best['name']} ({best['metrics']['total_time']:.2f}s)")
    
    safest = max(results, key=lambda x: x['metrics']['time_remaining'])
    print(f"  Most time buffer: {safest['name']} ({safest['metrics']['time_remaining']:.2f}s remaining)")
    print()


def main():
    """Main entry point for quick testing."""
    parser = argparse.ArgumentParser(
        description='Quick scenario tester for Mission 1 simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --velocity 0.35
  %(prog)s --scenario conservative
  %(prog)s --route orange,green,gray
  %(prog)s --compare-all
  %(prog)s --velocity-range 0.3 0.5
  %(prog)s --acceleration-range 0.5 1.0
        """
    )
    
    parser.add_argument('--velocity', type=float, 
                       help='Test with specific velocity (m/s)')
    parser.add_argument('--acceleration', type=float,
                       help='Test with specific acceleration (m/s²)')
    parser.add_argument('--scenario', type=str,
                       help='Test predefined scenario (conservative, aggressive, balanced, cautious)')
    parser.add_argument('--route', type=str,
                       help='Test custom route (comma-separated: orange,gray,green)')
    parser.add_argument('--compare-all', action='store_true',
                       help='Compare all predefined scenarios')
    parser.add_argument('--velocity-range', nargs=2, type=float, metavar=('MIN', 'MAX'),
                       help='Test velocity range')
    parser.add_argument('--acceleration-range', nargs=2, type=float, metavar=('MIN', 'MAX'),
                       help='Test acceleration range')
    
    args = parser.parse_args()
    
    # Initialize simulator
    print("Initializing simulator...")
    simulator = Mission1Simulator("robotics_competition.yaml")
    
    # Execute requested tests
    if args.velocity:
        orig = simulator.max_velocity
        simulator.max_velocity = args.velocity
        print(f"\n Testing with velocity: {args.velocity} m/s")
        metrics = simulator.calculate_route_metrics(['orange', 'gray', 'green'])
        print(f"Time: {metrics['total_time']:.2f}s | Margin: {metrics['time_remaining']:.2f}s")
        simulator.max_velocity = orig
    
    if args.acceleration:
        orig = simulator.acceleration
        simulator.acceleration = args.acceleration
        print(f"\nTesting with acceleration: {args.acceleration} m/s²")
        metrics = simulator.calculate_route_metrics(['orange', 'gray', 'green'])
        print(f"Time: {metrics['total_time']:.2f}s | Margin: {metrics['time_remaining']:.2f}s")
        simulator.acceleration = orig
    
    if args.scenario:
        test_scenario(simulator, args.scenario)
    
    if args.route:
        route = [x.strip() for x in args.route.split(',')]
        test_custom_route(simulator, route)
    
    if args.compare_all:
        compare_all_scenarios(simulator)
    
    if args.velocity_range:
        test_velocity_range(simulator, args.velocity_range[0], args.velocity_range[1])
    
    if args.acceleration_range:
        test_acceleration_range(simulator, args.acceleration_range[0], args.acceleration_range[1])
    
    # If no arguments, show help
    if len(sys.argv) == 1:
        parser.print_help()
        print("\nNo test specified. Try: --compare-all to see all scenarios")


if __name__ == "__main__":
    main()
