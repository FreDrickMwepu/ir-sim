# ITU Robotics for Good Youth Challenge 2025-2026
## Mission 1: Cultivation and Irrigation Simulator

### 📋 Overview

This simulation environment helps teams prepare for Mission 1 of the ITU Robotics for Good Youth Challenge. It provides comprehensive analysis of robot navigation strategies, timing optimization, and route planning for the cultivation and irrigation tasks.

**Purpose:** Research and development tool to optimize competition strategy before building the physical robot.

---

## 🚀 Quick Start

### Prerequisites
```bash
pip install numpy matplotlib pyyaml
```

### Running the Simulation
```bash
python3 mission1_simulation.py
```

The simulation will automatically:
1. ✅ Analyze all possible route orders (6 permutations)
2. ✅ Identify the optimal route
3. ✅ Simulate the complete trajectory with timing
4. ✅ Compare different strategies (2-trip vs 3-trip)
5. ✅ Test various velocity parameters
6. ✅ Generate visualization and export data

---

## 📁 Project Structure

```
Simulations/
├── robotics_competition.yaml      # Configuration file (field layout, robot specs)
├── mission1_simulation.py         # Main simulation script
├── route_optimizer.py             # Advanced route optimization utilities
└── results/                       # Output folder
    ├── trajectory_*.csv           # Trajectory data exports
    └── mission1_visualization_*.png  # Plots and visualizations
```

---

## ⚙️ Configuration File (`robotics_competition.yaml`)

### Key Parameters You Can Adjust

#### Robot Parameters
```yaml
robot:
  radius: 0.1              # Robot footprint (meters)
  velocity:
    max_linear: 0.4        # Maximum speed (m/s) - TRY: 0.3 to 0.5
  acceleration:
    linear: 0.7            # Acceleration (m/s²) - TRY: 0.5 to 1.0
```

#### Mission Timing
```yaml
mission:
  time_limit: 120          # Maximum mission time (seconds)
  seed_placement_time: 2.5 # Time per crop plot (seconds)
  irrigation_activation_time: 2.0  # Time at gate (seconds)
```

#### Field Layout
- **Start Zone:** `position: [0.585, 0.15]`
- **Orange Plot:** `position: [0.25, 0.995]` (left side)
- **Gray Plot:** `position: [0.585, 0.995]` (center)
- **Green Plot:** `position: [0.92, 0.995]` (right side)
- **Irrigation Gate:** `position: [0.585, 0.75]`

---

## 🎯 Key Features

### 1. Route Optimization
Compares all possible visiting orders for the 3 crop plots:
- Sequential orders (orange → gray → green)
- Distance-minimizing orders
- Time-optimal orders

**Output:** Ranked table showing distance, time, and feasibility for each route.

### 2. Trajectory Simulation
Generates realistic robot motion with:
- ✅ Acceleration/deceleration profiles (trapezoidal velocity)
- ✅ Precise waypoint navigation
- ✅ Stop times for seed placement and irrigation
- ✅ Real-time tracking of position and velocity

### 3. Timing Analysis
Detailed breakdown of mission phases:
- Travel time between waypoints
- Stop duration at each location
- Cumulative time tracking
- Time budget remaining

### 4. Strategy Comparison
Tests different mission approaches:
- **2-trip strategy:** Visit all 3 plots in sequence
- **3-trip strategy:** Return to start between plots (if needed)
- With/without return to start zone after irrigation

### 5. Parameter Sweep
Analyze sensitivity to different velocities:
- Tests range of speeds (e.g., 0.3, 0.35, 0.4 m/s)
- Shows impact on total mission time
- Helps identify optimal operating speed

### 6. Visualization
Generates comprehensive plots showing:
- Competition field layout with obstacles
- Crop plot locations (color-coded)
- Complete robot trajectory with direction arrows
- Velocity profile over time
- Distance traveled over time

### 7. Data Export
Saves detailed trajectory data to CSV:
- Columns: Time, X, Y, Velocity, Mission Phase
- Ready for analysis in Excel or Python
- Useful for further optimization

---

## 📊 Understanding the Output

### Console Output Example

```
======================================================================
ROUTE OPTIMIZATION ANALYSIS
======================================================================
Testing 6 possible route orders...

Rank   Route Order               Distance (m)    Time (s)     Remaining (s)   Feasible  
-----------------------------------------------------------------------------------------------
1      orange → gray → green        1.994         16.77        103.23      ✓ YES     
2      green → gray → orange        1.994         16.77        103.23      ✓ YES     
...

OPTIMAL ROUTE: orange → gray → green
Total Distance: 1.994 m
Total Time: 16.77 s
Time Remaining: 103.23 s
```

**Interpretation:**
- **Distance:** Total path length (shorter = more efficient)
- **Time:** Includes travel + stop times (must be < 120s)
- **Time Remaining:** Buffer for unexpected delays
- **Feasible:** ✓ if within 120-second time limit

### Simulation Summary

```
SIMULATION SUMMARY
======================================================================
Total Distance Traveled: 1.994 m
Total Time: 16.49 s
Time Remaining: 103.51 s
Mission Feasibility: ✓ FEASIBLE
Average Velocity: 0.121 m/s
```

**Key Metrics:**
- **Average Velocity:** Lower than max due to acceleration/deceleration
- **Feasibility:** Confirms mission can be completed within time limit
- **Time Remaining:** Large buffer means you can afford to be conservative

---

## 🔧 Customization Guide

### Testing Different Scenarios

#### Scenario 1: Faster Robot
Edit `robotics_competition.yaml`:
```yaml
robot:
  velocity:
    max_linear: 0.5  # Increase from 0.4 to 0.5
```

#### Scenario 2: Slower Seed Placement
Edit `robotics_competition.yaml`:
```yaml
mission:
  seed_placement_time: 3.5  # Increase from 2.5 to allow more time
```

#### Scenario 3: Test Specific Route
In `mission1_simulation.py`, change the main() function:
```python
# Instead of optimal route, test specific order
custom_route = ['gray', 'orange', 'green']
final_state = simulator.simulate_trajectory(
    route_order=custom_route,
    include_irrigation=True,
    include_return=False
)
```

#### Scenario 4: Include Return to Start
```python
final_state = simulator.simulate_trajectory(
    route_order=optimal_route,
    include_irrigation=True,
    include_return=True,  # Change to True
)
```

---

## 📈 Advanced Usage

### Using the Route Optimizer Module

```python
from route_optimizer import RouteOptimizer
import numpy as np

# Define your waypoints
waypoints = {
    'orange': np.array([0.25, 0.995]),
    'gray': np.array([0.585, 0.995]),
    'green': np.array([0.92, 0.995])
}

start_pos = np.array([0.585, 0.15])

# Create optimizer
optimizer = RouteOptimizer(waypoints, start_pos)

# Try different strategies
nn_route = optimizer.nearest_neighbor()
optimal_route, distance = optimizer.brute_force_optimal()

# Compare all strategies
optimizer.print_comparison_table()
```

### Analyzing Trajectory Data

The CSV output can be analyzed in Python:

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load trajectory data
df = pd.read_csv('results/trajectory_3-trip_orange_gray_green.csv')

# Plot custom analysis
plt.figure(figsize=(10, 6))
plt.plot(df['Time (s)'], df['Velocity (m/s)'])
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity Profile')
plt.grid(True)
plt.show()

# Calculate statistics
max_velocity = df['Velocity (m/s)'].max()
avg_velocity = df['Velocity (m/s)'].mean()
print(f"Max velocity: {max_velocity:.3f} m/s")
print(f"Average velocity: {avg_velocity:.3f} m/s")
```

---

## 🎓 Key Findings from Simulation

Based on default configuration:

### ✅ Mission Feasibility
- **Result:** Mission is highly feasible
- **Total time:** ~16.5 seconds (including all stops)
- **Time margin:** ~103 seconds remaining
- **Confidence:** Can complete with significant time buffer

### 🏆 Optimal Strategy
- **Best route:** orange → gray → green (or reverse)
- **Distance:** 1.994 meters
- **Why optimal:** Minimizes backtracking by visiting plots left-to-right

### ⚡ Speed Recommendations
- **0.3 m/s:** Very conservative, 17.9s total
- **0.4 m/s:** Recommended balance, 16.8s total
- **0.5 m/s:** Faster but may reduce control

### 🎯 Competition Strategy
1. **Route order matters:** Difference of 0.7s between best and worst
2. **Abundant time budget:** Focus on accuracy over speed
3. **No need to rush:** Time is not the limiting factor
4. **Reserve time for irrigation:** Still 103+ seconds after seeding

---

## 🐛 Troubleshooting

### Issue: Import Error for `ir-sim`
**Note:** This simulation uses standard Python libraries (numpy, matplotlib, yaml) and does NOT require the `ir-sim` library. The YAML configuration is compatible with ir-sim format but the simulation runs independently.

### Issue: Visualization Not Showing
If plots don't appear:
```python
# At end of mission1_simulation.py
plt.show()  # Should be included
```

Try running with:
```bash
python3 -i mission1_simulation.py  # Interactive mode
```

### Issue: Permission Denied for Results Folder
```bash
mkdir -p results
chmod 755 results
```

---

## 📚 Competition Rules Reference

### Mission 1 Scoring (from competition rules)
- **Seed placement:** Points per correctly placed seed
- **Irrigation:** Points for activating gate (selective irrigation in senior)
- **Time bonus:** Extra points for completing under time threshold
- **Penalties:** Incorrect seed placement, watering empty plots

### Technical Specifications
- **Field:** 1171mm × 1143mm per team
- **Time limit:** 120 seconds (2 minutes)
- **Seeds:** 18 total (6 small, 6 medium, 6 large)
- **Robot constraints:** Must fit within start zone at beginning

---

## 🔄 Future Enhancements

Potential additions for later development:

- [ ] Mission 2 simulation (Weeding and Harvesting)
- [ ] Obstacle avoidance algorithms
- [ ] Multiple robot coordination
- [ ] Real-time sensor simulation (line following, distance sensors)
- [ ] Integration with actual ir-sim visualization
- [ ] 3D visualization of robot motion
- [ ] Machine learning for path optimization
- [ ] ROS integration for hardware testing

---

## 🤝 Contributing

This is a competition preparation tool. Modifications and improvements are encouraged:

1. Test different robot configurations
2. Add new routing algorithms
3. Improve visualization
4. Optimize timing parameters
5. Share findings with your team

---

## 📞 Support

For questions about:
- **Simulation:** Check code comments and this README
- **Competition rules:** Refer to official ITU challenge documentation
- **Hardware:** Consult LEGO Spike Prime documentation

---

## 📄 License

This simulation is created for educational and competition preparation purposes.

---

## 🏁 Getting Started Checklist

- [x] Install dependencies (`pip install numpy matplotlib pyyaml`)
- [ ] Review competition field layout in YAML
- [ ] Run simulation with default settings
- [ ] Analyze route optimization results
- [ ] Review trajectory visualization
- [ ] Examine CSV output data
- [ ] Test different velocity parameters
- [ ] Customize for your robot specifications
- [ ] Document findings for your team
- [ ] Use insights to plan physical robot design

---

**Good luck in the ITU Robotics for Good Youth Challenge 2025-2026! 🤖🌱**

---

*Last updated: December 26, 2025*
