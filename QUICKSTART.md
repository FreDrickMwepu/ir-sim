# Quick Start Guide - Mission 1 Simulator

## 🎯 5-Minute Quick Start

### Step 1: Run the Main Simulation
```bash
python3 mission1_simulation.py
```

**What it does:**
- Analyzes all 6 possible route orders
- Finds the optimal route
- Simulates the complete mission with realistic physics
- Generates visualization (PNG) and data (CSV)
- Shows comprehensive timing analysis

**Expected output:**
- Console: Detailed route analysis and timing breakdown
- File: `results/trajectory_*.csv` with position/velocity data
- File: `results/mission1_visualization_*.png` with field plot

---

## 🧪 Testing Different Scenarios

### Option A: Use Quick Test Utility

**Compare all predefined scenarios:**
```bash
python3 quick_test.py --compare-all
```

**Test specific velocity:**
```bash
python3 quick_test.py --velocity 0.35
```

**Test velocity range:**
```bash
python3 quick_test.py --velocity-range 0.3 0.5
```

**Test specific route order:**
```bash
python3 quick_test.py --route gray,orange,green
```

**Test predefined scenario:**
```bash
python3 quick_test.py --scenario conservative
```

Available scenarios:
- `conservative` - Slower, more time for precision
- `balanced` - Recommended default settings
- `aggressive` - Fast movement, quick placement
- `cautious` - Extra safety margin

---

### Option B: Edit Configuration File

Edit `robotics_competition.yaml`:

```yaml
robot:
  velocity:
    max_linear: 0.35  # Change from 0.4 to 0.35
  acceleration:
    linear: 0.6       # Change from 0.7 to 0.6

mission:
  seed_placement_time: 3.0  # Change from 2.5 to 3.0
```

Then run:
```bash
python3 mission1_simulation.py
```

---

## 📊 Analyzing Results

### View CSV Data

The trajectory CSV contains:
- **Time (s):** Elapsed time in mission
- **X (m), Y (m):** Robot position coordinates
- **Velocity (m/s):** Current velocity
- **Phase:** Mission phase description

**Open in Excel or Python:**
```python
import pandas as pd
df = pd.read_csv('results/trajectory_3-trip_orange_gray_green.csv')
print(df.head())
```

---

### Understanding the Visualization

The generated PNG shows:
1. **Left plot:** Competition field with robot trajectory
   - Blue rectangle: Start zone
   - Colored rectangles: Crop plots (orange, gray, green)
   - Cyan rectangle: Irrigation gate
   - Red line: Robot path with direction arrows
   - Green circle: Start position
   - Red star: End position

2. **Right plot:** Velocity and distance over time
   - Blue line: Velocity profile (shows acceleration/deceleration)
   - Green line: Cumulative distance traveled
   - Red dashed line: 120-second time limit

---

## 🔧 Common Customizations

### 1. Change Robot Starting Position

Edit `robotics_competition.yaml`:
```yaml
robot:
  position: [0.4, 0.15, 0]  # Move left (was [0.585, 0.15, 0])
```

### 2. Adjust Crop Plot Locations

Edit `robotics_competition.yaml`:
```yaml
goals:
  - name: "orange_plot"
    position: [0.20, 1.00]  # Adjust as needed
```

### 3. Test Different Mission Strategies

In `mission1_simulation.py`, modify the main() function:

**Include return to start:**
```python
final_state = simulator.simulate_trajectory(
    route_order=optimal_route,
    include_irrigation=True,
    include_return=True  # Change to True
)
```

**Test specific route:**
```python
custom_route = ['gray', 'green', 'orange']
final_state = simulator.simulate_trajectory(
    route_order=custom_route,
    include_irrigation=True
)
```

---

## 🎓 Key Metrics Explained

### Time Budget Analysis
```
Total Time: 16.49 s
Time Limit: 120 s
Time Remaining: 103.51 s
```

**Interpretation:**
- Mission takes only 16.5 seconds
- You have 103 seconds buffer
- Time is NOT a constraint - focus on accuracy

### Route Comparison
```
Rank 1: orange → gray → green  (1.994 m, 16.77 s)
Rank 6: gray → green → orange  (2.265 m, 17.45 s)
```

**Difference:** 0.68 seconds between best and worst
**Recommendation:** Use optimal route but don't stress - all routes are feasible

### Velocity Impact
```
0.30 m/s → 17.86 s
0.40 m/s → 16.77 s (recommended)
0.50 m/s → 16.01 s
```

**Insight:** Faster velocity saves time but may reduce control
**Recommendation:** 0.4 m/s provides good balance

---

## ✅ Validation Checklist

Before competition day:

- [ ] Confirmed robot fits within start zone (radius ≤ 0.1 m)
- [ ] Verified max velocity achievable with your motors (test ≤ 0.4 m/s)
- [ ] Measured actual seed placement time (update YAML if > 2.5 s)
- [ ] Tested robot can reach all waypoint positions
- [ ] Confirmed irrigation gate activation mechanism
- [ ] Practiced optimal route order multiple times
- [ ] Added safety margin to all timing estimates
- [ ] Tested emergency stop procedures
- [ ] Verified robot autonomous navigation accuracy

---

## 🐛 Troubleshooting

### Error: "FileNotFoundError: robotics_competition.yaml"
**Solution:** Run from the correct directory
```bash
cd /Users/macbookpro/Simulations
python3 mission1_simulation.py
```

### Error: "ModuleNotFoundError: No module named 'yaml'"
**Solution:** Install dependencies
```bash
pip3 install pyyaml numpy matplotlib
```

### Plots not showing
**Solution:** Run in interactive mode
```bash
python3 -i mission1_simulation.py
```

Then in Python prompt:
```python
import matplotlib.pyplot as plt
plt.show()
```

### CSV file is empty
**Solution:** Check `results/` folder was created
```bash
mkdir -p results
python3 mission1_simulation.py
```

---

## 📈 Advanced Usage Examples

### 1. Python Script Integration

```python
from mission1_simulation import Mission1Simulator

# Create simulator
sim = Mission1Simulator("robotics_competition.yaml")

# Test custom parameters
sim.max_velocity = 0.35
sim.acceleration = 0.6
sim.seed_placement_time = 3.0

# Calculate metrics
metrics = sim.calculate_route_metrics(['orange', 'gray', 'green'])
print(f"Total time: {metrics['total_time']:.2f} s")

# Run full simulation
final_state = sim.simulate_trajectory(
    route_order=['orange', 'gray', 'green'],
    include_irrigation=True
)
```

### 2. Batch Testing

```python
from mission1_simulation import Mission1Simulator

velocities = [0.3, 0.35, 0.4, 0.45, 0.5]
results = []

for vel in velocities:
    sim = Mission1Simulator("robotics_competition.yaml")
    sim.max_velocity = vel
    
    metrics = sim.calculate_route_metrics(['orange', 'gray', 'green'])
    results.append({
        'velocity': vel,
        'time': metrics['total_time'],
        'feasible': metrics['feasible']
    })

# Print results
for r in results:
    print(f"v={r['velocity']:.2f} → t={r['time']:.2f}s")
```

### 3. Custom Visualization

```python
import matplotlib.pyplot as plt
import pandas as pd

# Load trajectory
df = pd.read_csv('results/trajectory_3-trip_orange_gray_green.csv')

# Create custom plot
plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.plot(df['X (m)'], df['Y (m)'], 'b-', linewidth=2)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Robot Path')
plt.grid(True)

plt.subplot(1, 2, 2)
plt.plot(df['Time (s)'], df['Velocity (m/s)'], 'r-')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity Profile')
plt.grid(True)

plt.tight_layout()
plt.savefig('custom_analysis.png')
plt.show()
```

---

## 🏆 Competition Tips from Simulation

### Key Findings:
1. **Time is abundant** - You have 100+ seconds buffer after completing mission
2. **Route matters** - But all routes complete well within time limit
3. **Accuracy over speed** - Use slower velocity for better control
4. **Practice stops** - Most time is spent at waypoints, not traveling

### Recommended Strategy:
- ✅ Use optimal route: orange → gray → green (or reverse)
- ✅ Set velocity to 0.35-0.40 m/s (conservative but fast enough)
- ✅ Allow 3+ seconds per seed placement (be generous)
- ✅ Reserve 5+ seconds for irrigation gate
- ✅ Don't rush - accuracy earns more points than speed

### What to Practice:
1. Precise positioning at crop plots (±10mm accuracy)
2. Reliable seed placement mechanism
3. Consistent irrigation gate activation
4. Autonomous navigation without human intervention
5. Recovery from unexpected obstacles

---

## 📞 Need Help?

**Simulation issues:**
- Check file paths and working directory
- Verify all dependencies installed
- Review error messages in console

**Competition questions:**
- Refer to official ITU challenge documentation
- Consult your team mentors

**Robot hardware:**
- LEGO Spike Prime documentation
- Motor specification sheets

---

**Ready to build your robot! 🤖 Good luck! 🌱**
