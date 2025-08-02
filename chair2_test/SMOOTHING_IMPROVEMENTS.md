# Stewart Platform Movement Smoothing

## Problem Solved
The actuators were "jitching up and down" because:
- IMU data comes in very fast (50-100Hz) with rapid, noisy changes
- Actuators can only move at 84mm/s maximum speed
- System was trying to instantly follow every small IMU change
- No filtering of noisy sensor data

## Solutions Implemented

### 1. **IMU Data Smoothing (Low-Pass Filtering)**
**Parameters:** `IMU_SMOOTHING_FACTOR = 0.15`

- Applied exponential moving average to all IMU inputs
- Filters: pitch, roll, acceleration, angular velocities
- Formula: `filtered_value = 0.15 * new_value + 0.85 * previous_filtered`
- **Effect:** Removes high-frequency noise and sudden spikes

### 2. **Target Rate Limiting**
**Parameters:** `TARGET_RATE_LIMIT = 2.0` mm/s

- Limits how fast actuator targets can change
- Prevents sudden jumps in target positions
- Calculates maximum allowed change per time step
- **Effect:** Creates smooth trajectory planning

### 3. **Deadband Filtering**
**Parameters:** `DEADBAND_THRESHOLD = 0.5` mm

- Ignores very small target changes (< 0.5mm)
- Reduces actuator "hunting" around target position
- Eliminates micro-movements from sensor noise
- **Effect:** Prevents unnecessary small movements

### 4. **Reduced Control Gain**
**Parameters:** `REDUCED_GAIN = 4.0` (was 8.0)

- Reduced proportional control aggressiveness
- Slower but smoother approach to targets
- Less overshoot and oscillation
- **Effect:** More stable, less jerky response

### 5. **Continuous Workspace Clamping**
- Clamps pitch/roll to valid ranges instead of rejecting data
- Prevents sudden switches between lookup table and fallback
- Maintains smooth operation at workspace boundaries
- **Effect:** No discontinuities at workspace limits

## Technical Implementation

### Data Flow:
```
Raw IMU Data → Low-Pass Filter → Motion Cueing → Lookup Table → 
Rate Limiting → Deadband Filter → Smooth Actuator Control
```

### Key Parameters:
| Parameter | Value | Purpose |
|-----------|--------|---------|
| `IMU_SMOOTHING_FACTOR` | 0.15 | Controls filtering strength |
| `TARGET_RATE_LIMIT` | 2.0 mm/s | Max target change rate |
| `DEADBAND_THRESHOLD` | 0.5 mm | Ignore small changes |
| `REDUCED_GAIN` | 4.0 | Proportional control gain |

### Tuning Guidelines:

**For Smoother Movement (if still jerky):**
- Decrease `IMU_SMOOTHING_FACTOR` (0.1-0.05)
- Decrease `TARGET_RATE_LIMIT` (1.5-1.0)
- Increase `DEADBAND_THRESHOLD` (0.7-1.0)
- Decrease `REDUCED_GAIN` (3.0-2.0)

**For More Responsive Movement (if too sluggish):**
- Increase `IMU_SMOOTHING_FACTOR` (0.2-0.3)
- Increase `TARGET_RATE_LIMIT` (3.0-5.0)
- Decrease `DEADBAND_THRESHOLD` (0.3-0.1)
- Increase `REDUCED_GAIN` (5.0-6.0)

## Expected Results

✅ **Smooth, continuous movement**
✅ **No more jerky actuator behavior**
✅ **Filtered sensor noise**
✅ **Stable operation at workspace boundaries**
✅ **Responsive but controlled motion**

## Debug Information

The system now logs:
- IMU smoothing initialization
- Workspace clamping when it occurs (max once/second)
- Fallback usage (only for serious errors)

## Testing

1. **Upload and monitor:**
   ```bash
   pio run --target upload
   pio device monitor
   ```

2. **Look for initialization:**
   ```
   [*] IMU smoothing initialized
   ```

3. **Test with gentle IMU movements first**

4. **Check for smooth actuator response**

5. **Verify no jerky movements during continuous motion**

The movement should now be much smoother and more natural!