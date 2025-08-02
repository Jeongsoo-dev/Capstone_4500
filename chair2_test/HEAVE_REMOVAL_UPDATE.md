# Heave Removal - Focus on Pitch and Roll Only

## Changes Made

The Stewart platform now focuses purely on **pitch and roll movements**, with all heave (vertical Z-axis acceleration) functionality removed for simplified operation.

### ✅ **Code Changes:**

1. **Disabled Heave Parameters**
   - Commented out `heaveScaleFactor` constant
   - Added clear comments indicating heave is disabled

2. **Removed Z-Acceleration Processing**
   - No longer extracts `acc_z` from JSON
   - Removed `filteredAccZ` smoothing variable
   - Updated smoothing function to exclude acceleration filtering

3. **Simplified Actuator Calculations**
   - Lookup table results used directly (no heave offset added)
   - Mathematical fallback removes heave parameter
   - `computeActuatorLength()` signature simplified

4. **Updated Function Signatures**
   - `applyIMUSmoothing()` - removed `acc_z` parameter
   - `computeActuatorLength()` - removed `heave` parameter

5. **Cleaned Up Comments and Debug Output**
   - Updated initialization message: `"IMU smoothing initialized (pitch/roll focus)"`
   - Removed heave references from debug logging

### 🎯 **What Now Works:**

- **Pure Rotational Motion**: Only pitch and roll angles control the platform
- **Motion Cueing**: Angular velocity still adds dynamic response
- **Smooth Operation**: All smoothing and rate limiting preserved
- **Lookup Table**: Precision positioning using your calibrated table
- **Fallback Safety**: Mathematical backup still available

### 📊 **Data Flow (Simplified):**

```
IMU Data → Extract Pitch/Roll + Angular Velocities → 
Low-Pass Smoothing → Motion Cueing → Lookup Table → 
Rate Limiting → Smooth Actuator Control
```

### 🔧 **Result:**
The platform will now respond purely to **tilt movements** (pitch and roll) with smooth, continuous motion. No vertical bounce or heave effects from acceleration changes.

### 🚀 **Ready to Test:**

```bash
cd chair2_test
pio run --target upload
pio device monitor
```

**Expected startup message:**
```
[*] IMU smoothing initialized (pitch/roll focus)
[✓] Lookup table loaded successfully
```

The platform will now provide pure pitch/roll motion that directly corresponds to your IMU orientation without any vertical acceleration effects! ✨