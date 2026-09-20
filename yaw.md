# Understanding Yaw-Jump Compensation Math in Flight Controllers

**Yaw-jump compensation** (often managed via **motor mixing saturation algorithms**) solves a fundamental physics problem in multirotors:
**yaw control relies on torque, whereas roll and pitch control rely on thrust**.
Because torque forces are much weaker than thrust forces,
aggressive yaw inputs can cause a drone to unexpectedly balloon upward (the "jump") or lose its attitude stability.

---

## 1. The Core Problem: Motor Mixing Saturation

In a standard quadcopter, the flight controller calculates a baseline command for each motor using a linear matrix mixer.
For a standard X-configuration drone, the raw motor commands look like this:

* Motor 1 = Throttle + Roll - Pitch - Yaw
* Motor 2 = Throttle - Roll + Pitch + Yaw
* Motor 3 = Throttle + Roll + Pitch + Yaw
* Motor 4 = Throttle - Roll - Pitch - Yaw

### What happens during an aggressive yaw command?

To execute a rapid clockwise yaw, the flight controller must abruptly
**increase** the speed of the counter-clockwise spinning motors (Motor 2 and Motor 3) and
**decrease** the clockwise spinning motors (Motor 1 and Motor 4).

If you are flying at 50% throttle and input a large yaw command (eg, +60%), the mathematical calculation yields:

* **Motors 2 & 3:** 50% + 60% = 110%
* **Motors 1 & 4:** 50% - 60% = -10%

Physical motors cannot spin at 110% or -10%. They saturate at their hard physical limits: **Max (100%)** and **Min (0%)**.

If the flight controller simply caps the values at 100% and 0%, the real-world execution becomes:

* **Motors 2 & 3:** 100% (clamped from 110%) — *An increase of 50%*
* **Motors 1 & 4:** 0% (clamped from -10%) — *A decrease of 50%*

### Why does the drone "jump"?

When the requested command is asymmetric relative to the hard physical limits, the mathematical balance breaks down.

If the flight controller prioritizes stabilization or attempts to preserve the high yaw rate at all costs,
it will dynamically push up the lower bounds to prevent the opposite motors from stopping entirely (0%).
When it shifts this baseline upward to squeeze out more yaw authority, the total sum of thrust spikes, popping the drone upward.

---

## 2. The Compensation Mathematics

To prevent yaw-jumps and maintain control, libraries like Betaflight and `crate-motor-mixers` implement a **Dynamic Mixer Range Scaling** algorithm.
Instead of blindly clipping individual values,
the math dynamically shifts and shrinks the requested components to fit perfectly within the achievable 0% to 100% bounds.

### Step A: Axis Prioritization

Flight stability relies on **Roll** and **Pitch** to stay airborne. Yaw is secondary.
The mixer first calculates the minimum and maximum boundaries required strictly by the current Roll and Pitch outputs:

* Current_Max = Throttle + Absolute(Roll) + Absolute(Pitch)
* Current_Min = Throttle - Absolute(Roll) - Absolute(Pitch)

### Step B: Checking for Over-allocation

The mixer checks how much headroom is left for Yaw. If the requested Yaw command exceeds the boundaries, saturation will occur. The headroom limit is defined as:

* Allowed_Yaw = Minimum of (100% - Current_Max) AND (Current_Min - 0%)

### Step C: Constraining or Reduction (The Compensation)

If the requested Yaw is greater than Allowed_Yaw, the mixer applies one of two mathematical corrections:

* **Method 1: Yaw Reduction (Preserves Attitude & Throttle)**
  The mixer clips or scales down the Yaw component so that it never forces a motor past its hard ceiling or floor.
  
  * Formula: Scaled_Yaw = Direction_of_Yaw * Minimum of (Absolute(Yaw) AND Allowed_Yaw)
  
  * Result: The drone yaws exactly as fast as physically possible without altering the average throttle or disrupting roll/pitch stabilization. **No yaw-jump occurs.**

* **Method 2: Dynamic Throttle Reduction (Preserves Yaw Authority)**
  If the pilot wants maximum yaw rate at all costs, the mixer will temporarily **reduce the baseline throttle** component to make structural room for the yaw command.
  If a motor hits 110%, it calculates the excess delta (Delta = 10%) and subtracts it directly from the throttle mix:
  
  * Formula: Adjusted_Throttle = Throttle - Delta
  
  * Result: By pulling the overall throttle down during the fast spin, it mathematically offsets the thrust spike from the spinning-up motor pairs,
    giving a perfectly flat spin with zero altitude change.
