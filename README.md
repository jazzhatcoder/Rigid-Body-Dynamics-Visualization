# Rigid Body Dynamics Visualization

A MATLAB simulation for visualizing rigid body dynamics using quaternion-based attitude parameterization. This project demonstrates torque-free rotation of a rigid body (cube) with configurable inertia properties, featuring real-time 3D visualization and spherical trajectory analysis.

## Demonstration

https://github.com/jazzhatcoder/Rigid-Body-Dynamics-Visualization/blob/326d0daca02e04fb943d66087f56b4911cde4f09/Intermediate%20Axis.mp4

## Features

### Core Simulation
- **Quaternion Kinematics**: Uses quaternions for singularity-free attitude representation
- **RK4 Integration**: Fourth-order Runge-Kutta numerical integration for accurate dynamics
- **General Inertia Tensor**: Supports full 3x3 inertia tensors with off-diagonal products of inertia
- **Euler's Equations**: Implements torque-free rigid body dynamics

### Visualizations

#### 1. Real-Time 3D Cube Animation
- Animated rotating cube with color-coded faces
- Principal axes (X, Y, Z) displayed in red, green, and blue
- Instantaneous rotation axis (dashed black line)
- Angular momentum vector (magenta dash-dot line, fixed in inertial space)
- Live quaternion and time display

#### 2. Interactive Spherical Representation
- Motion of principal axes traced on a unit sphere
- Real-time path tracking for all three principal axes
- Synchronized cube visualization within the sphere
- Time slider for playback control
- Play/pause animation controls
- Axis isolation buttons (view individual or all axes)
- Angular momentum direction indicator

#### 3. Component Analysis Plots
- X, Y, and Z components of each principal axis over time
- Reference lines showing angular momentum components
- Three separate figures for comprehensive analysis

## Files

- **`simulation.m`** - Main simulation script with all visualization features
- **`derivatives.m`** - Computes state derivatives (quaternion kinematics and Euler's equations)
- **`quat2dcm.m`** - Converts quaternions to direction cosine matrices (rotation matrices)

## Requirements

- MATLAB (tested with recent versions)
- No additional toolboxes required

## Usage

### Basic Simulation

Simply run the main script:

```matlab
simulation
```

### Customizing Parameters

Edit the parameters section in `simulation.m`:

```matlab
% Inertia tensor (3x3 symmetric matrix)
I_tensor = [5, 0, 0;    % Ixx, Ixy, Ixz
           0, 5, 0;     % Iyx, Iyy, Iyz
           0, 0, 5];    % Izx, Izy, Izz

% Initial angular velocity [rad/s]
w0 = [0., 0.5, 3.];  % [wx, wy, wz]

% Initial orientation (quaternion, scalar first)
q0 = [1, 0, 0, 0];   % Identity orientation

% Time parameters
tspan = [0 10];      % Simulation duration [seconds]
dt = 0.01;           % Time step [seconds]
```

### Interesting Configurations

The code includes commented examples demonstrating different dynamics:

1. **Intermediate Axis Theorem** (unstable rotation):
   ```matlab
   I_tensor = [1, 0, 0; 0, 5, 0; 0, 0, 3];
   w0 = [0.5, 0, 3.];
   ```

2. **Principal Axis Rotation** (stable):
   ```matlab
   I_tensor = [1, 0, 0; 0, 5, 0; 0, 0, 3];
   w0 = [0, 0, 3.];
   ```

3. **Symmetric Body Precession**:
   ```matlab
   I_tensor = [5, 0, 0; 0, 5, 0; 0, 0, 5];
   w0 = [0.5, 0, 3.];
   ```

## Interactive Controls

### Spherical Representation Figure

- **Time Slider**: Scrub through the simulation timeline
- **Play/Pause Button**: Animate the trajectory automatically
- **Axis Buttons**:
  - "All Axes" - Show all three principal axes
  - "X Only" - Isolate X-axis trajectory
  - "Y Only" - Isolate Y-axis trajectory
  - "Z Only" - Isolate Z-axis trajectory

## Mathematical Background

### Quaternion Kinematics

The orientation evolution is governed by:

```
dq/dt = 0.5 * q ⊗ [0; ω]
```

where `q` is the quaternion and `ω` is the angular velocity vector.

### Euler's Equations

For torque-free motion with a general inertia tensor:

```
I * dω/dt + ω × (I * ω) = 0
```

This is solved for the angular acceleration:

```
dω/dt = -I⁻¹ * (ω × (I * ω))
```

### Conservation Properties

- **Angular Momentum**: Constant in inertial frame (visualized as fixed magenta vector)
- **Rotational Energy**: Conserved throughout the simulation
- **Quaternion Normalization**: Maintained at each time step

## Output Figures

The simulation generates 6 figures:

1. **Rotating Cube** - 3D animation of the cube with axes
2. **Live Spherical Representation** - Real-time sphere plot during animation
3. **Interactive Spherical Representation** - Sphere plot with time controls
4. **X Components** - Time history of X-components of all principal axes
5. **Y Components** - Time history of Y-components of all principal axes
6. **Z Components** - Time history of Z-components of all principal axes

## Physics Insights

This simulation beautifully demonstrates several fundamental concepts:

- **Polhodes**: The curves traced by principal axes on the sphere
- **Precession and Nutation**: Visible when the body has asymmetric inertia
- **Intermediate Axis Theorem**: Rotation about the intermediate principal axis is unstable
- **Conservation Laws**: Angular momentum remains fixed in space

## Code Structure

### Main Simulation Loop
1. Initialize state with quaternion and angular velocity
2. Integrate equations of motion using RK4
3. Normalize quaternion to prevent drift
4. Store state history

### Visualization Pipeline
1. Convert quaternions to rotation matrices (DCM)
2. Transform cube vertices and axes
3. Update graphics objects
4. Synchronize multiple figure windows

### Callback Functions
- `updateMarkers()` - Updates sphere plot based on slider position
- `togglePlayback()` - Controls animation playback
- `animationTimerCallback()` - Timer function for smooth animation
- `showAll()`, `showXOnly()`, etc. - Axis isolation controls

## Tips

- Adjust `dt` for accuracy vs. speed trade-off
- Use smaller time steps (`dt = 0.001`) for highly asymmetric inertia tensors
- The animation speed can be modified by changing the `pause()` duration in the main loop
- Close figure windows to stop timers and free memory

## References

This simulation implements standard rigid body dynamics theory found in:
- Classical mechanics textbooks (Goldstein, Marion & Thornton)
- Spacecraft dynamics literature
- Quaternion algebra and rotation representation

## License

Feel free to use and modify this code for educational and research purposes.

## Author

Created for visualizing and understanding rigid body dynamics, quaternion kinematics, and rotational motion.
