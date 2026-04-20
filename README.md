# RobUST Robot Control Framework

A Unity-based real-time control system for the RobUST cable-driven robotic platform. This framework provides a cascaded real-time controller enabling easier integration of various sensors in the hardware stack as well as tension planning for cable actuation.

## Project Architecture

### Core Design Philosophy

Unlike typical Unity projects where each `MonoBehaviour` operates independently with its own `Update()` loop, this framework implements a centralized control architecture for more (soft) real-time robotic control. The system uses a single main control loop with dedicated driver threads for hardware interfaces.

### Key Components
Unity Scripts are always located in Assets/Scripts. These scripts can be attached to specific game objects in the Unity Scene. A basic Unity Scene for this project is implemented with a default `MainCamera`, an empty `RobotSystem` GameObject, and 3 tracker visuals for the robot frame, chest belt frame, and pelvic belt frame.

| Script | Function |
|--------|----------|
| `RobotController.cs` | Main control loop coordinating all subsystems |
| `LabviewTcpCommunicator.cs` | TCP interface to LabVIEW motor controllers |
| `TrackerManager.cs` | HTC Vive tracker interface via OpenVR |
| `ForcePlateManager.cs` | Vicon force plate integration via .NET SDK |
| `CableTensionPlanner.cs` | Quadratic programming solver for cable tensions |
| `RobotVisualizer.cs` | Unity scene visualization updates |
| `DataStructures.cs` | Shared data types and utilities |

## Driver Architecture

The framework employs three main driver scripts (`LabviewTcpCommunicator`, `TrackerManager`, `ForcePlateManager`) designed as standalone, reusable components. Each driver:

- Runs in its own thread with high-precision timing using Unity's `System.Diagnostics.Stopwatch`
- Maintains pseudo-deterministic sampling frequencies
- Uses data locks for thread-safe communication with the main control loop
- Can be integrated into other RobUST projects without modification

### `LabviewTcpCommunicator.cs`

Manages communication with the low-level motor controller running on a PXIe system:

- **Frequency**: 1000 Hz TCP transmission
- **Configuration**: PXIe IP address set in Unity Inspector, `tcpCommunicator.ConnectToServer()` called during initialization
- **Interface**: `UpdateTensionSetpoint(ReadOnlySpan<double> newTensions)` for allocation-free updates
- **Protocol**: Sends control mode + tension setpoints via TCP
- **Port**: 8053 (configurable)

### `TrackerManager.cs`

Interfaces with HTC Vive tracking system for pose estimation:

- **Frequency**: 90 Hz (limited by HTC hardware)
- **API**: OpenVR for direct access to tracker transformation matrices
- **Configuration**: Set up to support 3 trackers (frame reference, belt frames) but more can be added
- **Setup**: Automatically discovers connected trackers and displays serial numbers for configuration
- **Coordinate System**: Uses [HTC Vive tracker developer guidelines](https://developer.vive.com/documents/850/HTC_Vive_Tracker_3.0_Developer_Guidelines_v1.1_06022021.pdf) frame definition (page 21)

### `ForcePlateManager.cs`

Integrates with Vicon force measurement systems:

- **Interface**: Vicon DataStream SDK via Unity Vicon Plugin
- **Architecture**: Utilizies `ServerPush` mode - Vicon Box triggers reliable 100 Hz timing

## Robot Configuration
### RobUSTDescription

Acts as the single source of truth for the system's physical dimensions, hardware mappings, and user-specific biomechanical parameters required for kinematics, dynamics, and control:

- **Zero Runtime Allocation**: Instantiated once at startup via the `Create()` factory method, pre-allocating all arrays to prevent garbage collection spikes during high-frequency control loops
- **Flexible Cable Configurations**: Natively builds the correct active subset arrays depending on whether the system is initialized as a 4-cable (top only) or full 8-cable setup
- **Hardware Mapping**: Manages the precise routing between mathematical solver indices and the actual physical motor driver channels 
- **Robot Geometry**: Stores the hardcoded, Vive Tracker-calibrated global coordinates for all 8 frame pulleys and the 4 corners of the force plates
- **User Biometrics & End-Effector**: Calculates local belt attachment points and stores user variables (mass, trunk height, chest dimensions) to accurately scale the control model for different individuals

### `DataStructures.cs`

Foundational structs, base classes, and profiling utilities used across the robot's control, tracking, and prediction systems:

- **Hardware Data Models**: Defines `TrackerData` for managing right-handed OpenVR 4x4 pose matrices and `ForcePlateData` for tracking ground reaction forces and centers of pressure
- **Kinematic States**: Provides `RBState` (Rigid Body state containing position, Euler angles, and linear/angular velocities) specifically tailored for Model Predictive Control (MPC) predictions alongside a 6-DoF `Wrench` struct
- **Controller Architecture**: Establishes `BaseController<T>`, an abstract foundation for high-level control systems (e.g., MPC and Stability controllers) to compute subsequent control steps
- **Performance Profiling**: Includes `RobotProfiler` with dedicated Unity Profiler categories for tracking custom background thread workloads and execution intervals

## Physics and Control

### CableTensionPlanner

Solves the cable tension optimization problem using quadratic programming:

- **Solver**: Alglib Convex QP implementation (dense IPM)
- **Objective**: achieve desired wrench while minimizing parasitic wrench and maintaining minimum cable tensions
- **Configuration Parameters**:
  - Number of cables
  - Chest anteroposterior distance
  - Chest mediolateral distance
  - Belt size (small/medium/large)
- **Primary Function**: `public double[] CalculateTensions(double4x4 eeInRobotFrame, Wrench desiredWrench)`

### RobotVisualizer

Handles Unity scene updates and coordinate frame transformations for visualization only:

- **Coordinate Conversion**: Right-handed tracker data to left-handed Unity coordinate system
- **Initialization**: Draws spheres for pulley locations for reference positioning
- **Camera Setup**: Positions 3 Unity cameras relative to frame tracker for multiple robot views

## Setup and Installation

### Prerequisites

- Unity Hub with Unity Editor installed
- Steam account with SteamVR
- HTC Vive base stations and headset
- 3 HTC Vive trackers

### Quick Start

1. Clone Repository
2. Open Project in Unity
- Launch Unity Hub
- Select "Open Project"
- Navigate to and select the "Unity RobUST Controller" folder
3. Load Scene
- Open the "Robot Controller Scene" in Unity
4. Configure SteamVR
- Install SteamVR through Steam
- Connect and power on base stations and headset
- Pair the 3 Vive trackers using "Pair Controller" in SteamVR
- Verify tracker status icons are lit in SteamVR interface
5. Configure Tracker Serial Numbers
- Run the project once to see discovered tracker serial numbers in console
- Copy serial numbers to appropriate variables in TrackerManager:
  - Frame tracker serial
  - Chest belt tracker serial
  - Pelvis belt tracker serial
6. Open Vicon Software
- Go into "Live Mode" to for box-triggered data streaming

**Simulate Local TCP Listener**

For development and testing without commanding tensions to the LabVIEW motor control system set IP address in inspector to `127.0.0.1` or `localhost` and run a local TCP listener using `ncat` or similar tool:

```bash
ncat -l 8053
```
This will display incoming motor commands for verification of communication protocols.

**Execute Low-Level Tension Control System in LabVIEW**

1. Open `RobUST_Boilerplate.lvproj`
2. Open and run `Host_Panel.vi` under host computer
3. Open and run `Controllers/TCP_main.vi` under PXIe and run.
4. with cables slack, click tare
5. run Unity program and `LabviewTCPCommunicator` should connect 

### Dependencies
- Unity: 2021.3 LTS or newer
- SteamVR: Latest version through Steam
- OpenVR: Included with SteamVR installation
- Vicon Unity Plugin 1.3 `.unitypackage`
- Vicon System compatible with ViconDataStreamSDK 
- Alglib: Included for quadratic programming solver ---> Plugins Folder

### Usage Notes
- Ensure all trackers are awake before starting
- Frame tracker position is captured once during initialization and used as reference
- System requires SteamVR to be running and trackers connected before Unity execution
