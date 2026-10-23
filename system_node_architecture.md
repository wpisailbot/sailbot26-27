# Complete Node Architecture for Sailbot System

## System-Wide Node Architecture

```mermaid
graph TB
    subgraph "External Client Layer"
        FLUTTER[Flutter Telemetry App<br/>Cross-platform UI<br/>Gamepad Input<br/>Map Visualization<br/>Control Interface]
        FLUTTER_COMPONENTS[Flutter Components:<br/>NetworkComms gRPC Client<br/>AutonomousModeProvider<br/>Riverpod State Management<br/>Map Camera Widgets<br/>Control Widgets]
    end
    
    subgraph "Network Communication Layer"
        GRPC_PORT[gRPC Server<br/>Port 50051<br/>Protocol Bridge]
    end
    
    subgraph "ROS2 Lifecycle Nodes Managed by state_manager"
        subgraph "Core Communication"
            NETWORK[network_comms<br/>gRPC to ROS2 Bridge<br/>Telemetry Aggregation<br/>Heartbeat Monitor<br/>Command Processing]
        end
        
        subgraph "Navigation System"
            PATH_GEN[path_generator<br/>Waypoint Management<br/>Path Planning<br/>Route Calculation]
            PATH_FOL_VF[path_follower_vf<br/>Vector Field Navigation<br/>Target Following<br/>Look-ahead Logic]
            STATION_VF[station_keep_vf<br/>Station Keeping<br/>Pattern Generation<br/>Position Hold]
            SEARCH_VF[search_rescue_vf<br/>Search Patterns<br/>Lawnmower Navigation<br/>Area Coverage]
            COLLISION_VF[collision_avoidance_vf<br/>Obstacle Avoidance<br/>Dynamic Planning<br/>Safety Override]
        end
        
        subgraph "Control System"
            HEAD_CTRL_VF[heading_controller_vf<br/>Heading Control<br/>Fuzzy Logic<br/>Vector Field Integration]
            HEAD_CTRL[heading_controller<br/>Basic Heading Control<br/>PID Control<br/>Rudder Commands]
        end
        
        subgraph "Sensor Processing"
            AIRMAR[airmar_reader<br/>GPS Compass Data<br/>Marine Sensors<br/>NMEA Protocol]
            WIND_SMOOTH[wind_smoother<br/>Wind Data Filtering<br/>Signal Processing<br/>Smoothing Algorithms]
            BUOY_DETECT[buoy_detection<br/>Computer Vision<br/>Object Detection<br/>Camera Processing]
        end
        
        subgraph "Hardware Interface"
            ESP32[esp32_comms<br/>ESP32 Communication<br/>Trim Tab Control<br/>Serial Interface]
        end
        
        subgraph "Pathfinding Service"
            PATHFINDER[pathfinder_node<br/>A* Algorithm<br/>Route Planning<br/>C++ Performance]
        end
    end
    
    subgraph "ROS2 Standard Nodes"
        STATE_MGR[state_manager<br/>Lifecycle Management<br/>Node Orchestration<br/>Startup Sequence]
        HEAD_SELECT[heading_select<br/>Heading Source Selection<br/>Data Fusion<br/>Sensor Switching]
        FAKE_MOVE[fake_movement<br/>Simulation Mode<br/>Position Simulation<br/>Testing Support]
    end
    
    subgraph "Hardware Layer"
        PWM[PWM Controller<br/>Servo Control<br/>Motor Commands<br/>Heartbeat]
        TRIM_TAB[Trim Tab Hardware<br/>Physical Actuator<br/>Angle Control<br/>Status Feedback]
        SENSORS[Physical Sensors<br/>Airmar Weather Station<br/>ZED Camera<br/>Marine Instruments]
    end
    
    FLUTTER --> FLUTTER_COMPONENTS
    FLUTTER_COMPONENTS -.->|gRPC Commands| GRPC_PORT
    GRPC_PORT --> NETWORK
    
    NETWORK -->|Topics & Services| PATH_GEN
    NETWORK -->|single_waypoint| PATH_FOL_VF
    NETWORK -->|autonomous_mode| HEAD_CTRL_VF
    NETWORK -->|control commands| ESP32
    
    PATH_GEN -->|current_path| PATH_FOL_VF
    PATH_FOL_VF -->|target_position| HEAD_CTRL_VF
    HEAD_CTRL_VF -->|rudder_angle| PWM
    
    PATH_GEN -.->|current_path| STATION_VF
    PATH_GEN -.->|current_path| SEARCH_VF
    PATH_GEN -.->|current_path| COLLISION_VF
    
    AIRMAR -->|sensor_data| NETWORK
    AIRMAR -->|lat_long, heading| PATH_FOL_VF
    WIND_SMOOTH -->|true_wind_smoothed| PATH_FOL_VF
    BUOY_DETECT -->|buoy_position| NETWORK
    
    ESP32 -->|trim_state| NETWORK
    ESP32 <-->|Serial Comm| TRIM_TAB
    PWM <-->|PWM Signals| SENSORS
    
    STATE_MGR -.->|Lifecycle Control| NETWORK
    STATE_MGR -.->|Lifecycle Control| PATH_GEN
    STATE_MGR -.->|Lifecycle Control| AIRMAR
    
    PATHFINDER -->|GetPath Service| PATH_GEN
    HEAD_SELECT -->|heading| HEAD_CTRL_VF
    
    NETWORK -.->|telemetry_stream| FLUTTER_COMPONENTS
    PWM -.->|heartbeat| NETWORK
    ESP32 -.->|heartbeat| NETWORK
    
    classDef flutter fill:#e1f5fe,stroke:#01579b,stroke-width:3px
    classDef grpc fill:#f3e5f5,stroke:#4a148c,stroke-width:2px
    classDef lifecycle fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    classDef standard fill:#fff3e0,stroke:#e65100,stroke-width:2px
    classDef hardware fill:#ffebee,stroke:#b71c1c,stroke-width:2px
    classDef pathfinding fill:#f1f8e9,stroke:#33691e,stroke-width:2px
    
    class FLUTTER,FLUTTER_COMPONENTS flutter
    class GRPC_PORT grpc
    class NETWORK,PATH_GEN,PATH_FOL_VF,STATION_VF,SEARCH_VF,COLLISION_VF,HEAD_CTRL_VF,HEAD_CTRL,AIRMAR,WIND_SMOOTH,ESP32 lifecycle
    class STATE_MGR,HEAD_SELECT,FAKE_MOVE,BUOY_DETECT standard
    class PWM,TRIM_TAB,SENSORS hardware
    class PATHFINDER pathfinding
```

## Node Classification & Responsibilities

### Flutter Application (External Client)
- **Type**: Standalone Flutter/Dart application
- **Communication**: gRPC client to ROS2 system
- **Key Components**:
  - `NetworkComms` (gRPC client wrapper)
  - `AutonomousModeProvider` (Riverpod state management)
  - UI widgets for control and visualization

### ROS2 Lifecycle Nodes (Managed)
These nodes are managed by `state_manager` with coordinated startup/shutdown:

| Node | Primary Function | Key Topics/Services |
|------|------------------|---------------------|
| **network_comms** | gRPC ↔ ROS2 bridge, telemetry aggregation | `single_waypoint`, `autonomous_mode`, `current_path` |
| **path_generator** | Waypoint management, path planning | `waypoints`, `current_path`, GetPath service |
| **path_follower_vf** | Vector field navigation execution | `current_path`, `target_position` |
| **heading_controller_vf** | Fuzzy logic heading control | `target_position`, `rudder_angle` |
| **airmar_reader** | GPS/compass/weather data processing | `airmar_data/*` topics |
| **wind_smoother** | Wind data filtering and smoothing | `true_wind_smoothed`, `apparent_wind_smoothed` |
| **esp32_comms** | Trim tab hardware communication | `tt_control`, `trim_state` |

### ROS2 Standard Nodes
| Node | Type | Function |
|------|------|----------|
| **state_manager** | Node | Lifecycle orchestration, startup sequence |
| **heading_select** | Node | Heading source selection and fusion |
| **buoy_detection** | Node | Computer vision, object detection |
| **fake_movement** | Node | Simulation mode for testing |

### C++ Pathfinding Service
| Node | Language | Function |
|------|----------|----------|
| **pathfinder_node** | C++ | A* pathfinding algorithm, performance-critical |

## Navigation Mode Architecture

The system supports multiple navigation modes through different node combinations:

### 1. **Waypoint Navigation** (`path_test_vf.py`)
```
path_generator → path_follower_vf → heading_controller_vf → PWM
```

### 2. **Station Keeping** (`station_keep.py`)
```
station_keep_vf → heading_controller_vf → PWM
```

### 3. **Search & Rescue** (`search_rescue.py`)
```
search_rescue_vf → heading_controller_vf → PWM
```

### 4. **Collision Avoidance** (`collision_avoidance.py`)
```
collision_avoidance_vf → heading_controller_vf → PWM
```

## Communication Protocols

### 1. **Flutter ↔ ROS2**: gRPC over TCP (Port 50051)
- Commands: `AddWaypointCommand`, `AutonomousModeCommand`, etc.
- Telemetry: `StreamBoatState`, real-time boat status
- Video: `StreamVideo` for camera feeds

### 2. **ROS2 Internal**: Standard ROS2 Topics/Services
- QoS profiles for reliability
- Lifecycle management for coordinated startup
- Service calls for pathfinding

### 3. **Hardware Interface**: 
- **ESP32**: Serial communication for trim tab
- **Airmar**: NMEA 0183 protocol over serial
- **PWM Controller**: Custom protocol for servo control

## System Startup Sequence

1. **state_manager** starts first
2. **Lifecycle nodes** are activated in dependency order:
   - `airmar_reader`, `wind_smoother` (sensors first)
   - `network_comms` (communication bridge)
   - `path_generator` (planning)
   - `path_follower_vf`, `heading_controller_vf` (control)
   - `esp32_comms` (hardware)
3. **Standard nodes** start independently
4. **Flutter app** connects via gRPC when ROS2 system is ready

This architecture provides a clear separation between the external Flutter client (UI/commands) and the ROS2 node ecosystem (control/navigation), with `network_comms` serving as the critical bridge between the two systems.