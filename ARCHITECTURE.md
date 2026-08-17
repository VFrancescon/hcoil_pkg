# Architecture

This is a two-package ROS2 workspace:

| Package | Contents | Purpose |
|---|---|---|
| `hcoil_interfaces` | `msg/MagField.msg`, `msg/VoltAmp.msg` | Message definitions only. No nodes, no logic. |
| `hcoil_pkg` | C++ nodes, Python scripts, launch files, tests | Everything that actually runs. |

`hcoil_pkg` depends on `hcoil_interfaces`, so `hcoil_interfaces` must build first (colcon handles this automatically from dependency order in `package.xml`).

## Why `hcoil_pkg/hcoil_pkg/`?

The nested `hcoil_pkg/hcoil_pkg/__init__.py` is not a mistake — it's required by `ament_python_install_package(${PROJECT_NAME})` in `CMakeLists.txt`. The outer `hcoil_pkg/` is the ROS2/CMake package root; the inner `hcoil_pkg/` is the Python package directory that gets installed, and ROS2's convention is for it to share the parent package's name. Same reasoning applies to `include/hcoil_pkg/` — C++ headers live under a directory named after the package so `#include "hcoil_pkg/field_node.hpp"` doesn't collide with another package's headers once everything is installed system-wide.

## Data flow

```
publish_field.py / sweep_axis.py / sweep_axis_PRL.py
        │  publish MagField{bx,by,bz} (mT)
        ▼
   topic: magfield
        │
        ▼
    field_node  (src/field_node.cpp)
        │  - clamps each axis to the max-step-per-cycle and max-absolute-field limits
        │  - converts mT → A using per-axis calibration constants
        │  - converts A → V using an empirical current/voltage curve
        │  publishes one VoltAmp{voltage,current} per physical PSU
        ▼
  topics: VI/PSU0, VI/PSU1, VI/PSU2, VI/PSU3, VI/PSU4 (one per supply)
        │
        ▼
   psu_node × 5  (src/psu_node.cpp, one process per physical PSU)
        │  - re-checks voltage/current against 80%-of-rated limits
        │  - if not in debug mode, writes to the physical supply over serial (DxkdpLib)
        ▼
     hardware (PSU → coil)
```

Power on/off is a separate path: any client can call the `/PowerOn<PSU-name>` / `/PowerOff<PSU-name>` (`std_srvs/Trigger`) services directly on a given `psu_node` — it isn't tied to the field topic.

There is no closed-loop feedback: nothing reads back actual current/voltage from the hardware to confirm a setpoint was reached.

## Launch files (`hcoil_pkg/launch/`)

| File | What it does | When to use it |
|---|---|---|
| `field_launch.py` | Includes `psu_array_launch.py`, then starts `field_node`. | **The real entry point.** This is what brings up the full system. |
| `psu_array_launch.py` | Starts all 5 `psu_node` instances (`PSU0`–`PSU4`) with their per-supply calibration/COM-port parameters. | Included by `field_launch.py`. Its `debugMode` argument has no default, so launching it standalone requires passing `debugMode:=...` explicitly. |
| `psu_devel_launch.py` | Starts a single `psu_node` in a `psu_debug` namespace, `debugMode=False` hardcoded. | Manual single-PSU debugging. |
| `single_psu_launch.py` | Starts a single `psu_node` named `PSU2`, `debugMode=False` hardcoded, bound to `/dev/ttyUSB4`. | Manual single-PSU debugging — note this uses a *different* COM port than what `psu_array_launch.py` assigns to `PSU2` (`/dev/ttyUSB2`). These two single-PSU launch files are not interchangeable; check which COM port your hardware is actually on before using either. |

## Scripts (`hcoil_pkg/scripts/`)

| File | Installed via CMakeLists? | Notes |
|---|---|---|
| `publish_field.py` | Yes | Publishes one `MagField` message from `bx`/`by`/`bz` ROS params, then exits. |
| `sweep_axis.py` | Yes | Sweeps one axis from `-abs` to `+abs` mT in 10 steps. **This is the version `ros2 run hcoil_pkg sweep_axis.py` actually runs.** |
| `sweep_axis_PRL.py` | **No** | A separate, corrected variant of `sweep_axis.py` (fixes an axis-comparison bug in the original). It exists in the source tree but is not registered in `CMakeLists.txt`, so it is not currently runnable via `ros2 run`. Kept as-is here; wiring it in (or deciding it should replace `sweep_axis.py`) is a functional decision, not a file-layout one. |

## Safety limits — enforced in two places, two unit systems

- `field_node` checks `bx`/`by`/`bz` in **mT** (max absolute field, max change per message).
- `psu_node` independently checks `voltage`/`current` in **V/A** (80% of each supply's rated values).

Neither node is aware of the other's limit. This is a known architectural gap, not something this document's reorganization changes — see the project history/PR discussion for the fuller writeup of the tradeoffs.
