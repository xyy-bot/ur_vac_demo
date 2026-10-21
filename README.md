# UR VAC Demo

A minimal three-axis variable admittance control example. It uses RK4,
holds the initial TCP orientation, and runs on a virtual UR3e in MuJoCo or
a real UR robot. 

## Start with MuJoCo

Use Python 3.8-3.11. From this folder:

```console
python -m pip install -r requirements.txt
python main.py
```

Edit `config.py` and run again. The default is MuJoCo + virtual force + fixed parameters. Change `example`
to `variable_damping` or `variable_stiffness` to try VAC.

The robot window closes when the run ends. Close it early to stop.
After the robot has stopped, force, motion, and parameter plots open.
Blue marks the startup TCP point, green marks the target, and the orange arrow
shows the virtual force input. 

## Reading path

Start with one default MuJoCo run. Then read `config.py`, the numbered steps
in `main.py`, and `admittance.py`, in that order. These three files explain
the settings, experiment loop, and control equation. You can leave the
device and output helpers until you need them.

The control model is `M*a + D*v + K*x = force`: a virtual mass, damper,
and spring on each translation axis. M/D/K are three diagonal entries,
stored as XYZ tuples. Mass is fixed; the VAC examples vary D or K.
Positive force initially accelerates the corresponding positive axis.

### Where to make an experiment change

| File | Responsibility                                                      | When to read or edit it                                     |
| --- |---------------------------------------------------------------------|-------------------------------------------------------------|
| `config.py` | Select environment, force mode, timing, reference offset, and M/D/K | First: change experiment settings here                      |
| `main.py` | Connect modules and execute one control cycle at a time             | Follow the numbered loop to understand the whole experiment |
| `admittance.py` | Integrate the three-axis virtual model using RK4                    | Understand how force produces a compliant motion response   |
| `interaction_force.py` | Provide one force from CSV, a sine wave, or live F/T                | Change the force input                                      |
| `parameter_schedule.py` | Provide desired D/K versus time                                     | Change which parameter varies and when                      |
| `reference_trajectory.py` | Generate nominal position, velocity, and acceleration               | Change the planned motion before adding compliance          |

The supplied parameter schedule changes only X, smoothly between 3 and 5 s.
It is a time schedule, not an adaptive law. A zero `reference_offset` holds
the nominal point; for example `(0.02, 0, 0)` requests a smooth 2 cm nominal
motion along X over `reference_duration` seconds.

### Supporting files: read when needed

| File | Responsibility |
| --- | --- |
| `robot_interface.py` | Read the real UR state, zero/read F/T, send `speedL`, and stop/disconnect |
| `mujoco_interface.py` | Advance a virtual UR3e through Jacobian mapping, joint servo torques, and MuJoCo dynamics |
| `simulation.py` | Open, update, and close the optional MuJoCo viewer; physics lives in `mujoco_interface.py` |
| `passivity.py` | Apply an internal constraint before a stiffness update and check the resulting model step |
| `limits.py` | Validate settings and reject motion commands outside the configured bounds |
| `logger.py` | Write the recorded signals and configuration after the run |
| `plotting.py` | Produce the post-run figures and tracking statistics |
| `__init__.py` | Mark the directory as a Python package |

`assets/` contains the robot model, `data/` contains replay input, and
`outputs/` receives run results. `tests/` is for instructor checks.
No additional module is needed to change a basic experiment.

## Follow one control cycle

```text
interaction_force.py -------- force ---------------> admittance.py (RK4)
parameter_schedule.py -- desired D/K --> parameter constraint --^
                                                    |
                           virtual displacement, velocity, acceleration
                                                    |
reference_trajectory.py -- nominal motion ----------(+)--> target motion
                                                          |
                                       robot_interface.py OR mujoco_interface.py
                                                          |
                                                actual TCP state
                                                          |
                                         record now; save/plot after stopping
```

In `main.py`, each cycle reads force, selects and constrains parameters,
advances RK4, adds the nominal reference, checks motion bounds, sends a
command, and records the result. Force and accepted parameters stay constant
within the four RK4 stages. The virtual state persists between cycles.
The parameter constraint is internal; its model-port accounting is not a
full robot-port passivity guarantee with tracking and filtering.

### Four different meanings of position

All quantities below are XYZ vectors in one fixed control frame: MuJoCo
world frame or UR base frame. Position uses m, velocity m/s, acceleration
m/s^2, and force N.

| Code signal | Meaning |
| --- | --- |
| `robot.reference_position` | Absolute TCP position captured once at startup; the fixed origin |
| `p_ref`, `v_ref`, `a_ref` | Nominal motion from the trajectory generator; `p_ref` is relative to that origin |
| `controller.position`, `.velocity`, `.acceleration` | Virtual compliance response to force, relative to the nominal motion |
| `p_target`, `v_target`, `a_target` | Nominal motion plus the virtual response |
| Logged `actual_position`, `actual_velocity`, `actual_acceleration` | Robot motion; position is relative to startup, velocity is read from the interface, acceleration is estimated |

For example, if `p_ref.x = 0.02 m` and the virtual displacement is
`0.01 m`, the target is `0.03 m` from the startup TCP point. The actual
robot position may differ. The virtual displacement is **not** the measured
tracking error, and the measured state does not reset the RK4 state.

The real interface sends **`v_target = v_ref + controller.velocity` through
`speedL`**. It does not call `moveL` or `moveJ`. The absolute target position
is passed for bookkeeping and excursion checks; there is no additional
translational position-feedback term. `a_target` is recorded for comparison,
not used as the `speedL` acceleration setting. A separate angular correction
holds the initial orientation; this is still a three-axis translational VAC.

MuJoCo uses the same target velocity, mapped into joint motion and motor
torques. It integrates the robot dynamics rather than assigning the target
directly to the actual TCP position. Actual motion can therefore lag the
target in both environments.

## Force source

| `force_mode` | VAC input |
| --- | --- |
| `replay` | Recorded CSV, interpolated at the control rate |
| `virtual` | A simple sinusoidal test force |
| `hil` | Real UR TCP F/T sensor, with a basic low-pass filter |

Replay/virtual force enters VAC only. It is not also applied as a MuJoCo
rigid-body load. HIL uses only the measured force and requires a real robot.

The CSV columns are `time,fx,fy,fz`, in seconds/newtons, with increasing time
starting at zero. A sample is in `data/force_replay.csv`. Relative paths start
at this package folder. A replay ends at the CSV end or `duration`, whichever
comes first; only complete control intervals are executed. Use the proper
control frame: MuJoCo world frame or real UR base frame.

## Real robot


In `config.py`, set `environment="robot"`, `force_mode="hil"`, and `robot_ip`.
Then run `main.py`. Start stationary and without external contact; no initial
move is commanded. HIL calls `zeroFtSensor()` once before reading force and
starting the control loop. Zeroing failure aborts the run. Replay/virtual modes
do not zero the sensor. Configure the TCP, payload, and RTDE access first. Set the displacement
and speed limits for your setup. Ctrl+C, completion, and loop errors attempt
to stop and disconnect. The real interface still needs instructor validation
on hardware; automated tests use mocked RTDE.

`robot_interface.py` implements real RTDE access. `mujoco_interface.py`
implements virtual-robot motion using a simple joint servo. They share the
same VAC loop. The simulated servo is not an identified UR drive, and the
bundled scene has no contact/self-collision experiment.

## Results 

After a run, a new folder under `outputs/` contains:

- `data.csv` and `config.json`: recorded signals and settings.
- `trajectory.png`: 3 x 3 plots; XYZ rows and position/velocity/acceleration
  columns, comparing the nominal reference, reference + VAC, and actual motion.
- `force_parameters.png`: XYZ input force, applied damping, and stiffness.
- `statistics.csv`: peak/RMS force and tracking RMSE, MAE, and peak error.

The statistics are also printed in the terminal. Position errors are in mm;
force is in N. The statistics describe position tracking error; the `norm`
row uses the Euclidean vector norm. Displacements
are relative to the initial TCP point. Force plots show the input used by VAC
(after filtering in HIL). Actual velocity comes from RTDE or the MuJoCo TCP
Jacobian. Actual acceleration is estimated from consecutive actual velocities
using the configured control period; it can be noisy on hardware. Target
acceleration comes from the reference derivatives and the admittance equation.
All saving and plotting occur after the robot stops. The parameter constraint
covers the relative admittance model, not the energy supplied by a moving reference.

Run results are generated locally and excluded from Git. The `outputs/`
folder is created automatically on the first run.

## Instructor checks

From the parent of the `ur_vac_demo` folder, run:

```console
python -m unittest discover -s ur_vac_demo/tests
```

The checks cover RK4, parameter constraints, force input, MuJoCo motion,
reference derivatives, and result statistics. RTDE checks use mocks to verify
HIL zeroing and shutdown without connecting to or moving a real robot.

## Robot assets

The bundled UR3e model includes its source information in
`assets/ur3e/sources.json` and its third-party license in `assets/ur3e/LICENSE`.


