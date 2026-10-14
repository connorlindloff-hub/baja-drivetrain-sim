Baja SAE Drivetrain Simulation (C++ and MATLAB)

Disclosure of AI usage: Due to my personal lack of familiarity with both github and the required kinimatic equations claude was used to set up this repo and assist with my understanding of the requsite math. No AI generated code is represented in this project.

A time-stepped simulation of a Baja SAE vehicle's drivetrain (engine, CVT, final drive, vehicle dynamics), written in C++, with the results analyzed in MATLAB. The project compares final-drive ratios from 6.0:1 to 9.0:1 and looks at how each one affects acceleration and top speed.

Important: all vehicle and drivetrain parameters are estimates, and the model has not been validated against measured vehicle data. Treat the results as a demonstration of the modeling and analysis workflow and as relative comparisons between configurations, not as predictions of real-world performance.

What it does

The C++ program simulates a straight-line, full-throttle run on flat ground and logs one row of data every 0.01 s for 30 s. It repeats the run for seven final-drive ratios (6.0, 6.5, 7.0, 7.4, 8.0, 8.5, 9.0) and writes the results to CSV files. MATLAB then reads those files and produces the plots.

Model

At each time step the simulation:

Computes wheel speed from vehicle speed and wheel radius.
Sets the CVT ratio. The CVT tries to hold the engine at a target rpm of 3000. The wanted ratio is target engine speed / (wheel speed × final drive), clamped to the CVT's physical range (0.9 to 3.9). Below the engagement speed, the engine is held at 2400 rpm with the CVT at its lowest gear, to represent clutch slip during launch.
Looks up engine torque from a dyno table by linear interpolation. Torque is set to zero at 3600 rpm, the end of the dyno data.
Computes tractive force at the wheels: torque × CVT ratio × final drive × efficiency / wheel radius, capped by a simple traction limit (mu × weight).
Subtracts resistive forces: rolling resistance, aerodynamic drag (proportional to speed squared), and grade (zero by default).
Integrates acceleration into speed and position with a fixed time step (Euler method).
Parameters

All values are estimates unless noted.

Parameter	Value	Source
Vehicle mass (with driver)	295 kg (650 lb)	Estimated
Wheel radius	0.292 m (23 in diameter tire)	Estimated
Rolling resistance coefficient	0.07	Estimated
Drag area (CdA)	1.0 m²	Estimated
Air density	1.2 kg/m³	Standard value
Traction coefficient (mu)	0.6	Estimated
Final drive ratio (swept)	6.0 to 9.0 (7.4 baseline)	Estimated
CVT ratio range	0.9 to 3.9	Estimated
Drivetrain efficiency	0.90	Estimated
Engagement rpm	2400	Estimated
Target rpm	3000	Estimated
Engine torque curve	7 points, 2400 to 3600 rpm	SAE Baja restricted Kohler CH440 net corrected dyno data

Engine torque table used (converted from ft-lb to N·m):

rpm	2400	2600	2800	3000	3200	3400	3600
Torque (N·m)	25.1	24.5	23.6	22.5	20.9	19.7	18.3
Code structure

The C++ is organized as a small set of structs and functions in main.cpp:

Item	Purpose
Params	All simulation inputs, with default values
Row	One logged time step (time, position, speed, rpm, CVT ratio, torque, forces, acceleration)
Summary	Key results for one run (top speed, time to 25 mph, distance at 10 s)
clamp, interpolate	Helper functions (limit a value to a range; linear table lookup)
run_sim	Runs one simulation and returns the logged rows
summarize	Extracts the summary numbers from one run
write_csv, write_summary_csv	Write results to CSV files
ratio_label	Formats a ratio for use in a filename
Build and run
g++ -std=c++17 -Wall -Wextra -o sim torqueSim.cpp
./sim

Note: Project was written using a Linux Ubuntu distro through Visual Studo Code 

On Windows PowerShell, run .\sim.exe instead. The program prints one summary line per ratio and writes these files to the current folder:

run_Gf_6.0.csv through run_Gf_9.0.csv: the full time series for each ratio
summary.csv: one line per ratio with the key results
Results
Final drive	Top speed (mph)	Time to 25 mph (s)	Distance at 10 s (m)
6.0	37.6	4.56	105.0
6.5	37.5	4.56	105.0
7.0	37.5	4.56	105.0
7.4	37.0	4.56	105.0
8.0	34.2	4.56	104.9
8.5	32.2	4.56	104.9
9.0	30.4	4.56	103.9

Observations from this model:

Acceleration is identical across ratios. While the CVT can hold its target rpm, engine speed and torque at a given wheel speed do not depend on the final drive, because the CVT compensates. The launch is also limited by the traction cap, not by gearing.
Ratios from 6.0 to 7.0 reach nearly the same top speed (about 37.5 mph). In this range, top speed is limited by engine power against drag, not by gearing.
Above about 7.4:1, gearing becomes the limit and top speed falls steadily, reaching about 30 mph at 9.0:1.
<!-- TODO: add MATLAB figures once generated. Suggested figures: ![Top speed vs. final drive](figures/top_speed_vs_final_drive.png) ![Speed vs. time for each ratio](figures/speed_vs_time.png) ![Tractive force vs. resistance](figures/force_vs_speed.png) ![CVT ratio and engine rpm](figures/cvt_and_rpm.png) -->
MATLAB analysis

analyze_drivetrain.m reads summary.csv and the run files and generates the figures listed above. Run it from the folder containing the CSV files.

Verification
The torque interpolation was tested against hand-computed values, including points at, between, and beyond the ends of the table.
The first-step force and acceleration were checked by hand against the program's output.
Halving the time step from 0.01 s to 0.005 s changed simulated speeds by less than 0.02 mph, so the results are insensitive to the step size.
Top speeds were compared against hand calculations of the gearing-limited maximum.
Limitations
Parameters are estimated, and the model is not validated against measured vehicle data.
Motion is one-dimensional (straight line, flat ground). There is no cornering, suspension, or weight transfer.
The CVT shifts instantly to its target rpm. Real CVT dynamics, belt slip, and clutch behavior are not modeled.
Rotational inertia of the drivetrain and wheels is ignored.
Traction is a single constant-coefficient cap, with no tire slip model.
The engine governor is modeled as a hard torque cutoff at 3600 rpm, the end of the available dyno data. This can cause tiny oscillations in speed near top speed at coarser time steps.
Drag area, rolling resistance, and traction coefficient are rough guesses and have a large effect on the results.
Possible extensions
Sensitivity analysis: sweep mass, rolling resistance, and traction coefficient and compare their effects on top speed.
Simulate a hill climb (nonzero grade), where lower final-drive ratios should matter more.
Refactor into a class hierarchy (engine, CVT, final drive, vehicle) so components can be swapped independently.
Compare the model against logged data from a real vehicle.
Data source

Engine torque and power data: SAE Baja restricted Kohler CH440 net corrected performance sheet (file dated August 2022)
