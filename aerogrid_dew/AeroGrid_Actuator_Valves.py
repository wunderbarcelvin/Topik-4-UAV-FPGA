import cadquery as cq

# ==============================================================================
# 1. Parametric Dimensions (Aligned with AeroGrid-Dew Canopy Specs)
# ==============================================================================
frame_prof = 30.0        # mm (30x30 mm square frame rail)
rail_length = 340.0      # mm (Covers 3 pipes + side actuator mounts)

pipe_od = 28.0           # mm
pipe_wall = 2.0          # mm
pipe_pitch = 70.0        # mm
pipe_len = 260.0         # mm (Section length for detail assembly)
pipe_z_center = (frame_prof / 2.0) + (pipe_od / 2.0)  # 29.0 mm

# ==============================================================================
# 2. Structural Header Frame Rail & 3 PCN Collector Pipes
# ==============================================================================
header_rail = (
    cq.Workplane("YZ")
    .rect(frame_prof, frame_prof)
    .rect(frame_prof - 4.0, frame_prof - 4.0)
    .extrude(rail_length)
    .translate((-rail_length / 2.0, 0.0, 0.0))
)

pipes = None
for px in [-pipe_pitch, 0.0, pipe_pitch]:
    p = (
        cq.Workplane("XZ")
        .center(px, pipe_z_center)
        .circle(pipe_od / 2.0)
        .circle((pipe_od / 2.0) - pipe_wall)
        .extrude(-pipe_len)  # Extrudes along +Y
    )
    pipes = p if pipes is None else pipes.union(p)

# ==============================================================================
# 3. Thermal Control Actuator: Micro-Rinse Manifold + Nozzles
# ==============================================================================
manifold_y = 22.0
manifold_z = pipe_z_center + (pipe_od / 2.0) + 12.0  # 55.0 mm (12 mm above pipe crown)
manifold_len = 220.0

# Distribution pipe (10 mm OD)
rinse_header = (
    cq.Workplane("YZ")
    .center(manifold_y, manifold_z)
    .circle(5.0)
    .circle(3.5)
    .extrude(manifold_len)
    .translate((-115.0, 0.0, 0.0))
)

# End cap on right side (+X)
end_cap = (
    cq.Workplane("YZ")
    .center(manifold_y, manifold_z)
    .circle(6.0)
    .extrude(5.0)
    .translate((105.0, 0.0, 0.0))
)
rinse_manifold = rinse_header.union(end_cap)

# Three micro-mist spray nozzles aimed directly onto each pipe's top crown
for px in [-pipe_pitch, 0.0, pipe_pitch]:
    nozzle_stem = (
        cq.Workplane("XY")
        .center(px, manifold_y)
        .circle(3.0)
        .extrude(9.0)
        .translate((0.0, 0.0, manifold_z - 9.0))
    )
    nozzle_tip = (
        cq.Workplane("XY")
        .center(px, manifold_y)
        .circle(4.2)
        .extrude(2.5)
        .translate((0.0, 0.0, manifold_z - 10.5))
    )
    rinse_manifold = rinse_manifold.union(nozzle_stem).union(nozzle_tip)

# ==============================================================================
# 4. Thermal Control Actuator: 3.3V Latching Micro-Solenoid Valve
# ==============================================================================
valve_x = -135.0

# Cylindrical valve body
valve_body = (
    cq.Workplane("YZ")
    .center(manifold_y, manifold_z)
    .circle(9.5)
    .extrude(36.0)
    .translate((valve_x - 18.0, 0.0, 0.0))
)

# Inlet hose barb (connects to 15 L elevated clean-water tank)
inlet_barb = (
    cq.Workplane("YZ")
    .center(manifold_y, manifold_z)
    .circle(4.5)
    .circle(3.0)
    .extrude(16.0)
    .translate((valve_x - 34.0, 0.0, 0.0))
)

# Electromagnetic latching coil block mounted atop valve body
solenoid_coil = (
    cq.Workplane("XY")
    .center(valve_x, manifold_y)
    .rect(24.0, 22.0)
    .extrude(22.0)
    .translate((0.0, 0.0, manifold_z + 6.0))
)

solenoid_valve = valve_body.union(inlet_barb).union(solenoid_coil)

# ==============================================================================
# 5. Thermal Feedback Sensor: Snap-Fit Clip + PT100 RTD Probe on Center Pipe
# ==============================================================================
sensor_y = 75.0

# C-shaped snap clip hugging the 28 mm OD center pipe
clip_ring = (
    cq.Workplane("XZ")
    .center(0.0, pipe_z_center)
    .circle(16.2)
    .circle(14.0)
    .extrude(-14.0)
    .translate((0.0, sensor_y, 0.0))
)
# Cut bottom opening so it snaps onto pipe from above
clip_cutout = (
    cq.Workplane("XY")
    .center(0.0, sensor_y + 7.0)
    .rect(22.0, 18.0)
    .extrude(16.0)
    .translate((0.0, 0.0, pipe_z_center - 18.0))
)
sensor_clip = clip_ring.cut(clip_cutout)

# Stainless-steel PT100 cylindrical probe seated on pipe crown
pt100_probe = (
    cq.Workplane("XZ")
    .center(0.0, pipe_z_center + 16.0)
    .circle(2.5)
    .extrude(-26.0)
    .translate((0.0, sensor_y - 6.0, 0.0))
)
rtd_sensor = sensor_clip.union(pt100_probe)

# ==============================================================================
# 6. Airflow Control Actuator: Side Micro-Servo + Wind-Baffle Louver Damper
# ==============================================================================
louver_x = (rail_length / 2.0) + 14.0   # 184.0 mm (Windward edge of module)
louver_z = 36.0
louver_len = 230.0
baffle_tilt_deg = 35.0                  # Active PID throttling angle

# Side mounting bracket on header rail
servo_bracket = (
    cq.Workplane("XY")
    .center((rail_length / 2.0) + 6.0, 35.0)
    .rect(14.0, 32.0)
    .extrude(26.0)
    .translate((0.0, 0.0, 10.0))
)

# Micro-servo housing + spline + linkage arm
servo_body = (
    cq.Workplane("XY")
    .center((rail_length / 2.0) + 12.0, 35.0)
    .rect(14.0, 24.0)
    .extrude(24.0)
    .translate((0.0, 0.0, 16.0))
)
servo_arm = (
    cq.Workplane("XZ")
    .center((rail_length / 2.0) + 14.0, 44.0)
    .rect(5.0, 18.0)
    .extrude(-4.0)
    .translate((0.0, 48.0, 0.0))
)
airflow_servo = servo_bracket.union(servo_body).union(servo_arm)

# Pivot shaft + angled aerodynamic wind-baffle louver blade
pivot_shaft = (
    cq.Workplane("XZ")
    .center(louver_x, louver_z)
    .circle(2.5)
    .extrude(-louver_len)
    .translate((0.0, 25.0, 0.0))
)

louver_blade = (
    cq.Workplane("XY")
    .rect(56.0, louver_len - 10.0)
    .extrude(2.5)
    .translate((0.0, (louver_len / 2.0) + 25.0, -1.25))
    .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), baffle_tilt_deg)
    .translate((louver_x, 0.0, louver_z))
)
airflow_louver = pivot_shaft.union(louver_blade)

# ==============================================================================
# 7. Export Multi-Component STEP Assembly for Autodesk Fusion
# ==============================================================================
assy = cq.Assembly(name="AeroGrid_PID_Actuator_SubAssembly")

assy.add(header_rail, name="01_Header_Frame_Rail", color=cq.Color(0.60, 0.63, 0.67, 1.0))
assy.add(pipes, name="02_PCN_Collector_Pipes", color=cq.Color(0.75, 0.90, 0.95, 0.55))
assy.add(rinse_manifold, name="03_Thermal_Rinse_Manifold_Nozzles", color=cq.Color(0.20, 0.55, 0.88, 1.0))
assy.add(solenoid_valve, name="04_Latching_Micro_Solenoid_Valve", color=cq.Color(0.12, 0.28, 0.58, 1.0))
assy.add(rtd_sensor, name="05_PT100_Pipe_Temp_Sensor", color=cq.Color(0.90, 0.45, 0.15, 1.0))
assy.add(airflow_servo, name="06_Airflow_Micro_Servo", color=cq.Color(0.20, 0.22, 0.25, 1.0))
assy.add(airflow_louver, name="07_Airflow_Wind_Baffle_Louver", color=cq.Color(0.85, 0.88, 0.90, 0.85))

assy.save("AeroGrid_Actuator_Valves_Fusion.step")
print("Successfully exported AeroGrid_Actuator_Valves_Fusion.step for Autodesk Fusion.")