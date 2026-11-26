import math
import cadquery as cq

# ==========================================
# AeroGrid-Dew Parametric Specifications
# ==========================================
module_width = 1000.0    # mm (X-axis)
module_length = 1000.0   # mm (Sloped canopy length)
tilt_angle = 22.0        # degrees from horizontal
theta = math.radians(tilt_angle)

# Pipe Array Specs
pipe_od = 28.0           # mm (Outer Diameter)
pipe_wall = 2.0          # mm (Wall thickness)
pipe_pitch = 70.0        # mm (Center-to-center spacing)
num_pipes = int(module_width / pipe_pitch)  # 14 pipes

# Frame & Post Specs
front_post_h = 1300.0    # mm (Low-side ground clearance)
frame_prof = 30.0        # mm (30x30 mm square hollow section)

# ==========================================
# Part 1: 4 Vertical Corner Posts (Adjusted for 22° Tilt)
# ==========================================
span_y = (module_length - frame_prof) * math.cos(theta)
rear_post_h = front_post_h + (module_length - frame_prof) * math.sin(theta)
x_offset = (module_width / 2.0) - (frame_prof / 2.0)

post_fl = cq.Workplane("XY").box(frame_prof, frame_prof, front_post_h).translate((-x_offset, frame_prof/2, front_post_h/2))
post_fr = cq.Workplane("XY").box(frame_prof, frame_prof, front_post_h).translate((x_offset, frame_prof/2, front_post_h/2))
post_rl = cq.Workplane("XY").box(frame_prof, frame_prof, rear_post_h).translate((-x_offset, frame_prof/2 + span_y, rear_post_h/2))
post_rr = cq.Workplane("XY").box(frame_prof, frame_prof, rear_post_h).translate((x_offset, frame_prof/2 + span_y, rear_post_h/2))

posts_part = post_fl.union(post_fr).union(post_rl).union(post_rr)

# ==========================================
# Part 2: 22° Tilted Support Frame
# ==========================================
outer_frame = (cq.Workplane("XY")
               .rect(module_width, module_length)
               .rect(module_width - 2 * frame_prof, module_length - 2 * frame_prof)
               .extrude(frame_prof)
               .translate((0, module_length / 2.0, 0))
               .rotate((0, 0, 0), (1, 0, 0), tilt_angle)
               .translate((0, 0, front_post_h)))

# ==========================================
# Part 3: 14 Semi-Transparent PCN Pipes
# ==========================================
start_x = -((num_pipes - 1) * pipe_pitch) / 2.0
pipe_z_local = frame_prof + (pipe_od / 2.0)

pipes_list = []
for i in range(num_pipes):
    px = start_x + i * pipe_pitch
    pipe = (cq.Workplane("XZ")
            .center(px, pipe_z_local)
            .circle(pipe_od / 2.0)
            .circle((pipe_od / 2.0) - pipe_wall)
            .extrude(-module_length)  # Extrude along +Y
            .rotate((0, 0, 0), (1, 0, 0), tilt_angle)
            .translate((0, 0, front_post_h)))
    pipes_list.append(pipe)

# ==========================================
# Part 4: Lower U-Channel Collection Gutter
# ==========================================
gutter_r_out = 25.0
gutter_r_in = 23.0

gutter_part = (cq.Workplane("YZ")
               .circle(gutter_r_out)
               .circle(gutter_r_in)
               .extrude(module_width)
               .translate((-module_width / 2.0, 0, 0))
               # Cut top half to form an open U-channel
               .cut(cq.Workplane("XY").box(module_width + 10, gutter_r_out * 3, gutter_r_out * 2).translate((0, 0, gutter_r_out)))
               .translate((0, -15.0, front_post_h - 15.0)))

# ==========================================
# Assemble as Multi-Component STEP for Autodesk Fusion
# ==========================================
assy = cq.Assembly(name="AeroGrid_Dew_1m2_Module")

assy.add(posts_part, name="Support_Posts", color=cq.Color(0.55, 0.58, 0.62, 1.0))
assy.add(outer_frame, name="Canopy_Frame_22deg", color=cq.Color(0.75, 0.78, 0.80, 1.0))
assy.add(gutter_part, name="Collection_Gutter", color=cq.Color(0.20, 0.45, 0.70, 1.0))

for idx, pipe_obj in enumerate(pipes_list, start=1):
    assy.add(
        pipe_obj,
        name=f"PCN_Coated_Pipe_{idx:02d}",
        color=cq.Color(0.75, 0.90, 0.95, 0.55)  # Translucent polycarbonate tint
    )

assy.save("AeroGrid_Dew_Fusion.step")
print("Exported AeroGrid_Dew_Fusion.step with separate components for Autodesk Fusion.")