import cadquery as cq

# ==========================================
# AeroGrid-Dew System Parameters
# ==========================================
# 1. Module Footprint & Array Geometry
module_width = 1000.0   # mm
module_length = 1000.0  # mm
tilt_angle = 22.0       # degrees

# 2. Pipe Dimensions
pipe_od = 28.0          # mm (Outer Diameter)
pipe_wall = 2.0         # mm (Wall thickness)
pipe_pitch = 70.0       # mm (Center-to-center spacing)
num_pipes = int(module_width / pipe_pitch) # 14 pipes

# 3. Post & Frame Clearances
post_height = 1300.0    # mm
frame_profile = 30.0    # mm (Square tubing size)

# ==========================================
# Part 1: Build the Support Frame & Posts
# ==========================================
# Four vertical corner posts
posts = (cq.Workplane("XY")
         .rect(module_width - frame_profile, module_length - frame_profile, forConstruction=True)
         .vertices()
         .box(frame_profile, frame_profile, post_height)
         .translate((0, 0, post_height/2)))

# Angled top frame holding the pipes (tilted 22 degrees)
top_frame = (cq.Workplane("XY")
             .box(module_width, module_length, frame_profile)
             .rotate((1, 0, 0), (0, 0, 0), tilt_angle)
             .translate((0, 0, post_height + (module_length/2 * 0.3746)))) # 0.3746 is sin(22 deg)

# ==========================================
# Part 2: Generate the Tubular Pipe Array
# ==========================================
# We create a single pipe, then pattern it across the module width
single_pipe = (cq.Workplane("YZ")
               .circle(pipe_od / 2.0)
               .circle((pipe_od / 2.0) - pipe_wall)
               .extrude(module_length)
               .translate((-module_width/2 + (pipe_pitch/2), -module_length/2, 0)))

# Duplicate the pipe 14 times with 70mm pitch
pipe_array = single_pipe
for i in range(1, num_pipes):
    next_pipe = single_pipe.translate((i * pipe_pitch, 0, 0))
    pipe_array = pipe_array.union(next_pipe)

# Rotate the entire pipe array 22 degrees to match the frame
pipe_array_tilted = (pipe_array
                     .rotate((1, 0, 0), (0, 0, 0), tilt_angle)
                     .translate((0, 0, post_height + (module_length/2 * 0.3746) + (frame_profile/2) + (pipe_od/2))))

# ==========================================
# Part 3: Add Lower Collection Gutter
# ==========================================
# A 50mm U-channel at the lower edge of the tilted array
gutter = (cq.Workplane("YZ")
          .move(0, -module_length/2)
          .circle(25.0)
          .circle(23.0) # 2mm wall thickness for gutter
          .extrude(module_width)
          .translate((-module_width/2, 0, 0))
          # Cut the top half off to make a U-channel
          .faces(">Z").box(module_width, 60, 25, combine='cut')
          .rotate((1, 0, 0), (0, 0, 0), tilt_angle)
          .translate((0, 0, post_height - 15))) # Shift down slightly below pipes

# ==========================================
# Assemble and Export
# ==========================================
# Combine all structural parts into a single assembly
aerogrid_assembly = posts.union(top_frame).union(pipe_array_tilted).union(gutter)

# Export the 3D model to a STEP file for AutoCAD / SolidWorks import
cq.exporters.export(aerogrid_assembly, 'AeroGrid_Dew_Module.step')
print("Successfully generated AeroGrid_Dew_Module.step for AutoCAD import.")