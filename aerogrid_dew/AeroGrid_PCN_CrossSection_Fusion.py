import math
import cadquery as cq

# ==============================================================================
# 1. Exact Pipe & Biphilic Coating Specifications
# ==============================================================================
pipe_od = 28.0             # mm (1 inch nominal)
pipe_wall = 2.0            # mm
r_core_in = (pipe_od / 2.0) - pipe_wall   # 12.0 mm
r_core_out = pipe_od / 2.0                # 14.0 mm

# Visually scaled coating thicknesses for clear CAD rendering (actual: 5-50 µm)
interlayer_thick = 0.35    # mm (Spectral & IR dielectric interlayer)
pcn_thick = 0.35           # mm (Zeta-tuned PCN biphilic layer)

r_inter_out = r_core_out + interlayer_thick   # 14.35 mm
r_pcn_out = r_inter_out + pcn_thick           # 14.70 mm

# Stepped telescoping lengths (allows all layers to be seen simultaneously)
len_core = 120.0           # mm
len_interlayer = 95.0      # mm
len_pcn = 70.0             # mm

# Biphilic pattern geometry (6 symmetric pairs around 360° -> ~22% / 78% ratio)
num_stripes = 6
stripe_width = 3.0         # mm (Hydrophilic nucleation stripe width)
wedge_protrusion = 3.0     # mm (Lateral reach of 10° tapered micro-wedges)
wedge_angle_deg = 10.0     # degrees (Laplace pressure gradient wedge tip)
wedge_base_z = wedge_protrusion / math.tan(math.radians(wedge_angle_deg))  # ~17.01 mm

# ==============================================================================
# 2. Build Layer 1: PMMA / Polycarbonate Core Structural Tube
# ==============================================================================
core_tube = (
    cq.Workplane("XY")
    .circle(r_core_out)
    .circle(r_core_in)
    .extrude(len_core)
)

# ==============================================================================
# 3. Build Layer 2: Spectral & Thermal Management Dielectric Interlayer
# ==============================================================================
spectral_interlayer = (
    cq.Workplane("XY")
    .circle(r_inter_out)
    .circle(r_core_out)
    .extrude(len_interlayer)
)

# ==============================================================================
# 4. Build Layer 3: Conformal Biphilic PCN Stripes (22%) & Runways (78%)
# ==============================================================================
# Full outer cylindrical shell before biphilic patterning
full_pcn_shell = (
    cq.Workplane("XY")
    .circle(r_pcn_out)
    .circle(r_inter_out)
    .extrude(len_pcn)
)

# Create 2D profile of one hydrophilic stripe with 3 periodic 10° tapered micro-wedges
half_w = stripe_width / 2.0
w_y = half_w + wedge_protrusion

stripe_pts = [
    (-half_w, 0.0),
    (half_w, 0.0),
    (half_w, 8.0),
    (w_y, 8.0),                          # Start of 10° wedge 1
    (half_w, 8.0 + wedge_base_z),        # Tapered tip 1
    (half_w, 28.0),
    (w_y, 28.0),                         # Start of 10° wedge 2
    (half_w, 28.0 + wedge_base_z),       # Tapered tip 2
    (half_w, 48.0),
    (w_y, 48.0),                         # Start of 10° wedge 3
    (half_w, 48.0 + wedge_base_z),       # Tapered tip 3
    (half_w, len_pcn),
    (-half_w, len_pcn),
]

# Extrude 2D wedge-stripe radially along +X and intersect with cylindrical shell
wedge_cutter = (
    cq.Workplane("YZ")
    .polyline(stripe_pts)
    .close()
    .extrude(r_pcn_out + 2.0)
)

single_hydrophilic_stripe = full_pcn_shell.intersect(wedge_cutter)

# Pattern 6 hydrophilic stripes evenly around the pipe circumference (every 60°)
hydrophilic_stripes = single_hydrophilic_stripe
for i in range(1, num_stripes):
    rotated_stripe = single_hydrophilic_stripe.rotate((0, 0, 0), (0, 0, 1), i * (360.0 / num_stripes))
    hydrophilic_stripes = hydrophilic_stripes.union(rotated_stripe)

# Subtract hydrophilic stripes from full outer shell -> exact 78% hydrophobic runways
hydrophobic_runways = full_pcn_shell.cut(hydrophilic_stripes)

# ==============================================================================
# 5. Add Representative Coalescing Dew Droplets (Visual Aid for Render)
# ==============================================================================
droplet_positions = [
    # (angle_deg, z_pos, radius_mm)
    (22.0, 16.0, 0.9),
    (26.0, 36.0, 1.3),
    (30.0, 56.0, 1.8),
    (86.0, 22.0, 1.1),
    (90.0, 46.0, 1.6),
]

droplets = None
for ang_deg, z_pos, r_drop in droplet_positions:
    ang_rad = math.radians(ang_deg)
    # Seat droplet slightly into the hydrophobic runway surface (contact angle > 135°)
    r_center = r_pcn_out + (r_drop * 0.65)
    dx = r_center * math.cos(ang_rad)
    dy = r_center * math.sin(ang_rad)
    drop = cq.Workplane("XY").sphere(r_drop).translate((dx, dy, z_pos))
    droplets = drop if droplets is None else droplets.union(drop)

# ==============================================================================
# 6. Export Multi-Component STEP Assembly for Autodesk Fusion
# ==============================================================================
assy = cq.Assembly(name="AeroGrid_PCN_Pipe_CrossSection")

assy.add(
    core_tube,
    name="01_Core_PMMA_Tube_28mmOD",
    color=cq.Color(0.82, 0.92, 0.96, 0.45)  # Transparent clear PMMA
)
assy.add(
    spectral_interlayer,
    name="02_Spectral_IR_Dielectric_Interlayer",
    color=cq.Color(0.95, 0.72, 0.25, 0.85)  # Amber/gold optical filter tint
)
assy.add(
    hydrophilic_stripes,
    name="03A_Hydrophilic_PCN_Stripes_Zeta_Plus30mV",
    color=cq.Color(0.10, 0.48, 0.85, 1.0)   # Deep blue (protonated g-C3N4 + Chitosan/SiO2)
)
assy.add(
    hydrophobic_runways,
    name="03B_Hydrophobic_PDMS_mSiO2_Runways",
    color=cq.Color(0.92, 0.94, 0.96, 1.0)   # Matte white (PDMS + m-SiO2/BaSO4 radiative layer)
)
assy.add(
    droplets,
    name="04_Coalescing_Dew_Droplets",
    color=cq.Color(0.35, 0.78, 0.98, 0.75)  # Translucent water blue
)

assy.save("AeroGrid_PCN_CrossSection_Fusion.step")
print("Generated AeroGrid_PCN_CrossSection_Fusion.step successfully.")