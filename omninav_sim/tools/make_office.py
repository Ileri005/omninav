#!/usr/bin/env python3
"""Generate office.sdf (Gazebo Harmonic world) for OMNI-Nav from a few measurements.

Room frame used below (metres), matching JJ's sketch:
  x runs along the room length, from END A (doors 6,7,8) to END B (doors 9,10 + lobby)
  y runs across the room, 0 = left side (Office Rules side), W = right side (kitchenette)
  The open ENTRANCE area sits to the left of the room (negative y), past the Office Rules wall.

The robot spawns at the world origin (0,0). ORIGIN shifts the whole layout so (0,0)
lands in open floor near END A. Edit the numbers in this block and re-run to regenerate.
"""
FOOT = 0.28                    # measured foot length (m)
L = 53 * FOOT                  # room length = 14.84 m
W = 16 * FOOT                  # room width  = 4.48 m
T = 0.10                       # wall thickness
H = 2.0                        # wall height
DOOR = 1.02                    # door width (measured)

X_RULES = 3.5                  # where the Office Rules wall sits along the room
E = 2.5                        # depth of the open entrance area (to the left)
LOBBY_Y0, LOBBY_Y1 = -1.0, 1.9 # lobby opening in the END B wall
LOBBY_LEN = 3.2                # how far the lobby extends past END B

DESK = (1.2, 0.6, 0.75)        # one desk per block (measured 120 x 60 cm)
DESK_CENTRES = [(6.5, 1.9), (6.5, 2.6), (8.3, 1.9), (8.3, 2.6)]

ORIGIN = (1.5, 2.2)            # room point that becomes the world origin (robot spawn)

# ---- model list: (name, x, y, z, yaw, sx, sy, sz) in room frame ----
models = []

def box(name, x0, y0, x1, y1, h=H, z0=0.0):
    """Axis-aligned box from corner (x0,y0) to (x1,y1)."""
    models.append((name, (x0 + x1) / 2, (y0 + y1) / 2, z0 + h / 2,
                   abs(x1 - x0), abs(y1 - y0), h))

def wall_x(name, x, y0, y1):   # wall running along y at fixed x
    box(name, x - T / 2, y0, x + T / 2, y1)

def wall_y(name, y, x0, x1):   # wall running along x at fixed y
    box(name, x0, y - T / 2, x1, y + T / 2)

# Main room
wall_x("wall_end_a", 0.0, 0.0, W)                 # doors 6,7,8 (closed, flush)
wall_y("wall_right", W, 0.0, L)                   # kitchenette side
wall_y("wall_left_upper", 0.0, 0.0, X_RULES)      # door 5 (closed, flush)
wall_x("wall_office_rules", X_RULES, -E, 0.0)
wall_y("wall_entrance_outer", -E, X_RULES, L)     # building entrance treated as closed
wall_x("wall_end_b_left", L, -E, LOBBY_Y0)
wall_x("wall_end_b_right", L, LOBBY_Y1, W)        # doors 9,10 (closed, flush)

# Lobby (doors 1-4, closed, flush)
wall_y("wall_lobby_left", LOBBY_Y0, L, L + LOBBY_LEN)
wall_y("wall_lobby_right", LOBBY_Y1, L, L + LOBBY_LEN)
wall_x("wall_lobby_end", L + LOBBY_LEN, LOBBY_Y0, LOBBY_Y1)

# Furniture (solid to the floor, so the 0.15 m lidar plane sees them)
for i, (cx, cy) in enumerate(DESK_CENTRES, 1):
    box(f"desk_{i}", cx - DESK[0] / 2, cy - DESK[1] / 2,
        cx + DESK[0] / 2, cy + DESK[1] / 2, h=DESK[2])
box("kitchenette_counter", 7.5, W - 0.6, 11.5, W, h=0.9)
box("sofa", 10.0, -E, 12.0, -E + 0.8, h=0.8)
box("plant_1", 1.05, 0.05, 1.35, 0.35, h=0.6)
box("plant_2", 2.65, 0.05, 2.95, 0.35, h=0.6)
# TV on a stand: uncomment and set position once known
# box("tv_stand", x0, y0, x1, y1, h=1.2)

# Extra obstacles for the navigation demo. These are deliberately NOT in the saved map
# (office.pgm was made before they existed), so Nav2 only finds them with the live lidar
# and has to steer around them. Coordinates are WORLD/MAP coordinates (same as RViz).
# name, x, y, size_x, size_y, height
OBSTACLES = [
    ("obst_chair_1",  5.0, -1.10, 0.45, 0.45, 0.9),   # chair pulled out from desk 1
    ("obst_chair_2",  6.8, -1.10, 0.45, 0.45, 0.9),   # chair pulled out from desk 3
    ("obst_bin",      1.5,  1.60, 0.35, 0.35, 0.6),   # bin on the kitchen side near the start
    ("obst_box_1",   10.5, -1.00, 0.60, 0.40, 0.5),   # carton in the open far area
    ("obst_box_2",    3.0, -3.20, 0.50, 0.50, 0.5),   # carton in the entrance area
]
for name, wx, wy, sx, sy, h in OBSTACLES:
    cx, cy = wx + ORIGIN[0], wy + ORIGIN[1]          # world -> room frame
    box(name, cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2, h=h)

# Door locations (for Nav2 goals / ArUco markers), room frame, centre of each door
DOORS = {
    "door_5": (2.0, 0.0), "door_6": (0.0, 0.9), "door_7": (0.0, 2.2), "door_8": (0.0, 3.5),
    "door_9": (L, 3.9), "door_10": (L, 3.0),
    "door_1": (L + 1.0, LOBBY_Y0), "door_4": (L + 1.0, LOBBY_Y1),
    "door_2": (L + LOBBY_LEN, -0.2), "door_3": (L + LOBBY_LEN, 1.1),
}

COLOURS = {"wall": "0.85 0.85 0.82 1", "desk": "0.95 0.95 0.95 1",
           "kitchenette": "0.9 0.88 0.8 1", "sofa": "0.15 0.15 0.15 1",
           "plant": "0.2 0.6 0.25 1", "obst": "0.85 0.4 0.1 1"}

def colour(name):
    for key, c in COLOURS.items():
        if name.startswith(key):
            return c
    return "0.7 0.7 0.7 1"

def model_sdf(name, x, y, z, sx, sy, sz):
    wx, wy = x - ORIGIN[0], y - ORIGIN[1]
    geom = f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"
    c = colour(name)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{wx:.3f} {wy:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry>{geom}</geometry></collision>
        <visual name="visual"><geometry>{geom}</geometry>
          <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material></visual>
      </link>
    </model>"""

def world_sdf():
    body = "".join(model_sdf(*m) for m in models)
    doors = "\n".join(f"    <!-- {n}: x={x - ORIGIN[0]:.2f} y={y - ORIGIN[1]:.2f} -->"
                      for n, (x, y) in DOORS.items())
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="office">
    <!-- Generated by make_office.py: edit the numbers there and re-run, rather than editing here. -->
    <!-- Room {L:.2f} m x {W:.2f} m. Robot spawns at world (0,0). Door centres in world coordinates: -->
{doors}

    <!-- 4 ms physics step (default is 1 ms): 4x less physics work, plenty for a velocity-controlled robot -->
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.8 0.78 0.7 1</ambient><diffuse>0.8 0.78 0.7 1</diffuse></material></visual>
      </link>
    </model>
{body}
  </world>
</sdf>
"""

def preview(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(figsize=(13, 6))
    for name, x, y, z, sx, sy, sz in models:
        fc = "#555" if name.startswith("wall") else "#c9b98f" if "kitchen" in name else \
             "#4a8a4f" if "plant" in name else "#333" if "sofa" in name else \
             "#e07a2e" if name.startswith("obst") else "#8fa8c9"
        ax.add_patch(Rectangle((x - sx / 2 - ORIGIN[0], y - sy / 2 - ORIGIN[1]), sx, sy, fc=fc, ec="black", lw=0.5))
        if not name.startswith("wall"):
            ax.text(x - ORIGIN[0], y - ORIGIN[1], name.replace("_", " "), ha="center", va="center", fontsize=7)
    for n, (x, y) in DOORS.items():
        ax.plot(x - ORIGIN[0], y - ORIGIN[1], "o", color="#d1495b")
        ax.annotate(n.replace("door_", "D"), (x - ORIGIN[0], y - ORIGIN[1]), xytext=(4, 4),
                    textcoords="offset points", fontsize=8, color="#d1495b")
    ax.add_patch(Rectangle((-0.33 / 2, -0.268 / 2), 0.33, 0.268, fc="#f4a261", ec="black"))
    ax.annotate("robot spawn (0,0)", (0, 0), xytext=(0.4, -0.5), fontsize=8, arrowprops=dict(arrowstyle="->"))
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.set_xlabel("world x (m)"); ax.set_ylabel("world y (m)")
    ax.set_title(f"office.sdf top-down: room {L:.2f} m x {W:.2f} m, doors as red dots")
    ax.autoscale_view(); fig.tight_layout(); fig.savefig(path, dpi=130)

if __name__ == "__main__":
    with open("office.sdf", "w") as f:
        f.write(world_sdf())
    preview("office_layout.png")
    print(f"wrote office.sdf ({len(models)} models) and office_layout.png")