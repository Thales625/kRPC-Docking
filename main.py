import krpc
from time import sleep
from math import sqrt

from PyVecs import Vector3

from PhaseController import PhaseController

def sign(v):
    if v >= 0: return 1
    return -1

def clamp(v, min, max):
    if v > max: return max
    if v < min: return min
    return v

conn = krpc.connect("Docking")
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
control = vessel.control
auto_pilot = vessel.auto_pilot
drawing = conn.drawing

surface_ref = vessel.surface_reference_frame
vessel_ref = vessel.reference_frame

vessel_dp = vessel.parts.controlling.docking_port
if vessel_dp is None: raise Exception("Set control to the Docking Port!")

target_vessel = space_center.target_vessel
if target_vessel is None:
    target_dp = space_center.target_docking_port
    target_vessel = target_dp.part.vessel
else:
    target_dp = target_vessel.parts.docking_ports[0]

if target_dp is None: raise Exception("Select a target!")

# AUTO PILOT
auto_pilot.engage()
auto_pilot.reference_frame = surface_ref
auto_pilot.target_roll = 0
auto_pilot.stopping_time = (.2, .2, .2)
auto_pilot.deceleration_time = (5, 5, 5)

# STREAMS
stream_vel = conn.add_stream(target_dp.part.velocity, vessel_ref)
stream_target_pos = conn.add_stream(target_dp.position, vessel_ref)
stream_target_dir = conn.add_stream(target_dp.direction, vessel_ref)

stream_rcs_force = conn.add_stream(getattr, vessel, "available_rcs_force")
stream_mass = conn.add_stream(getattr, vessel, "mass")

# LINES
line_x = drawing.add_line((0, 0, 0), (0, 0, 0), vessel_ref)
line_y = drawing.add_line((0, 0, 0), (0, 0, 0), vessel_ref)
line_z = drawing.add_line((0, 0, 0), (0, 0, 0), vessel_ref)

line_x.color = (1, 0, 0)
line_y.color = (0, 1, 0)
line_z.color = (0, 0, 1)

# SIZES
target_max_radius = max([Vector3(i).magnitude() for i in target_vessel.bounding_box(target_vessel.reference_frame)])
vessel_max_radius = max([Vector3(i).magnitude() for i in vessel.bounding_box(vessel_ref)])
sum_radius = target_max_radius + vessel_max_radius + 1

# PARAMS
vx_max = 10
vy_max = 10
vz_max = 10
docking_dist = 2

# DP POS
vessel_dp_pos = Vector3(vessel_dp.position(vessel_ref))

# PHASE CONTROLLER
def wait_aim():
    control.rcs = False

    while abs(auto_pilot.error) > 1:
        print(Vector3(vessel.angular_velocity(surface_ref)).magnitude(), auto_pilot.error)
        auto_pilot.target_direction = -Vector3(space_center.transform_direction(stream_target_dir(), vessel_ref, surface_ref))
        sleep(0.5)

    control.rcs = True

    sleep(0.5)

    phase_controller.next_phase()

def aproach_transition():
    global target_pos, target_dir
    
    if (-target_pos.normalize()).dot(target_dir) > 0:
        phase_controller.next_phase()
        return

def aproach():
    global delta
    delta = target_pos - sum_radius * Vector3(delta.x, 0, delta.z).normalize()

    print("APROACH")

    if delta.magnitude() < 0.5:
        phase_controller.next_phase()

def correct_y():
    global delta, target_dir

    print("CORRECT")

    delta.y += target_dir.y * docking_dist
    delta.x = 0
    delta.z = 0
    
    if abs(delta.y) < 0.5:
        phase_controller.next_phase()

def go_target():
    global delta, target_dir

    print("GO_TARGET")
    
    delta += target_dir * docking_dist

    if delta.magnitude() < 0.1:
        phase_controller.next_phase()

def docking():
    global vy_max

    print("DOCKING")

    vy_max = .1

phase_controller = PhaseController([(wait_aim, None), (aproach, aproach_transition), (correct_y, None), (go_target, None), (docking, None)]) # STAGES: aproach, go_target, docking

# CONST
sqrt_two = sqrt(2)

# INIT
control.forward = 0
control.right = 0
control.up = 0

target_vessel.control.sas = True

while True:
    sleep(0.05)

    # get streams
    try:
        vel = Vector3(stream_vel()) # VESSEL REF
        target_pos = Vector3(stream_target_pos()) # VESSEL REF
        target_dir = Vector3(stream_target_dir()) # VESSEL REF
        force_rcs = Vector3(stream_rcs_force()[0]) # VESSEL REF
        mass = stream_mass()
    
        a_rcs = (force_rcs / mass) * 0.1

        delta = target_pos - vessel_dp_pos

        phase_controller.loop()

        # vel target
        vx_target = (-sqrt_two * sqrt(a_rcs.x) * (delta.x / sqrt(abs(delta.x)))) if delta.x != 0 else 0
        vy_target = (-sqrt_two * sqrt(a_rcs.y) * (delta.y / sqrt(abs(delta.y)))) if delta.y != 0 else 0
        vz_target = (-sqrt_two * sqrt(a_rcs.z) * (delta.z / sqrt(abs(delta.z)))) if delta.z != 0 else 0

        # clamp vel target
        vx_target = clamp(vx_target, -vx_max, vx_max)
        vy_target = clamp(vy_target, -vy_max, vy_max)
        vz_target = clamp(vz_target, -vz_max, vz_max)

        # rcs control
        control.forward =  (vel.y - vy_target) / a_rcs.y
        control.right   =  (vel.x - vx_target) / a_rcs.x
        control.up      = -(vel.z - vz_target) / a_rcs.z # -BOTTOM

        # auto pilot aim
        auto_pilot.target_direction = -Vector3(space_center.transform_direction(target_dir, vessel_ref, surface_ref))

        # lines draw
        line_x.end = (delta.x, 0, 0)
        line_y.end = (0, delta.y, 0)
        line_z.end = (0, 0, delta.z)
    except ValueError as e:
        print("End program")
        break