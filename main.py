import krpc
from time import sleep
from math import sqrt

from Vector import Vector3

def sign(v):
    if v == 0: return 0
    if v > 0: return 1
    if v < 0: return -1

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

'''
drawing.add_line((0, 0, 0), (10, 0, 0), vessel_ref).color = (1, 0, 0) # UP
drawing.add_line((0, 0, 0), (0, 10, 0), vessel_ref).color = (0, 1, 0) # FORWARD
drawing.add_line((0, 0, 0), (0, 0, 10), vessel_ref).color = (0, 0, 1) # RIGHT
'''

# SIZES
#target_max_radius = max([Vector3(i).magnitude() for i in target_vessel.bounding_box(target_vessel.reference_frame)])
#vessel_max_radius = max([Vector3(i).magnitude() for i in vessel.bounding_box(vessel_ref)])

# PARAMS
vx_max = 10
vy_max = 10
vz_max = 10

# INIT
phase = 0
control.forward = 0
control.right = 0
control.up = 0

control.rcs = False

target_vessel.control.sas = True

auto_pilot.target_direction = -Vector3(space_center.transform_direction(stream_target_dir(), vessel_ref, surface_ref))
auto_pilot.wait()

control.rcs = True

while True: 
    sleep(0.05)
    vel = Vector3(stream_vel()) # VESSEL REF
    target_pos = Vector3(stream_target_pos()) # VESSEL REF
    target_dir = Vector3(stream_target_dir()) # VESSEL REF
    mass = stream_mass()
    force = Vector3(stream_rcs_force()[0])

    a_rcs = (force / mass) * 0.1

    if phase == 0:
        delta = target_pos + target_dir * 5
        if delta.magnitude() < 0.1: phase = 1
    elif phase == 1:
        delta = target_pos
        vy_max = .1
    
    vx_target = -a_rcs.x * sqrt(abs(delta.x) / a_rcs.x) * sign(delta.x)
    vy_target = -a_rcs.y * sqrt(abs(delta.y) / a_rcs.y) * sign(delta.y)
    vz_target = -a_rcs.z * sqrt(abs(delta.z) / a_rcs.z) * sign(delta.z)

    # clamp vel target
    vx_target = clamp(vx_target, -vx_max, vx_max)
    vy_target = clamp(vy_target, -vy_max, vy_max)
    vz_target = clamp(vz_target, -vz_max, vz_max)

    control.forward = (vel.y - vy_target) / a_rcs.y
    control.right = (vel.x - vx_target) / a_rcs.x
    control.up = -(vel.z - vz_target) / a_rcs.z

    auto_pilot.target_direction = -Vector3(space_center.transform_direction(target_dir, vessel_ref, surface_ref))

    line_x.end = (delta.x, 0, 0)
    line_y.end = (0, delta.y, 0)
    line_z.end = (0, 0, delta.z)



    #print(delta.x, vx_target)
    #print(delta.y, vy_target)
    #print(delta.z, vz_target)