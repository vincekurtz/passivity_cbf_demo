#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import os
from pydrake.all import *
from reduced_order_model import ReducedOrderModelPlant
from controller import Gen3Controller
from planners import GuiPlanner, SimplePlanner

############## Setup Parameters #################

sim_time = np.inf
dt = 3e-3
target_realtime_rate = 1.0

# Initial joint angles
q0 = np.pi*np.array([-0.1,0.1,0.6,-0.5,0.2,-0.5,0])

# initial end-effector pose
x0 = np.array([np.pi-0.5,  
               0,
               np.pi/2,
               0.2,
               0.3,
               0.4])

# High-level planner
planner = "gui"    # must be "gui" or "simple"

# Type of controller to use. Must be "standard", "constrained" or "ours".
# 
#  - The "unconstrained" method directly applies a standard task-space passivity controller.
#  - The "damped_least_squares" method is the same as above, but with a damped Jacobian
#  - The "constrained" approach attempts to match this controller while enforcing constraints.
#  - "ours" modifies the reference so passivity and constraint satisfaction can be guaranteed.
control_strategy = "ours"

# Type of constraints to apply. Must be "singularity", "joint" or "none"
constraint_type = "singularity"

include_manipuland = False
show_diagram = False
make_plots = True

#################################################

# Find the (local) description file relative to drake
robot_description_path = "./models/gen3_7dof/urdf/GEN3_URDF_V12.urdf"
drake_path = getDrakePath()
robot_description_file = "drake/" + os.path.relpath(robot_description_path, start=drake_path)

# Set up the diagram and MultibodyPlant
builder = DiagramBuilder()
scene_graph = builder.AddSystem(SceneGraph())
plant = builder.AddSystem(MultibodyPlant(time_step=dt))
plant.RegisterAsSourceForSceneGraph(scene_graph)

# Create a "controllable" plant, which has access only to the robot arm and gripper,
# and not any data about other objects in the scene
c_plant = MultibodyPlant(time_step=dt)

# Load the robot arm model from a urdf file
robot_urdf = FindResourceOrThrow(robot_description_file)
gen3 = Parser(plant=plant).AddModelFromFile(robot_urdf,"gen3")
c_gen3 = Parser(plant=c_plant).AddModelFromFile(robot_urdf,"gen3")

# Add a frame to the controller's plant which represents the end-effector position.
X_ee = RigidTransform()
X_ee.set_translation([0.0,0.0,0.13])
ee_frame = FixedOffsetFrame(
                    "end_effector",
                    c_plant.GetFrameByName("end_effector_link"),
                    X_ee, c_gen3)
c_plant.AddFrame(ee_frame)

# Load the gripper model from a urdf file
gripper_file = "drake/" + os.path.relpath("./models/hande_gripper/urdf/robotiq_hande.urdf", start=drake_path)
gripper_urdf = FindResourceOrThrow(gripper_file)
gripper = Parser(plant=plant).AddModelFromFile(gripper_urdf,"gripper")
c_gripper = Parser(plant=c_plant).AddModelFromFile(gripper_urdf,"gripper")

# Fix the gripper to the manipulator arm
X_EE = RigidTransform()
plant.WeldFrames(plant.GetFrameByName("end_effector_link",gen3), plant.GetFrameByName("hande_base_link", gripper), X_EE)
c_plant.WeldFrames(c_plant.GetFrameByName("end_effector_link",c_gen3), c_plant.GetFrameByName("hande_base_link", c_gripper), X_EE)

# Fix the base of the manipulator to the world
plant.WeldFrames(plant.world_frame(),plant.GetFrameByName("base_link",gen3))
c_plant.WeldFrames(c_plant.world_frame(),c_plant.GetFrameByName("base_link",c_gen3))

# Add a flat ground with friction
X_BG = RigidTransform()
surface_friction = CoulombFriction(
        static_friction = 0.7,
        dynamic_friction = 0.1)
plant.RegisterCollisionGeometry(
        plant.world_body(),      # the body for which this object is registered
        X_BG,                    # The fixed pose of the geometry frame G in the body frame B
        HalfSpace(),             # Defines the geometry of the object
        "ground_collision",      # A name
        surface_friction)        # Coulomb friction coefficients
plant.RegisterVisualGeometry(
        plant.world_body(),
        X_BG,
        HalfSpace(),
        "ground_visual",
        np.array([0.5,0.5,0.5,0.0]))    # Color set to be completely transparent

#plant.set_contact_model(ContactModel.kHydroelasticWithFallback)

# Load an object to manipulate
if include_manipuland:
    #manipuland_sdf = FindResourceOrThrow("drake/examples/manipulation_station/models/061_foam_brick.sdf")
    #manipuland_sdf = FindResourceOrThrow("drake/manipulation/models/ycb/sdf/003_cracker_box.sdf")
    #manipuland_sdf = FindResourceOrThrow("drake/manipulation/models/ycb/sdf/009_gelatin_box.sdf")
    manipuland_sdf = "./models/manipulands/peg.sdf"
    manipuland = Parser(plant=plant).AddModelFromFile(manipuland_sdf,"manipuland")

    box_urdf = "./models/manipulands/peg_box.sdf"
    box = Parser(plant=plant).AddModelFromFile(box_urdf,"box")
    X_box = RigidTransform()
    X_box.set_translation(np.array([-0.5,0.5,0]))
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link",box), X_box)

c_plant.Finalize()
plant.Finalize()
assert plant.geometry_source_is_registered()

# Add (relaxed) end-effector target visualization
ee_source = scene_graph.RegisterSource("ee")
ee_frame = GeometryFrame("ee")
scene_graph.RegisterFrame(ee_source, ee_frame)

ee_shape = Mesh(os.path.abspath("./models/hande_gripper/meshes/hand-e_with_fingers.obj"),scale=1e-3)
ee_color = np.array([0.1,0.1,0.1,0.4])

ee_geometry = GeometryInstance(X_ee.inverse(), ee_shape, "ee")
ee_geometry.set_illustration_properties(MakePhongIllustrationProperties(ee_color))
scene_graph.RegisterGeometry(ee_source, ee_frame.id(), ee_geometry)

# Add (non-relaxed) end-effector target visualization
target_source = scene_graph.RegisterSource("target")
target_frame = GeometryFrame("target")
scene_graph.RegisterFrame(target_source, target_frame)

#target_shape = Sphere(0.04)
#X_target = RigidTransform()
target_shape = Mesh(os.path.abspath("./models/hande_gripper/meshes/hand-e_with_fingers.obj"),scale=1e-3)
X_target = X_ee.inverse()
target_color = np.array([1.0,1.0,0.0,0.4])

target_geometry = GeometryInstance(X_target, target_shape, "target")
target_geometry.set_illustration_properties(MakePhongIllustrationProperties(target_color))
scene_graph.RegisterGeometry(target_source, target_frame.id(), target_geometry)

# Create planner block, which determines target end-effector setpoints and gripper state
if planner == "simple":
    rom_planner = builder.AddSystem(SimplePlanner(target_frame.id()))
elif planner == "gui":
    rom_planner = builder.AddSystem(GuiPlanner(target_frame.id()))
else:
    raise ValueError("Invalid planner %s" % planner)
rom_planner.set_name("High-level Planner")

# Create reduced-order model (double integrator)
rom_dof = 6   # degrees of freedom in the reduced-order model
rom = builder.AddSystem(ReducedOrderModelPlant(rom_dof, ee_frame.id()))
rom.set_name("RoM")

# Create PD controller for reduced-order model
rom_ctrl = builder.AddSystem(PidController(kp=2*np.ones(rom_dof), 
                                           ki=np.zeros(rom_dof),
                                           kd=2*np.ones(rom_dof)))
rom_ctrl.set_name("RoM_controller")

# Create whole-body controller
ctrl = Gen3Controller(c_plant,dt,     # we use c_plant, which doesn't include objects in 
        strategy=control_strategy,    # the workspace, for dynamics computations
        constraints=constraint_type)
controller = builder.AddSystem(ctrl)

# Connect blocks in the control diagram
builder.Connect(                                            # planner sends target end-effector
        rom_planner.GetOutputPort("end_effector_setpoint"), # pose to the RoM (PD) controller
        rom_ctrl.get_input_port_desired_state())

builder.Connect(                                        # planner sends gripper commands 
        rom_planner.GetOutputPort("gripper_command"),   # directly to the whole-body controller
        controller.GetInputPort("gripper_command"))

builder.Connect(                                # RoM PD controller sends target end-effector
        rom_ctrl.get_output_port(),             # accelerations to the whole-body controller
        controller.GetInputPort("rom_input"))

builder.Connect(                                    # RoM plant sends RoM state (end-effector
        rom.GetOutputPort("x"),                     # pose and twist) to the PD controller
        rom_ctrl.get_input_port_estimated_state())  # and the whole-body controller
builder.Connect(
        rom.GetOutputPort("x"), 
        controller.GetInputPort("rom_state"))

builder.Connect(                                  # whole-body controller sends torques
        controller.GetOutputPort("arm_torques"),  # to the arm and gripper
        plant.get_actuation_input_port(gen3))
builder.Connect(
        controller.GetOutputPort("gripper_forces"),
        plant.get_actuation_input_port(gripper))

builder.Connect(                                        # whole-body controller sends resolved
        controller.GetOutputPort("resolved_rom_input"), # RoM inputs (consistent with joint
        rom.GetInputPort("u"))                          # limits, etc) to the RoM

builder.Connect(                                # whole-body plant sends arm and gripper
        plant.get_state_output_port(gen3),      # state to the whole-body controller.
        controller.GetInputPort("arm_state"))
builder.Connect(
        plant.get_state_output_port(gripper),
        controller.GetInputPort("gripper_state"))

# Set up the Scene Graph
builder.Connect(
        scene_graph.get_query_output_port(),
        plant.get_geometry_query_input_port())
builder.Connect(
        plant.get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id()))
builder.Connect(
        rom.GetOutputPort("ee_geometry"),
        scene_graph.get_source_pose_port(ee_source))
builder.Connect(
        rom_planner.GetOutputPort("target_geometry"),
        scene_graph.get_source_pose_port(target_source))

# Set up the Visualizer
visualizer_params = DrakeVisualizerParams(role=Role.kIllustration)  # kProximity for collision geometry,
                                                                    # kIllustration for visual geometry
DrakeVisualizer().AddToBuilder(builder=builder, scene_graph=scene_graph, params=visualizer_params)
#ConnectContactResultsToDrakeVisualizer(builder, plant)

# Add loggers
rom_logger = LogOutput(rom.GetOutputPort("x"),builder)
rom_logger.set_name("rom_logger")

plant_logger = LogOutput(controller.GetOutputPort("end_effector"),builder)
plant_logger.set_name("plant_logger")

V_logger = LogOutput(controller.GetOutputPort("storage_function"),builder)
V_logger.set_name("V_logger")

Vdot_logger = LogOutput(controller.GetOutputPort("storage_function_dot"),builder)
Vdot_logger.set_name("Vdot_logger")

err_logger = LogOutput(controller.GetOutputPort("error"),builder)
err_logger.set_name("error_logger")

mu_logger = LogOutput(controller.GetOutputPort("manipulability_index"),builder)
mu_logger.set_name("mu_logger")

# Compile the diagram: no adding control blocks from here on out
diagram = builder.Build()
diagram.set_name("diagram")
diagram_context = diagram.CreateDefaultContext()

# Visualize the diagram
if show_diagram:
    plt.figure()
    plot_system_graphviz(diagram,max_depth=2)
    plt.show()

# Simulator setup
simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(target_realtime_rate)
simulator.set_publish_every_time_step(False)

# Set initial states
plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
plant.SetPositions(plant_context, gen3, q0)   # Manipulator arm
plant.SetVelocities(plant_context, gen3, np.zeros(7))

rom_context = diagram.GetMutableSubsystemContext(rom, diagram_context)
rom_context.SetContinuousState(np.hstack([x0,np.zeros(6,)]))

if include_manipuland:
    plant.SetPositions(plant_context, manipuland, np.array([0.7,0,0.7,0,-0.0,0.5,0.1]))

# Run the simulation
simulator.Initialize()
try:
    simulator.AdvanceTo(sim_time)
except KeyboardInterrupt:
    print("Simulation stopped via KeyboardInterrupt")

# Make some plots
if make_plots:
    t = rom_logger.sample_times()

    ###########################################################
    # Plot manipulability index
    ###########################################################

    plt.figure(figsize=(3,3))
    plt.plot(t[1:],mu_logger.data().T[1:], linewidth='2')
    plt.gca().axhline(y=0.03, linewidth='2', linestyle='--', color="grey")
    plt.ylim(0,0.1)
    plt.xlabel("time (s)")
    plt.ylabel("$\mu(\mathbf{q})$")

    ###########################################################
    # Plot desired and actual end-effector pose
    ###########################################################

    #plt.figure()  # End effector rpy and angular velocity comparison
    #plt.subplot(2,1,1)
    #plt.plot(t, rom_logger.data()[:3,:].T,linewidth='2',linestyle='--')
    #plt.gca().set_prop_cycle(None)  # reset the color cycle
    #plt.plot(t, plant_logger.data()[:3,:].T,linewidth='2')
    ##plt.legend(['RoM - x','RoM - y','RoM - z','Plant - x','Plant - y','Plant - z'])
    #plt.xlabel("time (s)")
    #plt.ylabel("End Effector RPY")

    #plt.subplot(2,1,2)
    #plt.plot(t, rom_logger.data()[6:9,:].T,linewidth='2',linestyle='--')
    #plt.gca().set_prop_cycle(None)  # reset the color cycle
    #plt.plot(t, plant_logger.data()[6:9,:].T,linewidth='2')
    ##plt.legend(['RoM - x','RoM - y','RoM - z','Plant - x','Plant - y','Plant - z'])
    #plt.xlabel("time (s)")
    #plt.ylabel("End Effector Angular Velocity")

    #plt.figure()  # End effector position and velocity comparison
    #plt.subplot(2,1,1)
    #plt.plot(t, rom_logger.data()[3:6,:].T,linewidth='2',linestyle='--')
    #plt.gca().set_prop_cycle(None)  # reset the color cycle
    #plt.plot(t, plant_logger.data()[3:6,:].T,linewidth='2')
    ##plt.legend(['RoM - x','RoM - y','RoM - z','Plant - x','Plant - y','Plant - z'])
    #plt.xlabel("time (s)")
    #plt.ylabel("End Effector Position")

    #plt.subplot(2,1,2)
    #plt.plot(t, rom_logger.data()[9:,:].T,linewidth='2',linestyle='--')
    #plt.gca().set_prop_cycle(None)  # reset the color cycle
    #plt.plot(t, plant_logger.data()[9:,:].T,linewidth='2')
    ##plt.legend(['RoM - x','RoM - y','RoM - z','Plant - x','Plant - y','Plant - z'])
    #plt.xlabel("time (s)")
    #plt.ylabel("End Effector Velocity")

    ###########################################################
    # Plot storage function and output error
    ###########################################################

    plt.figure(figsize=(3,3))
    plt.subplot(2,1,1)
    plt.plot(t[1:], V_logger.data().T[1:], linewidth='2',label="Storage Function")
    #plt.plot(t[1:], err_logger.data().T[1:], linewidth='2',label="Output Error")
    plt.ylabel("$V$")

    plt.subplot(2,1,2)
    plt.plot(t[1:], Vdot_logger.data().T[1:], linewidth='2')
    plt.gca().axhline(0, linewidth='2', linestyle="--", color="grey")
    plt.xlabel("time (s)")
    plt.ylabel("$\dot{V}$")

    plt.show()
