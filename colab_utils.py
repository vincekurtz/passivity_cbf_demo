import numpy as np
import matplotlib.pyplot as plt
import os
from pydrake.all import *
from reduced_order_model import ReducedOrderModelPlant
from controller import Gen3Controller
from planners import JupyterGuiPlanner

# Ignore warning about invalid halfspace geometry
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

def setup_colab_simulation(controller_type, constraint_type, install_path, zmq_url, include_manipuland=False):
    """
    A convienience function for setting up a simulation on Google Colab. 
    Does basically the same thing as simulate.py, but interfaces with meshcat
    rather than DrakeVisualizer and uses jupyter widgets rather than tkinter
    for a gui. 

    @param controller_type     String indicating why type of controller to use
    @param constraint_type     String indicating what types of contstraints to apply
    @param install_path        Path from where the code is being run to this directory
    @param zmq_url             Reference URL for connecting to meshcat
    @param include_manipuland  (optional) boolean indicating whether to include a simple peg to manipulate in the scene

    """
    ############## Setup Parameters #################

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
                   0.5])

    # Map controller_type and constraint_type strings used in colab to the strings used
    # for differentiating controller and constraint types by the controller
    ctrl_dict = {"Unconstrained":"unconstrained", "Standard":"constrained", "Passivity Guaranteed":"ours"}
    cons_dict = {"Singularity Avoidance":"singularity", "Joint Limits":"joint", "None":"none"}

    # Type of controller to use. Must be "standard", "constrained" or "ours".
    # 
    #  - The "standard" method directly applies a standard task-space passivity controller.
    #  - The "constrained" approach attempts to match this controller while enforcing constraints.
    #  - "ours" modifies the reference so passivity and constraint satisfaction can be guaranteed.
    control_strategy = ctrl_dict[controller_type]

    # Type of constraints to apply. Must be "singularity", "joint" or "none"
    constraint_type = cons_dict[constraint_type]

    #################################################

    # Find the (local) description file relative to drake
    robot_description_path = install_path + "/models/gen3_7dof/urdf/GEN3_URDF_V12.urdf"
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

    # Load the gripper model from a urdf file
    gripper_file = "drake/" + os.path.relpath(install_path + "/models/hande_gripper/urdf/robotiq_hande.urdf", start=drake_path)
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

    # Load an object to manipulate
    if include_manipuland:
        manipuland_sdf = install_path + "/models/manipulands/peg.sdf"
        manipuland = Parser(plant=plant).AddModelFromFile(manipuland_sdf,"manipuland")

    c_plant.Finalize()
    plant.Finalize()
    assert plant.geometry_source_is_registered()

    # Add end-effector visualization
    ee_source = scene_graph.RegisterSource("ee")
    ee_frame = GeometryFrame("ee")
    scene_graph.RegisterFrame(ee_source, ee_frame)

    ee_shape = Mesh(os.path.abspath(install_path + "/models/hande_gripper/meshes/hand-e_with_fingers.obj"),scale=1e-3)
    ee_color = np.array([0.1,0.1,0.1,0.4])
    X_ee = RigidTransform()

    ee_geometry = GeometryInstance(X_ee, ee_shape, "ee")
    ee_geometry.set_illustration_properties(MakePhongIllustrationProperties(ee_color))
    scene_graph.RegisterGeometry(ee_source, ee_frame.id(), ee_geometry)

    # Create planner block, which determines target end-effector setpoints and gripper state
    rom_planner = builder.AddSystem(JupyterGuiPlanner())
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

    # Set up the Visualizer
    meshcat = ConnectMeshcatVisualizer(builder=builder,
                                        zmq_url=zmq_url,
                                        scene_graph=scene_graph,
                                        output_port=scene_graph.get_query_output_port())

    # Compile the diagram: no adding control blocks from here on out
    diagram = builder.Build()
    diagram.set_name("diagram")
    diagram_context = diagram.CreateDefaultContext()

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

    simulator.Initialize()
    print("Ready to Simulate!")

    return simulator, rom_planner.gui

def display_gui(widget_list):
    """
    Display a list of ipywidgets. This allows us to show a display in a different
    place than where we created it.
    """
    [display(widget) for widget in widget_list]

