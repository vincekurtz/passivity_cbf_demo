# A simple high-level planner which sets desired pose and gripper state.

from pydrake.all import *
import numpy as np
import tkinter as tk
import ipywidgets as widgets

class SimplePlanner(LeafSystem):
    """ This is a simple system block with no inputs. It simply outpus

        1) A desired end-effector pose [roll;pitch;yaw;x;y;z] (and pose dot)
        2) A desired gripper state (open or closed)
    """
    def __init__(self, frame_id):
        LeafSystem.__init__(self)

        # The target pose must be a class-level variable, since
        # we use it to set geometry info as well
        # Set nominal poses and gripper state
        self.pose_nom = np.array([np.pi,  
                                  0.0,
                                  np.pi/2,
                                  0.2,
                                  0.4,
                                  0.4])
        self.twist_nom = np.zeros(6)

        # Declare Drake input and output ports
        self.DeclareVectorOutputPort(
                "end_effector_setpoint",
                BasicVector(12),
                self.SetEndEffectorOutput)

        self.DeclareAbstractOutputPort(
                "gripper_command",
                lambda : AbstractValue.Make(True),
                self.SetGripperOutput)

        # Geometry output port for visualization
        self.frame_id = frame_id
        fpv = FramePoseVector()
        fpv.set_value(self.frame_id, RigidTransform())

        self.DeclareAbstractOutputPort(
                "target_geometry",
                lambda: AbstractValue.Make(fpv),
                self.SetGeometryOutput)

    def SetEndEffectorOutput(self, context, output):
        if context.get_time() < 5:
            self.pose_nom = np.array([np.pi-0.5,  
                                    0.0,
                                    np.pi/2,
                                    0.1,
                                    0.8,
                                    0.50])
        else:
            self.pose_nom = np.array([np.pi-0.5,  
                                    0.0,
                                    np.pi/2,
                                    0.0,
                                   -0.5,
                                    0.50])

        target_state = np.hstack([self.pose_nom,self.twist_nom])
        output.SetFromVector(target_state)

    def SetGripperOutput(self, context, output):
        gripper_closed = False
        output.set_value(gripper_closed)

    def SetGeometryOutput(self, context, output):
        """
        Send the target pose as a geometry output for visualizing the target pose.
        """
        fpv = output.get_mutable_value()

        X = RigidTransform()
        X.set_rotation(RollPitchYaw(self.pose_nom[:3]))
        X.set_translation(self.pose_nom[3:])

        fpv.set_value(self.frame_id, X)

class JupyterGuiPlanner(SimplePlanner):
    """ 
    This is a simple system block with no inputs. It simply outpus

        1) A desired end-effector pose [roll;pitch;yaw;x;y;z] (and pose dot)
        2) A desired gripper state (open or closed)

    based on user input from a jupyter GUI.
    """
    def __init__(self, frame_id):
        SimplePlanner.__init__(self, frame_id)
        
        # Set up interactive display using ipywidgets
        print("setting up display")
        self.roll = widgets.FloatSlider(
                                value=self.pose_nom[0],
                                min=0,
                                max=2*np.pi,
                                step=0.01,
                                description="Roll",
                                orientation='horizontal',
                                readout=True)
        self.pitch = widgets.FloatSlider(
                                value=self.pose_nom[1],
                                min=-np.pi/2+0.3,    # restrictive pitch limits to 
                                max=np.pi/2-0.3,     # avoid gimbal lock issues
                                step=0.01,
                                description="Pitch",
                                orientation='horizontal',
                                readout=True)
        self.yaw = widgets.FloatSlider(
                                value=self.pose_nom[2],
                                min=0,
                                max=2*np.pi,
                                step=0.01,
                                description="Yaw",
                                orientation='horizontal',
                                readout=True)
        self.x = widgets.FloatSlider(
                                value=self.pose_nom[3],
                                min=-1.0,
                                max=1.0,
                                step=0.01,
                                description="X",
                                orientation='horizontal',
                                readout=True)
        self.y = widgets.FloatSlider(
                                value=self.pose_nom[4],
                                min=-1.0,
                                max=1.0,
                                step=0.01,
                                description="Y",
                                orientation='horizontal',
                                readout=True)
        self.z = widgets.FloatSlider(
                                value=self.pose_nom[5],
                                min=0.0,
                                max=1.0,
                                step=0.01,
                                description="Z",
                                orientation='horizontal',
                                readout=True)

        self.gripper = widgets.ToggleButton(
                                value=False,
                                description="Toggle Gripper")

        # Save sliders in a list so we can display them later
        self.gui = [self.roll, self.pitch, self.yaw, self.x, self.y, self.z, self.gripper]

    def SetEndEffectorOutput(self, context, output):
        self.pose_nom = np.hstack([  # need to update this for geometry output
            self.roll.value,
            self.pitch.value,
            self.yaw.value,
            self.x.value,
            self.y.value,
            self.z.value])
        target_state = np.hstack([self.pose_nom, self.twist_nom])
        output.SetFromVector(target_state)

    def SetGripperOutput(self, context, output):
        output.set_value(self.gripper.value)

class GuiPlanner(SimplePlanner):
    """ 
    This is a simple system block with no inputs. It simply outpus

        1) A desired end-effector pose [roll;pitch;yaw;x;y;z] (and pose dot)
        2) A desired gripper state (open or closed)

    based on user input from a gui.
    """
    def __init__(self, frame_id):
        SimplePlanner.__init__(self, frame_id)

        self.gripper_closed = False

        # Set up interactive window using Tkinter
        self.window = tk.Tk()
        self.window.title("Planner")

        self.DeclarePeriodicPublishNoHandler(0.01, 0.0)   # schedule window updates via self.DoPublish

        self.roll = tk.Scale(self.window, 
                     from_=-2*np.pi, 
                     to=2*np.pi,
                     resolution=-1,
                     label="Roll",
                     length=400,
                     orient=tk.HORIZONTAL)
        self.roll.pack()
        self.roll.set(self.pose_nom[0])

        self.pitch = tk.Scale(self.window, 
                     from_=-np.pi/2+0.3,    # restrictive pitch limits to 
                     to=np.pi/2-0.3,        # avoid gimbal lock issues
                     resolution=-1,
                     label="Pitch",
                     length=400,
                     orient=tk.HORIZONTAL)
        self.pitch.pack()
        self.pitch.set(self.pose_nom[1])

        self.yaw = tk.Scale(self.window, 
                     from_=-2*np.pi, 
                     to=2*np.pi,
                     resolution=-1,
                     label="Yaw",
                     length=400,
                     orient=tk.HORIZONTAL)
        self.yaw.pack()
        self.yaw.set(self.pose_nom[2])

        self.x = tk.Scale(self.window, 
                     from_=-1.0, 
                     to=1.0,
                     resolution=-1,
                     label="X",
                     length=400,
                     orient=tk.HORIZONTAL)
        self.x.pack()
        self.x.set(self.pose_nom[3])

        self.y = tk.Scale(self.window, 
                     from_=-1.0, 
                     to=1.0,
                     resolution=-1,
                     label="Y",
                     length=400,
                     orient=tk.HORIZONTAL)
        self.y.pack()
        self.y.set(self.pose_nom[4])

        self.z = tk.Scale(self.window, 
                     from_=0.0, 
                     to=1.0,
                     resolution=-1,
                     label="Z",
                     length=400,
                     orient=tk.HORIZONTAL)
        self.z.pack()
        self.z.set(self.pose_nom[5])

        self.gripper_button = tk.Button(self.window,
                                    text="Toggle Gripper",
                                    state=tk.NORMAL,
                                    command=self.toggle_gripper_state)
        self.gripper_button.pack()

        self.reset_button = tk.Button(self.window,
                                    text="Reset",
                                    state=tk.NORMAL,
                                    command=self.reset)
        self.reset_button.pack()

    def reset(self):
        self.gripper_closed = False
        self.roll.set(self.pose_nom[0])
        self.pitch.set(self.pose_nom[1])
        self.yaw.set(self.pose_nom[2])
        self.x.set(self.pose_nom[3])
        self.y.set(self.pose_nom[4])
        self.z.set(self.pose_nom[5])

    def toggle_gripper_state(self):
        self.gripper_closed = not self.gripper_closed

    def DoPublish(self, context, output):
        self.window.update_idletasks()
        self.window.update()

    def SetEndEffectorOutput(self, context, output):
        self.pose_nom = np.hstack([  # need to update this for geometry/visualizer output
            self.roll.get(),
            self.pitch.get(),
            self.yaw.get(),
            self.x.get(),
            self.y.get(),
            self.z.get()])
        target_state = np.hstack([self.pose_nom, self.twist_nom])
        output.SetFromVector(target_state)

    def SetGripperOutput(self, context, output):
        output.set_value(self.gripper_closed)
