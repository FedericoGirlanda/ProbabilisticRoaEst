import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.use("WebAgg")
import numpy as np

from pydrake.all import ( DiagramBuilder, Simulator, LogVectorOutput)
from pydrake.examples.acrobot import ( AcrobotPlant)

class DrakeStepSimulator():
    def __init__(self, controller, dt_log):
        # Setup acrobot plant
        builder = DiagramBuilder()
        plant = AcrobotPlant()
        context = plant.CreateDefaultContext()
        parameters = plant.get_mutable_parameters(context)
        plant.SetMitAcrobotParameters(parameters)
        acrobot = builder.AddSystem(plant)

        # Setup step controller
        controller_plant = builder.AddSystem(controller)
        builder.Connect(controller_plant.get_output_port(),
                        acrobot.get_input_port())
        
        # Setup a logger for the acrobot state
        self.dt_log = dt_log
        self.state_logger = LogVectorOutput(plant.get_output_port(0), builder, self.dt_log)
        self.input_logger = LogVectorOutput(plant.get_input_port(0), builder, self.dt_log)

        # Build-up the diagram
        diagram = builder.Build()

        # Set up a simulator to run this diagram
        self.simulator = Simulator(diagram)
        self.simulator.Initialize()

    def step_simulation(self,x0, t0, tf):

        context = self.simulator.get_mutable_context()

        # Set the initial conditions (theta1, theta2, theta1dot, theta2dot)
        context.SetContinuousState(x0)
        context.SetTime(t0)

        self.simulator.AdvanceTo(tf)

        N = int(tf/self.dt_log)
        x_sim = self.state_logger.FindLog(context).data()
        u_sim = self.input_logger.FindLog(context).data()
        t_sim = np.linspace(t0,tf,N+1)

        return t_sim, x_sim, u_sim