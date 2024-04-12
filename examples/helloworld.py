"""Launch Isaac Sim Simulator first."""

from omni.isaac.kit import SimulationApp

# launch omniverse app
config = {"headless": False}
simulation_app = SimulationApp(config)


"""Rest everything follows."""

from omni.isaac.core.simulation_context import SimulationContext

if __name__ == "__main__":
    # get simulation context
    simulation_context = SimulationContext()
    # rest and play simulation
    simulation_context.reset()
    # step simulation
    i = 0
    while i < 1000:
        simulation_context.step()
        i += 1
        if i % 100 == 0:
            print(i)
    # stop simulation
    simulation_context.stop()