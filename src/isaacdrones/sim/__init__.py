"""The only package that imports Isaac Sim (`isaacsim`, `omni`, `pxr`).

The rest of isaacdrones goes through `Simulation`. Importing this package does not
import Isaac Sim; creating a `Simulation` does.
"""

from isaacdrones.sim.simulation import Simulation

__all__ = ["Simulation"]
