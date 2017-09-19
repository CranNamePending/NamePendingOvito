# Load dependencies
import ovito
import ovito.data

# Load the native code module
import ovito.plugins.Mesh

# Load submodules.
from .surface_mesh import SurfaceMesh
from .data_collection import DataCollection

# Inject selected classes into parent module.
ovito.data.SurfaceMesh = SurfaceMesh
ovito.data.__all__ += ['SurfaceMesh']
