# Load dependencies
import ovito.io

# Load the native module.
from ovito.plugins.StdObj import DataSeriesExporter

# Register export formats.
ovito.io.export_file._formatTable["txt/series"] = DataSeriesExporter
