# Load system libraries
import numpy

# Load dependencies
import ovito.modifiers

# Load the native code modules.
import Particles
import ParticlesModify

# Inject modifier classes into parent module.
ovito.modifiers.ColorCodingModifier = ParticlesModify.ColorCodingModifier
ovito.modifiers.AssignColorModifier = ParticlesModify.AssignColorModifier
ovito.modifiers.AmbientOcclusionModifier = ParticlesModify.AmbientOcclusionModifier
ovito.modifiers.DeleteSelectedParticlesModifier = ParticlesModify.DeleteSelectedParticlesModifier
ovito.modifiers.ShowPeriodicImagesModifier = ParticlesModify.ShowPeriodicImagesModifier
ovito.modifiers.WrapPeriodicImagesModifier = ParticlesModify.WrapPeriodicImagesModifier
ovito.modifiers.ComputePropertyModifier = ParticlesModify.ComputePropertyModifier
ovito.modifiers.FreezePropertyModifier = ParticlesModify.FreezePropertyModifier
ovito.modifiers.ClearSelectionModifier = ParticlesModify.ClearSelectionModifier
ovito.modifiers.InvertSelectionModifier = ParticlesModify.InvertSelectionModifier
ovito.modifiers.FreezeSelectionModifier = ParticlesModify.FreezeSelectionModifier
ovito.modifiers.ManualSelectionModifier = ParticlesModify.ManualSelectionModifier
ovito.modifiers.SelectExpressionModifier = ParticlesModify.SelectExpressionModifier
ovito.modifiers.SelectParticleTypeModifier = ParticlesModify.SelectParticleTypeModifier
ovito.modifiers.SliceModifier = ParticlesModify.SliceModifier
ovito.modifiers.AffineTransformationModifier = ParticlesModify.AffineTransformationModifier
ovito.modifiers.BinAndReduceModifier = ParticlesModify.BinAndReduceModifier
ovito.modifiers.StructureIdentificationModifier = ParticlesModify.StructureIdentificationModifier
ovito.modifiers.CommonNeighborAnalysisModifier = ParticlesModify.CommonNeighborAnalysisModifier
ovito.modifiers.BondAngleAnalysisModifier = ParticlesModify.BondAngleAnalysisModifier
ovito.modifiers.CreateBondsModifier = ParticlesModify.CreateBondsModifier
ovito.modifiers.CentroSymmetryModifier = ParticlesModify.CentroSymmetryModifier
ovito.modifiers.ClusterAnalysisModifier = ParticlesModify.ClusterAnalysisModifier
ovito.modifiers.CoordinationNumberModifier = ParticlesModify.CoordinationNumberModifier
ovito.modifiers.CalculateDisplacementsModifier = ParticlesModify.CalculateDisplacementsModifier
ovito.modifiers.HistogramModifier = ParticlesModify.HistogramModifier
ovito.modifiers.ScatterPlotModifier = ParticlesModify.ScatterPlotModifier
ovito.modifiers.AtomicStrainModifier = ParticlesModify.AtomicStrainModifier
ovito.modifiers.WignerSeitzAnalysisModifier = ParticlesModify.WignerSeitzAnalysisModifier
ovito.modifiers.VoronoiAnalysisModifier = ParticlesModify.VoronoiAnalysisModifier

# Implement the 'rdf' attribute of the CoordinationNumberModifier class.
def _CoordinationNumberModifier_rdf(self):
    """
    Returns a NumPy array containing the radial distribution function (RDF) computed by the modifier.    
    The returned array is two-dimensional and consists of the [*r*, *g(r)*] data points of the tabulated *g(r)* function.
    
    Note that accessing this array is only possible after the modifier has computed its results. 
    Thus, you have to call :py:meth:`ovito.ObjectNode.compute` first to ensure that this information is up to date.
    """
    rdfx = numpy.asarray(self.rdf_x)
    rdfy = numpy.asarray(self.rdf_y)
    return numpy.transpose((rdfx,rdfy))
ovito.modifiers.CoordinationNumberModifier.rdf = property(_CoordinationNumberModifier_rdf)