"""
This module contains all modifiers available in OVITO. See :ref:`this introduction page <modifiers_overview>` to learn more
about modifiers and the data pipeline system of OVITO.

The abstract base class of all modifier types is :py:class:`~ovito.pipeline.Modifier`.
Typically, you create a modifier instance, configure its parameters and insert it into a
data :py:class:`~ovito.pipeline.Pipeline` as follows::

    mod = AssignColorModifier()
    mod.color = (0.2, 1.0, 0.9)
    pipeline.modifiers.append(mod)

The following modifier types are available. Please consult the :ovitoman:`OVITO user manual <../../particles.modifiers>` for a
more in-depth description of what these modifiers do.

============================================== =========================================
Python class name                              User interface name
============================================== =========================================
:py:class:`AcklandJonesModifier`               :guilabel:`Ackland-Jones analysis`
:py:class:`AffineTransformationModifier`       :guilabel:`Affine transformation`
:py:class:`AmbientOcclusionModifier`           :guilabel:`Ambient occlusion`
:py:class:`AssignColorModifier`                :guilabel:`Assign color`
:py:class:`AtomicStrainModifier`               :guilabel:`Atomic strain`
:py:class:`CalculateDisplacementsModifier`     :guilabel:`Displacement vectors`
:py:class:`CentroSymmetryModifier`             :guilabel:`Centrosymmetry parameter`
:py:class:`ChillPlusModifier`                  :guilabel:`Chill+`
:py:class:`ClearSelectionModifier`             :guilabel:`Clear selection`
:py:class:`ClusterAnalysisModifier`            :guilabel:`Cluster analysis`
:py:class:`ColorCodingModifier`                :guilabel:`Color coding`
:py:class:`CombineDatasetsModifier`            :guilabel:`Combine datasets`
:py:class:`CommonNeighborAnalysisModifier`     :guilabel:`Common neighbor analysis`
:py:class:`ComputePropertyModifier`            :guilabel:`Compute property`
:py:class:`ConstructSurfaceModifier`           :guilabel:`Construct surface mesh`
:py:class:`CoordinationAnalysisModifier`       :guilabel:`Coordination analysis`
:py:class:`CoordinationPolyhedraModifier`      :guilabel:`Coordination polyhedra`
:py:class:`CreateBondsModifier`                :guilabel:`Create bonds`
:py:class:`CreateIsosurfaceModifier`           :guilabel:`Create isosurface`
:py:class:`DeleteSelectedModifier`             :guilabel:`Delete selected`
:py:class:`DislocationAnalysisModifier`        :guilabel:`Dislocation analysis (DXA)`
:py:class:`ElasticStrainModifier`              :guilabel:`Elastic strain calculation`
:py:class:`ExpandSelectionModifier`            :guilabel:`Expand selection`
:py:class:`ExpressionSelectionModifier`        :guilabel:`Expression selection`
:py:class:`FreezePropertyModifier`             :guilabel:`Freeze property`
:py:class:`GenerateTrajectoryLinesModifier`    :guilabel:`Generate trajectory lines`
:py:class:`HistogramModifier`                  :guilabel:`Histogram`
:py:class:`IdentifyDiamondModifier`            :guilabel:`Identify diamond structure`
:py:class:`InterpolateTrajectoryModifier`      :guilabel:`Interpolate trajectory`
:py:class:`InvertSelectionModifier`            :guilabel:`Invert selection`
:py:class:`LoadTrajectoryModifier`             :guilabel:`Load trajectory`
:py:class:`ManualSelectionModifier`            :guilabel:`Manual selection`
:py:class:`PolyhedralTemplateMatchingModifier` :guilabel:`Polyhedral template matching`
:py:class:`PythonScriptModifier`               :guilabel:`Python script`
:py:class:`ReplicateModifier`                  :guilabel:`Replicate`
:py:class:`ScatterPlotModifier`                :guilabel:`Scatter plot`
:py:class:`SelectTypeModifier`                 :guilabel:`Select type`
:py:class:`SliceModifier`                      :guilabel:`Slice`
:py:class:`SpatialBinningModifier`             :guilabel:`Spatial binning`
:py:class:`SpatialCorrelationFunctionModifier` :guilabel:`Spatial correlation function`
:py:class:`VoronoiAnalysisModifier`            :guilabel:`Voronoi analysis`
:py:class:`VoroTopModifier`                    :guilabel:`VoroTop analysis`
:py:class:`WignerSeitzAnalysisModifier`        :guilabel:`Wigner-Seitz defect analysis`
:py:class:`WrapPeriodicImagesModifier`         :guilabel:`Wrap at periodic boundaries`
============================================== =========================================

*Note that some analysis modifiers of the graphical version of OVITO are missing in the list above and are not accessible from Python.
That is because they perform computations that can be achieved equally well using the Numpy module
in a more straightforward manner.*

"""

# Load the native module.
from ..plugins.PyScript import PythonScriptModifier

__all__ = ['PythonScriptModifier']