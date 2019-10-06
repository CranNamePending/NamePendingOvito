///////////////////////////////////////////////////////////////////////////////
//
//  Copyright (2018) Alexander Stukowski
//
//  This file is part of OVITO (Open Visualization Tool).
//
//  OVITO is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  OVITO is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
///////////////////////////////////////////////////////////////////////////////

#include <ovito/crystalanalysis/CrystalAnalysis.h>
#include <ovito/crystalanalysis/modifier/dxa/DislocationAnalysisModifier.h>
#include <ovito/crystalanalysis/modifier/dxa/StructureAnalysis.h>
#include <ovito/crystalanalysis/modifier/elasticstrain/ElasticStrainModifier.h>
#include <ovito/crystalanalysis/objects/DislocationVis.h>
#include <ovito/crystalanalysis/objects/DislocationNetworkObject.h>
#include <ovito/crystalanalysis/objects/ClusterGraphObject.h>
#include <ovito/crystalanalysis/objects/BurgersVectorFamily.h>
#include <ovito/crystalanalysis/objects/Microstructure.h>
#include <ovito/crystalanalysis/importer/CAImporter.h>
#include <ovito/crystalanalysis/importer/DislocImporter.h>
#include <ovito/crystalanalysis/importer/ParaDiSImporter.h>
#include <ovito/crystalanalysis/exporter/CAExporter.h>
#include <ovito/crystalanalysis/exporter/VTKDislocationsExporter.h>
#include <ovito/pyscript/binding/PythonBinding.h>
#include <ovito/particles/scripting/PythonBinding.h>
#include <ovito/core/app/PluginManager.h>

namespace Ovito { namespace CrystalAnalysis {

using namespace PyScript;

PYBIND11_MODULE(CrystalAnalysisPython, m)
{
	// Register the classes of this plugin with the global PluginManager.
	PluginManager::instance().registerLoadedPluginClasses();

	py::options options;
	options.disable_function_signatures();

	auto DislocationAnalysisModifier_py = ovito_class<DislocationAnalysisModifier, StructureIdentificationModifier>(m,
			":Base class: :py:class:`ovito.pipeline.Modifier`\n\n"
			"This analysis modifier extracts all dislocations in a crystal and converts them to continuous line segments. "
			"The computational method behind this is called *Dislocation Extraction Algorithm* (DXA) and is described "
			"in the paper `[MSMSE 20 (2012), 085007] <http://stacks.iop.org/0965-0393/20/085007>`__. "
			"See also the corresponding :ovitoman:`user manual page <../../particles.modifiers.dislocation_analysis>` for this modifier. "
			"\n\n"
			"The extracted dislocation lines are output as a :py:class:`~ovito.data.DislocationNetwork` object by the modifier "
			"and can be accessed through the :py:attr:`DataCollection.dislocations <ovito.data.DataCollection.dislocations>` field "
			"after the modification pipeline has been evaluated. This is demonstrated in the example script below. "
			"\n\n"
			"Furthermore, you can use the :py:func:`~ovito.io.export_file` function to write the dislocation lines "
			"to a so-called CA file. The CA file format is described in the documentation section of the OVITO user manual for the "
			"Dislocation Analysis modifier."
			"\n\n"
			"**Modifier outputs:**"
			"\n\n"
			" * :py:attr:`DataCollection.dislocations <ovito.data.DataCollection.dislocations>` (:py:class:`~ovito.data.DislocationNetwork`):\n"
			"   This property of the output data collection provides access to the dislocation lines found by the modifier.\n"
			"   See the example script below.\n"
			" * ``DislocationAnalysis.total_line_length`` (:py:attr:`attribute <ovito.data.DataCollection.attributes>`):\n"
			"   The total length of all dislocation lines found by the DXA.\n"
			" * ``DislocationAnalysis.length.1/n<ijk>`` (:py:attr:`attribute <ovito.data.DataCollection.attributes>`):\n"
			"   The modifier outputs a set of attributes that indicate the length of dislocations broken down by Burgers vector type. "
			"   For example, the attribute ``DislocationAnalysis.length.1/6<112>`` specifies the total amount of Shockley partials found by the DXA.\n"
			" * ``DislocationAnalysis.length.other`` (:py:attr:`attribute <ovito.data.DataCollection.attributes>`):\n"
			"   The length of dislocation lines with an unusual Burgers vector that do not belong to any of the predefined standard dislocation types.\n"
			" * ``DislocationAnalysis.cell_volume`` (:py:attr:`attribute <ovito.data.DataCollection.attributes>`):\n"
			"   The volume of the simulation cell. This is output for convenience to enable the calculation of dislocation densities from the line length.\n"
			" * ``DislocationAnalysis.counts.OTHER`` (:py:attr:`attribute <ovito.data.DataCollection.attributes>`):\n"
			"   The number of particles not matching any of the known structure types.\n"
			" * ``DislocationAnalysis.counts.FCC`` (:py:attr:`attribute <ovito.data.DataCollection.attributes>`):\n"
			"   The number of particles with local FCC structure.\n"
			" * ``DislocationAnalysis.counts.HCP`` (:py:attr:`attribute <ovito.data.DataCollection.attributes>`):\n"
			"   The number of particles with local HCP structure.\n"
			" * ``DislocationAnalysis.counts.BCC`` (:py:attr:`attribute <ovito.data.DataCollection.attributes>`):\n"
			"   The number of particles with local BCC structure.\n"
			" * ``DislocationAnalysis.counts.CubicDiamond`` (:py:attr:`attribute <ovito.data.DataCollection.attributes>`):\n"
			"   The number of particles with local cubic diamond structure.\n"
			" * ``DislocationAnalysis.counts.HexagonalDiamond`` (:py:attr:`attribute <ovito.data.DataCollection.attributes>`):\n"
			"   The number of particles with local hexagonal diamond structure.\n"
			"\n\n"
			"Example:\n\n"
			".. literalinclude:: ../example_snippets/dislocation_analysis_modifier.py\n"
			"   :lines: 4-\n"
			)
		.def_property("trial_circuit_length", &DislocationAnalysisModifier::maxTrialCircuitSize, &DislocationAnalysisModifier::setMaxTrialCircuitSize,
				"The maximum length of trial Burgers circuits constructed by the DXA to discover dislocations. "
				"The length is specified in terms of the number of atom-to-atom steps."
				"\n\n"
				":Default: 14\n")
		.def_property("circuit_stretchability", &DislocationAnalysisModifier::circuitStretchability, &DislocationAnalysisModifier::setCircuitStretchability,
				"The number of steps by which a Burgers circuit can stretch while it is being advanced along a dislocation line."
				"\n\n"
				":Default: 9\n")
		.def_property("input_crystal_structure", &DislocationAnalysisModifier::inputCrystalStructure, &DislocationAnalysisModifier::setInputCrystalStructure,
				"The type of crystal to analyze. Must be one of: "
				"\n\n"
				"  * ``DislocationAnalysisModifier.Lattice.FCC``\n"
				"  * ``DislocationAnalysisModifier.Lattice.HCP``\n"
				"  * ``DislocationAnalysisModifier.Lattice.BCC``\n"
				"  * ``DislocationAnalysisModifier.Lattice.CubicDiamond``\n"
				"  * ``DislocationAnalysisModifier.Lattice.HexagonalDiamond``\n"
				"\n\n"
				":Default: ``DislocationAnalysisModifier.Lattice.FCC``\n")
		.def_property("line_smoothing_enabled", &DislocationAnalysisModifier::lineSmoothingEnabled, &DislocationAnalysisModifier::setLineSmoothingEnabled,
				"Flag that enables the smoothing of extracted dislocation lines after they have been coarsened."
				"\n\n"
				":Default: True\n")
		.def_property("line_coarsening_enabled", &DislocationAnalysisModifier::lineCoarseningEnabled, &DislocationAnalysisModifier::setLineCoarseningEnabled,
				"Flag that enables the coarsening of extracted dislocation lines, which reduces the number of sample points along the lines."
				"\n\n"
				":Default: True\n")
		.def_property("line_smoothing_level", &DislocationAnalysisModifier::lineSmoothingLevel, &DislocationAnalysisModifier::setLineSmoothingLevel,
				"The number of iterations of the line smoothing algorithm to perform."
				"\n\n"
				":Default: 1\n")
		.def_property("line_point_separation", &DislocationAnalysisModifier::linePointInterval, &DislocationAnalysisModifier::setLinePointInterval,
				"Sets the desired distance between successive sample points along the dislocation lines, measured in multiples of the interatomic spacing. "
				"This parameter controls the amount of coarsening performed during post-processing of dislocation lines."
				"\n\n"
				":Default: 2.5\n")
		.def_property("defect_mesh_smoothing_level", &DislocationAnalysisModifier::defectMeshSmoothingLevel, &DislocationAnalysisModifier::setDefectMeshSmoothingLevel,
				"Specifies the number of iterations of the surface smoothing algorithm to perform when post-processing the extracted defect mesh."
				"\n\n"
				":Default: 8\n")
		.def_property("only_perfect_dislocations", &DislocationAnalysisModifier::onlyPerfectDislocations, &DislocationAnalysisModifier::setOnlyPerfectDislocations,
				"This flag controls whether the algorithm should extract only perfect dislocations (and no partial dislocations, which is normally done for FCC/HCP and diamond lattices). "
				"Make sure you set the :py:attr:`.circuit_stretchability` parameter to a high value when activating this option, because "
				"large Burgers circuits are needed to identify dissociated dislocations with a wide core. "
				"\n\n"
				":Default: False\n")
		.def_property("color_by_type", &DislocationAnalysisModifier::colorByType, &DislocationAnalysisModifier::setColorByType,
				"Controls whether the modifier assigns a color to each particle based on the identified structure type. "
				"\n\n"
				":Default: ``True``\n")
		.def_property("only_selected", &DislocationAnalysisModifier::onlySelectedParticles, &DislocationAnalysisModifier::setOnlySelectedParticles,
				"Lets the modifier perform the analysis only for selected particles. Particles that are not selected will be treated as if they did not exist."
				"\n\n"
				":Default: ``False``\n")
		.def_property("disloc_vis", &DislocationAnalysisModifier::dislocationVis, &DislocationAnalysisModifier::setDislocationVis,
				"The :py:class:`~ovito.vis.DislocationVis` element controlling the visual representation of the generated dislocation lines.\n")
		.def_property("defect_vis", &DislocationAnalysisModifier::defectMeshVis, &DislocationAnalysisModifier::setDefectMeshVis,
				"The :py:class:`~ovito.vis.SurfaceMeshVis` element controlling the visual representation of the generated defect mesh.\n")
		.def_property("output_interface_mesh", &DislocationAnalysisModifier::outputInterfaceMesh, &DislocationAnalysisModifier::setOutputInterfaceMesh)
	;

	ovito_enum<StructureAnalysis::LatticeStructureType>(DislocationAnalysisModifier_py, "Lattice")
		.value("Other", StructureAnalysis::LATTICE_OTHER)
		.value("FCC", StructureAnalysis::LATTICE_FCC)
		.value("HCP", StructureAnalysis::LATTICE_HCP)
		.value("BCC", StructureAnalysis::LATTICE_BCC)
		.value("CubicDiamond", StructureAnalysis::LATTICE_CUBIC_DIAMOND)
		.value("HexagonalDiamond", StructureAnalysis::LATTICE_HEX_DIAMOND)
	;

	ovito_class<ElasticStrainModifier, StructureIdentificationModifier>(m,
			":Base class: :py:class:`ovito.pipeline.Modifier`\n\n"
			"This modifier computes the atomic-level elastic strain and deformation gradient tensors in crystalline systems. "
			"See also the corresponding :ovitoman:`user manual page <../../particles.modifiers.elastic_strain>` for this modifier. "
			"\n\n"
			"The modifier first performs an identification of the local crystal structure and stores the results in the ``Structure Type`` particle "
			"property. Possible structure type values are listed under the :py:attr:`.input_crystal_structure` property. "
			"Atoms that do not form a crystalline structure or which are part of defects are assigned the special type ``OTHER`` (=0). "
			"For these atoms the local elastic deformation cannot be computed. "
			"\n\n"
			"If :py:attr:`.calculate_deformation_gradients` is set to true, the modifier outputs a new particle property named ``Elastic Deformation Gradient``, "
			"which contains the per-atom elastic deformation gradient tensors. Each tensor has nine components stored in column-major order. "
			"Atoms for which the elastic deformation gradient could not be determined (i.e. which are classified as ``OTHER``) will be assigned the null tensor. "
			"\n\n"
			"If :py:attr:`.calculate_strain_tensors` is set to true, the modifier outputs a new particle property named ``Elastic Strain``, "
			"which contains the per-atom elastic strain tensors. Each symmetric strain tensor has six components stored in the order XX, YY, ZZ, XY, XZ, YZ. "
			"Atoms for which the elastic strain tensor could not be determined (i.e. which are classified as ``OTHER``) will be assigned the null tensor. "
			"\n\n"
			"Furthermore, the modifier generates a particle property ``Volumetric Strain``, which stores the trace divided by three of the local elastic strain tensor. "
			"Atoms for which the elastic strain tensor could not be determined (i.e. which are classified as ``OTHER``) will be assigned a value of zero. "
			"\n\n"
			)
		.def_property("input_crystal_structure", &ElasticStrainModifier::inputCrystalStructure, &ElasticStrainModifier::setInputCrystalStructure,
				"The type of crystal to analyze. Must be one of: "
				"\n\n"
				"  * ``ElasticStrainModifier.Lattice.FCC``\n"
				"  * ``ElasticStrainModifier.Lattice.HCP``\n"
				"  * ``ElasticStrainModifier.Lattice.BCC``\n"
				"  * ``ElasticStrainModifier.Lattice.CubicDiamond``\n"
				"  * ``ElasticStrainModifier.Lattice.HexagonalDiamond``\n"
				"\n\n"
				":Default: ``ElasticStrainModifier.Lattice.FCC``\n")
		.def_property("calculate_deformation_gradients", &ElasticStrainModifier::calculateDeformationGradients, &ElasticStrainModifier::setCalculateDeformationGradients,
				"Flag that enables the output of the calculated elastic deformation gradient tensors. The per-particle tensors will be stored in a new "
				"particle property named ``Elastic Deformation Gradient`` with nine components (stored in column-major order). "
				"Particles for which the local elastic deformation cannot be calculated, are assigned the null tensor. "
				"\n\n"
				":Default: False\n")
		.def_property("calculate_strain_tensors", &ElasticStrainModifier::calculateStrainTensors, &ElasticStrainModifier::setCalculateStrainTensors,
				"Flag that enables the calculation and out of the elastic strain tensors. The symmetric strain tensors will be stored in a new "
				"particle property named ``Elastic Strain`` with six components (XX, YY, ZZ, XY, XZ, YZ). "
				"\n\n"
				":Default: True\n")
		.def_property("push_strain_tensors_forward", &ElasticStrainModifier::pushStrainTensorsForward, &ElasticStrainModifier::setPushStrainTensorsForward,
				"Selects the frame in which the elastic strain tensors are calculated. "
				"\n\n"
				"If true, the *Eulerian-Almansi* finite strain tensor is computed, which measures the elastic strain in the global coordinate system (spatial frame). "
				"\n\n"
				"If false, the *Green-Lagrangian* strain tensor is computed, which measures the elastic strain in the local lattice coordinate system (material frame). "
				"\n\n"
				":Default: True\n")
		.def_property("lattice_constant", &ElasticStrainModifier::latticeConstant, &ElasticStrainModifier::setLatticeConstant,
				"Lattice constant (*a*:sub:`0`) of the ideal unit cell."
				"\n\n"
				":Default: 1.0\n")
		.def_property("axial_ratio", &ElasticStrainModifier::axialRatio, &ElasticStrainModifier::setAxialRatio,
				"The *c/a* ratio of the ideal unit cell for crystals with hexagonal symmetry."
				"\n\n"
				":Default: sqrt(8/3)\n")
	;

	ovito_class<CAImporter, ParticleImporter>{m}
	;

	ovito_class<DislocImporter, ParticleImporter>{m}
	;

	ovito_class<ParaDiSImporter, ParticleImporter>{m}
	;

	ovito_class<CAExporter, FileExporter>{m}
		.def_property("export_mesh", &CAExporter::meshExportEnabled, &CAExporter::setMeshExportEnabled)
	;

	ovito_class<VTKDislocationsExporter, FileExporter>{m}
	;

	auto DislocationVis_py = ovito_class<DislocationVis, TransformingDataVis>(m,
			":Base class: :py:class:`ovito.vis.DataVis`\n\n"
			"Controls the visual appearance of dislocation lines extracted by a :py:class:`~ovito.modifiers.DislocationAnalysisModifier`. "
			"An instance of this class is attached to every :py:class:`~ovito.data.DislocationNetwork` data object. "
			"\n\n"
			"See also the corresponding :ovitoman:`user manual page <../../display_objects.dislocations>` for more information on this visual element. ",
			// Python class name:
			"DislocationVis")
		.def_property("shading", &DislocationVis::shadingMode, &DislocationVis::setShadingMode,
				"The shading style used for the lines.\n"
				"Possible values:"
				"\n\n"
				"   * ``DislocationVis.Shading.Normal`` (default) \n"
				"   * ``DislocationVis.Shading.Flat``\n"
				"\n")
		.def_property("burgers_vector_width", &DislocationVis::burgersVectorWidth, &DislocationVis::setBurgersVectorWidth,
				"Specifies the width of Burgers vector arrows (in length units)."
				"\n\n"
				":Default: 0.6\n")
		.def_property("burgers_vector_width", &DislocationVis::burgersVectorScaling, &DislocationVis::setBurgersVectorScaling,
				"The scaling factor applied to displayed Burgers vectors. This can be used to exaggerate the arrow size."
				"\n\n"
				":Default: 1.0\n")
		.def_property("burgers_vector_color", &DislocationVis::burgersVectorColor, &DislocationVis::setBurgersVectorColor,
				"The color of Burgers vector arrows."
				"\n\n"
				":Default: ``(0.7, 0.7, 0.7)``\n")
		.def_property("show_burgers_vectors", &DislocationVis::showBurgersVectors, &DislocationVis::setShowBurgersVectors,
				"Boolean flag that enables the display of Burgers vector arrows."
				"\n\n"
				":Default: ``False``\n")
		.def_property("show_line_directions", &DislocationVis::showLineDirections, &DislocationVis::setShowLineDirections,
				"Boolean flag that enables the visualization of line directions."
				"\n\n"
				":Default: ``False``\n")
		.def_property("line_width", &DislocationVis::lineWidth, &DislocationVis::setLineWidth,
				"Controls the display width (in units of length of the simulation) of dislocation lines."
				"\n\n"
				":Default: ``1.0``\n")
		.def_property("indicate_character", &DislocationVis::lineColoringMode, &DislocationVis::setLineColoringMode,
				"Controls how the display color of dislocation lines is chosen."
				"Possible values:"
				"\n\n"
				"   * ``DislocationVis.ColoringMode.ByDislocationType`` (default) \n"
				"   * ``DislocationVis.ColoringMode.ByBurgersVector``\n"
				"   * ``DislocationVis.ColoringMode.ByCharacter``\n"
				"\n")
	;

	ovito_enum<DislocationVis::LineColoringMode>(DislocationVis_py, "ColoringMode")
		.value("ByDislocationType", DislocationVis::ColorByDislocationType)
		.value("ByBurgersVector", DislocationVis::ColorByBurgersVector)
		.value("ByCharacter", DislocationVis::ColorByCharacter)
	;

	ovito_class<DislocationNetworkObject, PeriodicDomainDataObject>(m,
			":Base class: :py:class:`ovito.data.DataObject`"
			"\n\n"
			"This data object stores the network of dislocation lines extracted by a :py:class:`~ovito.modifiers.DislocationAnalysisModifier`. "
			"You can access it through the :py:attr:`DataCollection.dislocations` field. "
			"\n\n"
			"The dislocation network is associated with a :py:class:`~ovito.vis.DislocationVis` element "
			"controlling the visual appearance of the dislocation lines. It can be accessed through "
			"the :py:attr:`~DataObject.vis` attribute of the :py:class:`~DataObject` base class."
			"\n\n"
			"Example:\n\n"
			".. literalinclude:: ../example_snippets/dislocation_analysis_modifier.py\n"
			"   :lines: 4-\n"
			"\n\n"
			"**File export**"
			"\n\n"
			"A dislocation network can be written to a data file in the form of polylines using the "
			":py:func:`ovito.io.export_file` function (select the ``vtk/disloc`` output format). "
			"During export, a non-periodic version is produced by clipping dislocation lines at the domain boundaries. ",
			// Python class name:
			"DislocationNetwork")

		.def_property_readonly("segments", py::cpp_function(&DislocationNetworkObject::segments, py::return_value_policy::reference_internal),
				"The list of dislocation segments in this dislocation network. "
				"This list-like object is read-only and contains :py:class:`~ovito.data.DislocationSegment` objects.")
	;

	py::class_<DislocationSegment>(m, "DislocationSegment",
			"A single dislocation line from a :py:class:`DislocationNetwork`. "
			"\n\n"
			"The list of dislocation segments is returned by the :py:attr:`DislocationNetwork.segments` attribute.")
		.def_readonly("id", &DislocationSegment::id,
				"The unique identifier of this dislocation segment.")
		.def_property_readonly("is_loop", &DislocationSegment::isClosedLoop,
				"This property indicates whether this segment forms a closed dislocation loop. "
				"Note that an infinite dislocation line passing through a periodic boundary is also considered a loop. "
				"\n\n"
				"See also the :py:attr:`.is_infinite_line` property. ")
		.def_property_readonly("is_infinite_line", &DislocationSegment::isInfiniteLine,
				"This property indicates whether this segment is an infinite line passing through a periodic simulation box boundary. "
				"A segment is considered infinite if it is a closed loop and its start and end points do not coincide. "
				"\n\n"
				"See also the :py:attr:`.is_loop` property. ")
		.def_property_readonly("length", &DislocationSegment::calculateLength,
				"Returns the length of this dislocation segment.")
		.def_property_readonly("true_burgers_vector", py::cpp_function([](const DislocationSegment& segment) -> const Vector3& {
					return segment.burgersVector.localVec();
				}, py::return_value_policy::reference_internal),
				"The Burgers vector of the segment, expressed in the local coordinate system of the crystal. Also known as the True Burgers vector.")
		.def_property_readonly("spatial_burgers_vector", [](const DislocationSegment& segment) -> Vector3 {
					return segment.burgersVector.toSpatialVector();
				},
				"The Burgers vector of the segment, expressed in the global coordinate system of the simulation. This vector is calculated "
				"by transforming the true Burgers vector from the local lattice coordinate system to the global simulation coordinate system "
				"using the average orientation matrix of the crystal cluster the dislocation segment is embedded in.")
		.def_property_readonly("points", [](const DislocationSegment& segment) {
					py::array_t<FloatType> array({ (size_t)segment.line.size(), (size_t)3 });
					for(size_t i = 0; i < segment.line.size(); i++) {
						for(size_t j = 0; j < 3; j++) {
							array.mutable_at(i, j) = segment.line[i][j];
						}
					}
					return array;
				},
				"The list of space points that define the shape of this dislocation segment. "
        		"This is a *N* x 3 Numpy array, where *N* is the number of points along the "
        		"segment. For closed loops, the first and the last point coincide.")
		.def_property_readonly("cluster_id", [](const DislocationSegment& segment) {
					return segment.burgersVector.cluster()->id;
				},
				"The numeric identifier of the crystal cluster of atoms containing this dislocation segment. "
				"\n\n"
				"The true Burgers vector of the segment is expressed in the local coordinate system of this crystal cluster.")
	;

	ovito_class<ClusterGraphObject, DataObject>(m, "ClusterGraph")
		.def_property_readonly("clusters", py::cpp_function(&ClusterGraphObject::clusters, py::return_value_policy::reference_internal))
		//.def("find_cluster", make_function(&ClusterGraphObject::findCluster, return_internal_reference<>()))
	;

	py::class_<Cluster>(m, "Cluster")
		.def_readonly("id", &Cluster::id,
				"The unique identifier of this atomic cluster.")
		.def_readonly("atom_count", &Cluster::atomCount)
		.def_readonly("orientation", &Cluster::orientation)
	;

	ovito_class<BurgersVectorFamily, ElementType>{m};
	ovito_class<MicrostructurePhase, ElementType>{m};
	ovito_class<Microstructure, SurfaceMesh>{m};
}

OVITO_REGISTER_PLUGIN_PYTHON_INTERFACE(CrystalAnalysisPython);

}	// End of namespace
}	// End of namespace
