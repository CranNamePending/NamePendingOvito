////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright 2019 Alexander Stukowski
//
//  This file is part of OVITO (Open Visualization Tool).
//
//  OVITO is free software; you can redistribute it and/or modify it either under the
//  terms of the GNU General Public License version 3 as published by the Free Software
//  Foundation (the "GPL") or, at your option, under the terms of the MIT License.
//  If you do not alter this notice, a recipient may use your version of this
//  file under either the GPL or the MIT License.
//
//  You should have received a copy of the GPL along with this program in a
//  file LICENSE.GPL.txt.  You should have received a copy of the MIT License along
//  with this program in a file LICENSE.MIT.txt
//
//  This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND,
//  either express or implied. See the GPL or the MIT License for the specific language
//  governing rights and limitations.
//
////////////////////////////////////////////////////////////////////////////////////////

#include <ovito/particles/Particles.h>
#include <ovito/particles/objects/ParticlesObject.h>
#include <ovito/particles/util/CutoffNeighborFinder.h>
#include <ovito/delaunay/DelaunayTessellation.h>
#include <ovito/delaunay/ManifoldConstructionHelper.h>
#include <ovito/mesh/surface/SurfaceMesh.h>
#include <ovito/grid/modifier/MarchingCubes.h>
#include <ovito/stdobj/simcell/SimulationCellObject.h>
#include <ovito/stdobj/properties/PropertyAccess.h>
#include <ovito/core/dataset/pipeline/ModifierApplication.h>
#include <ovito/core/dataset/DataSet.h>
#include <ovito/core/utilities/units/UnitsManager.h>
#include <ovito/core/utilities/concurrent/ParallelFor.h>
#include "ConstructSurfaceModifier.h"

namespace Ovito { namespace Particles {

using namespace Ovito::Delaunay;

IMPLEMENT_OVITO_CLASS(ConstructSurfaceModifier);
DEFINE_PROPERTY_FIELD(ConstructSurfaceModifier, smoothingLevel);
DEFINE_PROPERTY_FIELD(ConstructSurfaceModifier, probeSphereRadius);
DEFINE_PROPERTY_FIELD(ConstructSurfaceModifier, onlySelectedParticles);
DEFINE_PROPERTY_FIELD(ConstructSurfaceModifier, selectSurfaceParticles);
DEFINE_PROPERTY_FIELD(ConstructSurfaceModifier, transferParticleProperties);
DEFINE_REFERENCE_FIELD(ConstructSurfaceModifier, surfaceMeshVis);
DEFINE_PROPERTY_FIELD(ConstructSurfaceModifier, method);
DEFINE_PROPERTY_FIELD(ConstructSurfaceModifier, gridResolution);
DEFINE_PROPERTY_FIELD(ConstructSurfaceModifier, radiusFactor);
DEFINE_PROPERTY_FIELD(ConstructSurfaceModifier, isoValue);
SET_PROPERTY_FIELD_LABEL(ConstructSurfaceModifier, smoothingLevel, "Smoothing level");
SET_PROPERTY_FIELD_LABEL(ConstructSurfaceModifier, probeSphereRadius, "Probe sphere radius");
SET_PROPERTY_FIELD_LABEL(ConstructSurfaceModifier, onlySelectedParticles, "Use only selected input particles");
SET_PROPERTY_FIELD_LABEL(ConstructSurfaceModifier, selectSurfaceParticles, "Select particles on the surface");
SET_PROPERTY_FIELD_LABEL(ConstructSurfaceModifier, transferParticleProperties, "Transfer particle properties to surface");
SET_PROPERTY_FIELD_LABEL(ConstructSurfaceModifier, method, "Construction method");
SET_PROPERTY_FIELD_LABEL(ConstructSurfaceModifier, gridResolution, "Resolution");
SET_PROPERTY_FIELD_LABEL(ConstructSurfaceModifier, radiusFactor, "Radius scaling");
SET_PROPERTY_FIELD_LABEL(ConstructSurfaceModifier, isoValue, "Iso value");
SET_PROPERTY_FIELD_UNITS_AND_MINIMUM(ConstructSurfaceModifier, probeSphereRadius, WorldParameterUnit, 0);
SET_PROPERTY_FIELD_UNITS_AND_MINIMUM(ConstructSurfaceModifier, smoothingLevel, IntegerParameterUnit, 0);
SET_PROPERTY_FIELD_UNITS_AND_RANGE(ConstructSurfaceModifier, gridResolution, IntegerParameterUnit, 2, 600);
SET_PROPERTY_FIELD_UNITS_AND_MINIMUM(ConstructSurfaceModifier, radiusFactor, PercentParameterUnit, 0);

/******************************************************************************
* Constructs the modifier object.
******************************************************************************/
ConstructSurfaceModifier::ConstructSurfaceModifier(DataSet* dataset) : AsynchronousModifier(dataset),
	_smoothingLevel(8),
	_probeSphereRadius(4),
	_onlySelectedParticles(false),
	_selectSurfaceParticles(false),
	_transferParticleProperties(false),
	_method(AlphaShape),
	_gridResolution(50),
	_radiusFactor(1.0),
	_isoValue(0.6)
{
	// Create the vis element for rendering the surface generated by the modifier.
	setSurfaceMeshVis(new SurfaceMeshVis(dataset));
}

/******************************************************************************
* Asks the modifier whether it can be applied to the given input data.
******************************************************************************/
bool ConstructSurfaceModifier::OOMetaClass::isApplicableTo(const DataCollection& input) const
{
	return input.containsObject<ParticlesObject>();
}

/******************************************************************************
* Creates and initializes a computation engine that will compute the
* modifier's results.
******************************************************************************/
Future<AsynchronousModifier::ComputeEnginePtr> ConstructSurfaceModifier::createEngine(const PipelineEvaluationRequest& request, ModifierApplication* modApp, const PipelineFlowState& input)
{
	// Get modifier inputs.
	const ParticlesObject* particles = input.expectObject<ParticlesObject>();
	particles->verifyIntegrity();
	const PropertyObject* posProperty = particles->expectProperty(ParticlesObject::PositionProperty);
	ConstPropertyPtr selProperty;
	if(onlySelectedParticles())
		selProperty = particles->expectProperty(ParticlesObject::SelectionProperty)->storage();
	const SimulationCellObject* simCell = input.expectObject<SimulationCellObject>();
	if(simCell->is2D())
		throwException(tr("The construct surface mesh modifier does not support 2d simulation cells."));

	// Collect the set of particle properties that should be transferred over to the surface mesh vertices.
	std::vector<ConstPropertyPtr> particleProperties;
	if(transferParticleProperties()) {
		for(const PropertyObject* property : particles->properties()) {
			// Certain properties should not be transferred to the mesh vertices.
			if(property->type() == ParticlesObject::SelectionProperty) continue;
			if(property->type() == ParticlesObject::PositionProperty) continue;
			if(property->type() == ParticlesObject::IdentifierProperty) continue;
			particleProperties.push_back(property->storage());
		}
	}
	
	if(method() == AlphaShape) {
		// Create engine object. Pass all relevant modifier parameters to the engine as well as the input data.
		return std::make_shared<AlphaShapeEngine>(posProperty->storage(),
				std::move(selProperty),
				simCell->data(),
				probeSphereRadius(),
				smoothingLevel(),
				selectSurfaceParticles(),
				std::move(particleProperties));
	}
	else {
		// Create engine object. Pass all relevant modifier parameters to the engine as well as the input data.
		return std::make_shared<GaussianDensityEngine>(posProperty->storage(),
				std::move(selProperty),
				simCell->data(),
				radiusFactor(),
				isoValue(),
				gridResolution(),
				particles->inputParticleRadii(),
				std::move(particleProperties));
	}
}

/******************************************************************************
* Performs the actual analysis. This method is executed in a worker thread.
******************************************************************************/
void ConstructSurfaceModifier::AlphaShapeEngine::perform()
{
	setProgressText(tr("Constructing surface mesh"));

	if(probeSphereRadius() <= 0)
		throw Exception(tr("Radius parameter must be positive."));

	if(mesh().cell().volume3D() <= FLOATTYPE_EPSILON*FLOATTYPE_EPSILON*FLOATTYPE_EPSILON)
		throw Exception(tr("Simulation cell is degenerate."));

	double alpha = probeSphereRadius() * probeSphereRadius();
	FloatType ghostLayerSize = probeSphereRadius() * FloatType(3);

	// Check if combination of radius parameter and simulation cell size is valid.
	for(size_t dim = 0; dim < 3; dim++) {
		if(mesh().cell().pbcFlags()[dim]) {
			int stencilCount = (int)ceil(ghostLayerSize / mesh().cell().matrix().column(dim).dot(mesh().cell().cellNormalVector(dim)));
			if(stencilCount > 1)
				throw Exception(tr("Cannot generate Delaunay tessellation. Simulation cell is too small, or radius parameter is too large."));
		}
	}

	// If there are too few particles, don't build Delaunay tessellation.
	// It is going to be invalid anyway.
	size_t numInputParticles = positions()->size();
	if(selection()) {
		numInputParticles = positions()->size() - boost::count(ConstPropertyAccess<int>(selection()), 0);
	}
	if(numInputParticles <= 3) {
		// Release data that is no longer needed.
		releaseWorkingData();
		return;
	}

	// Algorithm is divided into several sub-steps.
	// Assign weights to sub-steps according to estimated runtime.
	beginProgressSubStepsWithWeights({ 10, 30, 2, 2, 4 });

	// Generate Delaunay tessellation.
	DelaunayTessellation tessellation;
	if(!tessellation.generateTessellation(
			mesh().cell(), 
			ConstPropertyAccess<Point3>(positions()).cbegin(), 
			positions()->size(), 
			ghostLayerSize,
			selection() ? ConstPropertyAccess<int>(selection()).cbegin() : nullptr, 
			*this))
		return;

	nextProgressSubStep();

	// Determines the region a solid Delaunay cell belongs to.
	// We use this callback function to compute the total volume of the solid region.
	auto tetrahedronRegion = [this, &tessellation](DelaunayTessellation::CellHandle cell) {
		if(tessellation.isGhostCell(cell) == false) {
			Point3 p0 = tessellation.vertexPosition(tessellation.cellVertex(cell, 0));
			Vector3 ad = tessellation.vertexPosition(tessellation.cellVertex(cell, 1)) - p0;
			Vector3 bd = tessellation.vertexPosition(tessellation.cellVertex(cell, 2)) - p0;
			Vector3 cd = tessellation.vertexPosition(tessellation.cellVertex(cell, 3)) - p0;
			addSolidVolume(std::abs(ad.dot(cd.cross(bd))) / FloatType(6));
		}
		return 0;
	};

	// This callback function is called for every surface facet created by the manifold construction helper.
	PropertyAccess<int> surfaceParticleSelectionArray(surfaceParticleSelection());
	auto prepareMeshFace = [&](HalfEdgeMesh::face_index face, const std::array<size_t,3>& vertexIndices, const std::array<DelaunayTessellation::VertexHandle,3>& vertexHandles, DelaunayTessellation::CellHandle cell) {
		// Mark vertex atoms as belonging to the surface.
		if(surfaceParticleSelectionArray) {
			for(size_t vi : vertexIndices) {
				OVITO_ASSERT(vi < surfaceParticleSelectionArray.size());
				surfaceParticleSelectionArray[vi] = 1;
			}
		}
	};

	// This callback function is called for every surface vertex created by the manifold construction helper.
	std::vector<size_t> vertexToParticleMap;
	auto prepareMeshVertex = [&](HalfEdgeMesh::vertex_index vertex, size_t particleIndex) {
		OVITO_ASSERT(vertex == vertexToParticleMap.size());
		vertexToParticleMap.push_back(particleIndex);
	};

	ManifoldConstructionHelper<false, false, true> manifoldConstructor(tessellation, mesh(), alpha, *positions());
	if(!manifoldConstructor.construct(tetrahedronRegion, *this, std::move(prepareMeshFace), std::move(prepareMeshVertex)))
		return;

	// Copy particle property values to mesh vertices.
	for(const ConstPropertyPtr& particleProperty : particleProperties()) {
		PropertyStorage* vertexProperty;
		if(SurfaceMeshVertices::OOClass().isValidStandardPropertyId(particleProperty->type())) {
			// Input property is also a standard property for mesh vertices.
			vertexProperty = mesh().createVertexProperty(static_cast<SurfaceMeshVertices::Type>(particleProperty->type())).get();
			OVITO_ASSERT(vertexProperty->dataType() == particleProperty->dataType());
			OVITO_ASSERT(vertexProperty->stride() == particleProperty->stride());
		}
		else if(SurfaceMeshVertices::OOClass().standardPropertyTypeId(particleProperty->name()) != 0) {
			// Input property name is that of a standard property for mesh vertices.
			// Must rename the property to avoid conflict, because user properties may not have a standard property name.
			QString newPropertyName = particleProperty->name() + tr("_particles");
			vertexProperty = mesh().createVertexProperty(particleProperty->dataType(), particleProperty->componentCount(), particleProperty->stride(), newPropertyName, false, particleProperty->componentNames()).get();
		}
		else {
			// Input property is a user property for mesh vertices.
			vertexProperty = mesh().createVertexProperty(particleProperty->dataType(), particleProperty->componentCount(), particleProperty->stride(), particleProperty->name(), false, particleProperty->componentNames()).get();
		}
		particleProperty->mappedCopyTo(*vertexProperty, vertexToParticleMap);
	}

	nextProgressSubStep();

	// Make sure every mesh vertex is only part of one surface manifold.
	mesh().makeManifold();

	nextProgressSubStep();
	if(!mesh().smoothMesh(_smoothingLevel, *this))
		return;

	// Create the 'Surface area' region property.
	PropertyAccess<FloatType> surfaceAreaProperty = mesh().createRegionProperty(SurfaceMeshRegions::SurfaceAreaProperty, true);

	// Compute surface area (total and per-region) by summing up the triangle face areas.
	nextProgressSubStep();
	setProgressMaximum(mesh().faceCount());
	for(HalfEdgeMesh::edge_index edge : mesh().firstFaceEdges()) {
		if(!incrementProgressValue()) return;
		const Vector3& e1 = mesh().edgeVector(edge);
		const Vector3& e2 = mesh().edgeVector(mesh().nextFaceEdge(edge));
		FloatType area = e1.cross(e2).length() / 2;
		addSurfaceArea(area);
		SurfaceMeshData::region_index region = mesh().faceRegion(mesh().adjacentFace(edge));
		surfaceAreaProperty[region] += area;
	}

	endProgressSubSteps();

	// Release data that is no longer needed.
	releaseWorkingData();
}

/******************************************************************************
* Performs the actual analysis. This method is executed in a worker thread.
******************************************************************************/
void ConstructSurfaceModifier::GaussianDensityEngine::perform()
{
	setProgressText(tr("Constructing surface mesh"));

	// Check input data.
	if(mesh().cell().volume3D() <= FLOATTYPE_EPSILON*FLOATTYPE_EPSILON*FLOATTYPE_EPSILON)
		throw Exception(tr("Simulation cell is degenerate."));

	if(positions()->size() == 0) {
		// Release data that is no longer needed.
		releaseWorkingData();
		return;
	}

	// Algorithm is divided into several sub-steps.
	// Assign weights to sub-steps according to estimated runtime.
	beginProgressSubStepsWithWeights({ 1, 30, 1600, 1500, 30, 500, 100, 300 });

	// Scale the atomic radii.
	for(FloatType& r : _particleRadii) r *= _radiusFactor;

	// Determine the cutoff range of atomic Gaussians.
	FloatType cutoffSize = FloatType(3) * *std::max_element(_particleRadii.cbegin(), _particleRadii.cend());

	// Determine the extents of the density grid.
	AffineTransformation gridBoundaries = mesh().cell().matrix();
	ConstPropertyAccess<Point3> positionsArray(positions());
	for(size_t dim = 0; dim < 3; dim++) {
		// Use bounding box of particles in directions that are non-periodic.
		if(!mesh().cell().pbcFlags()[dim]) {
			// Compute range of relative atomic coordinates in the current direction.
			FloatType xmin =  FLOATTYPE_MAX;
			FloatType xmax = -FLOATTYPE_MAX;
			for(const Point3& p : positionsArray) {
				FloatType rp = mesh().cell().inverseMatrix().prodrow(p, dim);
				if(rp < xmin) xmin = rp;
				if(rp > xmax) xmax = rp;
			}

			// Need to add extra margin along non-periodic dimensions, because Gaussian functions reach beyond atomic radii.
			FloatType rcutoff = cutoffSize / gridBoundaries.column(dim).length();
			xmin -= rcutoff;
			xmax += rcutoff;

			gridBoundaries.column(3) += xmin * gridBoundaries.column(dim);
			gridBoundaries.column(dim) *= (xmax - xmin);
		}
	}

	// Determine the number of voxels in each direction of the density grid.
	size_t gridDims[3];
	FloatType voxelSizeX = gridBoundaries.column(0).length() / _gridResolution;
	FloatType voxelSizeY = gridBoundaries.column(1).length() / _gridResolution;
	FloatType voxelSizeZ = gridBoundaries.column(2).length() / _gridResolution;
	FloatType voxelSize = std::max(voxelSizeX, std::max(voxelSizeY, voxelSizeZ));
	gridDims[0] = std::max((size_t)2, (size_t)(gridBoundaries.column(0).length() / voxelSize));
	gridDims[1] = std::max((size_t)2, (size_t)(gridBoundaries.column(1).length() / voxelSize));
	gridDims[2] = std::max((size_t)2, (size_t)(gridBoundaries.column(2).length() / voxelSize));

	nextProgressSubStep();

	// Allocate storage for the density grid values.
	std::vector<FloatType> densityData(gridDims[0] * gridDims[1] * gridDims[2], FloatType(0));

	// Set up a particle neighbor finder to speed up density field computation.
	CutoffNeighborFinder neighFinder;
	if(!neighFinder.prepare(cutoffSize, positions(), mesh().cell(), selection(), this))
		return;

	nextProgressSubStep();

	// Set up a matrix that converts grid coordinates to spatial coordinates.
	AffineTransformation gridToCartesian = gridBoundaries;
	gridToCartesian.column(0) /= gridDims[0] - (mesh().cell().pbcFlags()[0]?0:1);
	gridToCartesian.column(1) /= gridDims[1] - (mesh().cell().pbcFlags()[1]?0:1);
	gridToCartesian.column(2) /= gridDims[2] - (mesh().cell().pbcFlags()[2]?0:1);

	// Compute the accumulated density at each grid point.
	parallelFor(densityData.size(), *this, [&](size_t voxelIndex) {

		// Determine the center coordinates of the current grid cell.
		size_t ix = voxelIndex % gridDims[0];
		size_t iy = (voxelIndex / gridDims[0]) % gridDims[1];
		size_t iz = voxelIndex / (gridDims[0] * gridDims[1]);
		Point3 voxelCenter = gridToCartesian * Point3(ix, iy, iz);
		FloatType& density = densityData[voxelIndex];

		// Visit all particles in the vicinity of the center point.
		for(CutoffNeighborFinder::Query neighQuery(neighFinder, voxelCenter); !neighQuery.atEnd(); neighQuery.next()) {
			FloatType alpha = _particleRadii[neighQuery.current()];
			density += std::exp(-neighQuery.distanceSquared() / (FloatType(2) * alpha * alpha));
		}
	});
	if(isCanceled())
		return;

	nextProgressSubStep();

	// Construct isosurface of the density field.
	mesh().cell().setMatrix(gridBoundaries);
	MarchingCubes mc(mesh(), gridDims[0], gridDims[1], gridDims[2], densityData.data(), 1, false);
	if(!mc.generateIsosurface(_isoLevel, *this))
		return;

	nextProgressSubStep();

	// Transform mesh vertices from orthogonal grid space to world space.
	mesh().transformVertices(gridToCartesian);
	if(isCanceled())
		return;

	nextProgressSubStep();

	// Create mesh vertex properties for transferring particle property values to the surface.
	std::vector<std::pair<ConstPropertyAccess<FloatType,true>, PropertyAccess<FloatType,true>>> propertyMapping;
	for(const ConstPropertyPtr& particleProperty : particleProperties()) {
		// Can only transfer floating-point properties, because we'll need to blend values of several particles.
		if(particleProperty->dataType() == PropertyStorage::Float) {
			PropertyPtr vertexProperty;
			if(SurfaceMeshVertices::OOClass().isValidStandardPropertyId(particleProperty->type())) {
				// Input property is also a standard property for mesh vertices.
				vertexProperty = mesh().createVertexProperty(static_cast<SurfaceMeshVertices::Type>(particleProperty->type()), true);
				OVITO_ASSERT(vertexProperty->dataType() == particleProperty->dataType());
				OVITO_ASSERT(vertexProperty->stride() == particleProperty->stride());
			}
			else if(SurfaceMeshVertices::OOClass().standardPropertyTypeId(particleProperty->name()) != 0) {
				// Input property name is that of a standard property for mesh vertices.
				// Must rename the property to avoid conflict, because user properties may not have a standard property name.
				QString newPropertyName = particleProperty->name() + tr("_particles");
				vertexProperty = mesh().createVertexProperty(particleProperty->dataType(), particleProperty->componentCount(), particleProperty->stride(), newPropertyName, true, particleProperty->componentNames());
			}
			else {
				// Input property is a user property for mesh vertices.
				vertexProperty = mesh().createVertexProperty(particleProperty->dataType(), particleProperty->componentCount(), particleProperty->stride(), particleProperty->name(), true, particleProperty->componentNames());
			}
			propertyMapping.emplace_back(particleProperty, std::move(vertexProperty));
		}
	}

	// Transfer property values from particles to to mesh vertices.
	if(!propertyMapping.empty()) {
		// Compute the accumulated density at each grid point.
		parallelFor(mesh().vertexCount(), *this, [&](size_t vertexIndex) {
			// Visit all particles in the vicinity of the vertex.
			FloatType weightSum = 0;
			for(CutoffNeighborFinder::Query neighQuery(neighFinder, mesh().vertexPosition(vertexIndex)); !neighQuery.atEnd(); neighQuery.next()) {
				FloatType alpha = _particleRadii[neighQuery.current()];
				FloatType weight = std::exp(-neighQuery.distanceSquared() / (FloatType(2) * alpha * alpha));
				// Perform summation of particle contributions to the property values at the current mesh vertex.
				for(auto& p : propertyMapping) {
					for(size_t component = 0; component < p.first.componentCount(); component++) {
						p.second.value(vertexIndex, component) += weight * p.first.get(neighQuery.current(), component);
					}
				}
				weightSum += weight;
			}
			if(weightSum != 0) {
				// Normalize property values.
				for(auto& p : propertyMapping) {
					for(size_t component = 0; component < p.second.componentCount(); component++) {
						p.second.value(vertexIndex, component) /= weightSum;
					}
				}
			}
		});
		if(isCanceled())
			return;
	}

	// Flip surface orientation if cell is mirrored.
	if(gridToCartesian.determinant() < 0)
		mesh().flipFaces();

	nextProgressSubStep();

	if(!mesh().connectOppositeHalfedges())
		throw Exception(tr("Something went wrong. Isosurface mesh is not closed."));
	if(isCanceled())
		return;

	nextProgressSubStep();

	// Compute surface area (total and per-region) by summing up the triangle face areas.
	for(HalfEdgeMesh::edge_index edge : mesh().firstFaceEdges()) {
		if(isCanceled()) return;
		const Vector3& e1 = mesh().edgeVector(edge);
		const Vector3& e2 = mesh().edgeVector(mesh().nextFaceEdge(edge));
		FloatType area = e1.cross(e2).length() / 2;
		addSurfaceArea(area);
	}

	endProgressSubSteps();

	// Release data that is no longer needed.
	releaseWorkingData();
}

/******************************************************************************
* Injects the computed results of the engine into the data pipeline.
******************************************************************************/
void ConstructSurfaceModifier::AlphaShapeEngine::emitResults(TimePoint time, ModifierApplication* modApp, PipelineFlowState& state)
{
	ConstructSurfaceModifier* modifier = static_object_cast<ConstructSurfaceModifier>(modApp->modifier());

	// Create the output data object.
	SurfaceMesh* meshObj = state.createObject<SurfaceMesh>(QStringLiteral("surface"), modApp, tr("Surface"));
	mesh().transferTo(meshObj);
	meshObj->setDomain(state.getObject<SimulationCellObject>());
	meshObj->setVisElement(modifier->surfaceMeshVis());

	if(surfaceParticleSelection()) {
		ParticlesObject* particles = state.expectMutableObject<ParticlesObject>();
		particles->verifyIntegrity();
		particles->createProperty(surfaceParticleSelection());
	}

	state.addAttribute(QStringLiteral("ConstructSurfaceMesh.surface_area"), QVariant::fromValue(surfaceArea()), modApp);
	state.addAttribute(QStringLiteral("ConstructSurfaceMesh.solid_volume"), QVariant::fromValue(solidVolume()), modApp);

	state.setStatus(PipelineStatus(PipelineStatus::Success, tr("Surface area: %1\nSolid volume: %2\nSimulation cell volume: %3\nSolid volume fraction: %4\nSurface area per solid volume: %5\nSurface area per total volume: %6")
			.arg(surfaceArea())
			.arg(solidVolume())
			.arg(totalVolume())
			.arg(totalVolume() > 0 ? (solidVolume() / totalVolume()) : 0)
			.arg(solidVolume() > 0 ? (surfaceArea() / solidVolume()) : 0)
			.arg(totalVolume() > 0 ? (surfaceArea() / totalVolume()) : 0)));
}

/******************************************************************************
* Injects the computed results of the engine into the data pipeline.
******************************************************************************/
void ConstructSurfaceModifier::GaussianDensityEngine::emitResults(TimePoint time, ModifierApplication* modApp, PipelineFlowState& state)
{
	ConstructSurfaceModifier* modifier = static_object_cast<ConstructSurfaceModifier>(modApp->modifier());

	// Create the output data object.
	SurfaceMesh* meshObj = state.createObject<SurfaceMesh>(QStringLiteral("surface"), modApp, tr("Surface"));
	mesh().transferTo(meshObj);
	meshObj->setVisElement(modifier->surfaceMeshVis());

	// Set the spatial domain of the mesh.
	meshObj->setDomain(new SimulationCellObject(meshObj->dataset(), mesh().cell()));

	state.addAttribute(QStringLiteral("ConstructSurfaceMesh.surface_area"), QVariant::fromValue(surfaceArea()), modApp);

	state.setStatus(PipelineStatus(PipelineStatus::Success, tr("Surface area: %1").arg(surfaceArea())));
}

}	// End of namespace
}	// End of namespace
