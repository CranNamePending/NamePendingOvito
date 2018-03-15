///////////////////////////////////////////////////////////////////////////////
//
//  Copyright (2016) Alexander Stukowski
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

#include <plugins/crystalanalysis/CrystalAnalysis.h>
#include <plugins/crystalanalysis/objects/clusters/ClusterGraphObject.h>
#include <plugins/crystalanalysis/objects/patterns/PatternCatalog.h>
#include <plugins/crystalanalysis/objects/microstructure/MicrostructureObject.h>
#include <plugins/mesh/surface/RenderableSurfaceMesh.h>
#include <core/rendering/SceneRenderer.h>
#include <core/utilities/mesh/TriMesh.h>
#include <core/utilities/units/UnitsManager.h>
#include <core/dataset/animation/controller/Controller.h>
#include <plugins/stdobj/simcell/SimulationCellObject.h>
#include <core/dataset/DataSetContainer.h>
#include "SlipSurfaceVis.h"

namespace Ovito { namespace Plugins { namespace CrystalAnalysis {

IMPLEMENT_OVITO_CLASS(SlipSurfaceVis);
DEFINE_PROPERTY_FIELD(SlipSurfaceVis, smoothShading);
DEFINE_REFERENCE_FIELD(SlipSurfaceVis, surfaceTransparencyController);
SET_PROPERTY_FIELD_LABEL(SlipSurfaceVis, smoothShading, "Smooth shading");
SET_PROPERTY_FIELD_LABEL(SlipSurfaceVis, surfaceTransparencyController, "Surface transparency");
SET_PROPERTY_FIELD_UNITS_AND_RANGE(SlipSurfaceVis, surfaceTransparencyController, PercentParameterUnit, 0, 1);

/******************************************************************************
* Constructor.
******************************************************************************/
SlipSurfaceVis::SlipSurfaceVis(DataSet* dataset) : TransformingDataVis(dataset),
	_smoothShading(false)
{
	setSurfaceTransparencyController(ControllerManager::createFloatController(dataset));
}

/******************************************************************************
* Is called when the value of a property of this object has changed.
******************************************************************************/
void SlipSurfaceVis::propertyChanged(const PropertyFieldDescriptor& field)
{
	if(field == PROPERTY_FIELD(smoothShading)) {
		// This kind of parameter change triggers a regeneration of the cached RenderableSurfaceMesh.
		invalidateTransformedObjects();
	}
	TransformingDataVis::propertyChanged(field);
}

/******************************************************************************
* Lets the vis element transform a data object in preparation for rendering.
******************************************************************************/
Future<PipelineFlowState> SlipSurfaceVis::transformDataImpl(TimePoint time, DataObject* dataObject, PipelineFlowState&& flowState, const PipelineFlowState& cachedState, PipelineSceneNode* contextNode)
{
	// Get the microstructure object.
	MicrostructureObject* microstructureObj = dynamic_object_cast<MicrostructureObject>(dataObject);
	if(!microstructureObj)
		return std::move(flowState);

	// Get the simulation cell.
	SimulationCellObject* cellObject = microstructureObj->domain();
	if(!cellObject)
		return std::move(flowState);
			
	// Get the cluster graph.
	ClusterGraphObject* clusterGraphObject = flowState.findObject<ClusterGraphObject>();
	
	// Build lookup map of lattice structure names.
	QStringList structureNames;
	if(PatternCatalog* patternCatalog = flowState.findObject<PatternCatalog>()) {
		for(StructurePattern* pattern : patternCatalog->patterns()) {
			if(pattern->id() < 0) continue;
			while(pattern->id() >= structureNames.size()) structureNames.append(QString());
			structureNames[pattern->id()] = pattern->shortName();
		}
	}

	// Create compute engine.
	auto engine = std::make_shared<PrepareMeshEngine>(
		microstructureObj->storage(),
		clusterGraphObject ? clusterGraphObject->storage() : nullptr,
		cellObject->data(),
		structureNames,
		microstructureObj->cuttingPlanes(),
		smoothShading());

	// Submit engine for execution and post-process results.
	return dataset()->container()->taskManager().runTaskAsync(engine)
		.then(executor(), [this, flowState = std::move(flowState), dataObject](TriMesh&& surfaceMesh, std::vector<ColorA>&& materialColors) mutable {
			UndoSuspender noUndo(this);

			// Increase surface color brightness.
			for(ColorA& c : materialColors) {
				c.r() = std::min(c.r() + FloatType(0.3), FloatType(1));
				c.g() = std::min(c.g() + FloatType(0.3), FloatType(1));
				c.b() = std::min(c.b() + FloatType(0.3), FloatType(1));
			}
				
			// Output the computed mesh as a RenderableSurfaceMesh.
			OORef<RenderableSurfaceMesh> renderableMesh = new RenderableSurfaceMesh(this, dataObject, std::move(surfaceMesh), {});
			renderableMesh->setMaterialColors(std::move(materialColors));
			flowState.addObject(renderableMesh);
			return std::move(flowState);
		});
}

/******************************************************************************
* Computes the results and stores them in this object for later retrieval.
******************************************************************************/
void SlipSurfaceVis::PrepareMeshEngine::perform()
{
	setProgressText(tr("Preparing slip surface for display"));

	TriMesh surfaceMesh;	
	std::vector<ColorA> materialColors;
	
	if(!buildMesh(*_inputMesh, _simCell, _cuttingPlanes, _structureNames, surfaceMesh, materialColors, *this))
		throw Exception(tr("Failed to generate non-periodic version of slip surface for display. Simulation cell might be too small."));

	if(isCanceled())
		return;

	if(_smoothShading) {
		// Assign smoothing group to faces to interpolate normals.
		for(auto& face : surfaceMesh.faces())
			face.setSmoothingGroups(1);	
	}
			
	setResult(std::move(surfaceMesh), std::move(materialColors));
}

/******************************************************************************
* Computes the bounding box of the displayed data.
******************************************************************************/
Box3 SlipSurfaceVis::boundingBox(TimePoint time, DataObject* dataObject, PipelineSceneNode* contextNode, const PipelineFlowState& flowState, TimeInterval& validityInterval)
{
	// Compute mesh bounding box.
	Box3 bb;
	if(OORef<RenderableSurfaceMesh> meshObj = dataObject->convertTo<RenderableSurfaceMesh>(time)) {
		bb.addBox(meshObj->surfaceMesh().boundingBox());
	}
	return bb;
}

/******************************************************************************
* Lets the visualization element render the data object.
******************************************************************************/
void SlipSurfaceVis::render(TimePoint time, DataObject* dataObject, const PipelineFlowState& flowState, SceneRenderer* renderer, PipelineSceneNode* contextNode)
{
#if 0	
	// Ignore render calls for the original MicrostructureObject.
	// We are only interested in the RenderableSurfaceMesh.
	if(dynamic_object_cast<MicrostructureObject>(dataObject) != nullptr)
		return;

	if(renderer->isBoundingBoxPass()) {
		TimeInterval validityInterval;
		renderer->addToLocalBoundingBox(boundingBox(time, dataObject, contextNode, flowState, validityInterval));
		return;
	}

	// Get the rendering colors for the surface.
	FloatType surface_alpha = 1;
	TimeInterval iv;
	if(surfaceTransparencyController()) surface_alpha = FloatType(1) - surfaceTransparencyController()->getFloatValue(time, iv);
	ColorA color_surface(1, 1, 1, surface_alpha);

	// Do we have to re-create the render primitives from scratch?
	bool recreateSurfaceBuffer = !_surfaceBuffer || !_surfaceBuffer->isValid(renderer);

	// Get the renderable mesh.
	RenderableSurfaceMesh* renderableMesh = dynamic_object_cast<RenderableSurfaceMesh>(dataObject);
	if(!renderableMesh) return;

	// Do we have to update the render primitives?
	bool updateContents = _geometryCacheHelper.updateState(renderableMesh, surface_alpha)
					|| recreateSurfaceBuffer;

	// Re-create the render primitives if necessary.
	if(recreateSurfaceBuffer)
		_surfaceBuffer = renderer->createMeshPrimitive();

	// Update render primitives.
	if(updateContents) {
		
		auto materialColors = renderableMesh->materialColors();
		for(ColorA& c : materialColors)
			c.a() = surface_alpha;
		_surfaceBuffer->setMaterialColors(materialColors);

		_surfaceBuffer->setMesh(renderableMesh->surfaceMesh(), color_surface);
	}

	// Handle picking of triangles.
	renderer->beginPickObject(contextNode);
	_surfaceBuffer->render(renderer);
	renderer->endPickObject();
#endif	
}

/******************************************************************************
* Generates the final triangle mesh, which will be rendered.
******************************************************************************/
bool SlipSurfaceVis::buildMesh(const Microstructure& input, const SimulationCell& cell, const QVector<Plane3>& cuttingPlanes, const QStringList& structureNames, TriMesh& output, std::vector<ColorA>& materialColors, PromiseState& promise)
{
	// This predicate function selects the faces of the microstructure mesh 
	// that are part of the slip surfaces to be rendered.
	auto facePredicate = [](const Microstructure::Face* face) {
		return face->isSlipSurfaceFace() && face->isEvenFace();
	};

	// Convert all slip faces of the half-edge mesh to a triangle mesh.
	input.convertToTriMesh(output, facePredicate);
	
	// Color faces according to slip vector.
	auto fout = output.faces().begin();
	for(Microstructure::Face* face : input.faces()) {
		if(!facePredicate(face)) continue;

		int materialIndex = 0;
		if(Cluster* cluster = face->cluster()) {
			if(cluster->structure < structureNames.size() && structureNames[cluster->structure].isEmpty() == false) {
				ColorA c = StructurePattern::getBurgersVectorColor(structureNames[cluster->structure], face->burgersVector());
				auto iter = std::find(materialColors.begin(), materialColors.end(), c);
				if(iter == materialColors.end()) {
					materialIndex = materialColors.size();
					materialColors.push_back(c);
				}
				else materialIndex = iter - materialColors.begin();
			}
		}
		for(Microstructure::Edge* edge = face->edges()->nextFaceEdge()->nextFaceEdge(); edge != face->edges(); edge = edge->nextFaceEdge()) {
			fout->setMaterialIndex(materialIndex);
			++fout;
		}
	}
	OVITO_ASSERT(fout == output.faces().end());

	// Check for early abortion.
	if(promise.isCanceled())
		return false;

	// Convert vertex positions to reduced coordinates.
	for(Point3& p : output.vertices()) {
		p = cell.absoluteToReduced(p);
		OVITO_ASSERT(std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z()));
	}

	// Wrap mesh at periodic boundaries.
	for(size_t dim = 0; dim < 3; dim++) {
		if(cell.pbcFlags()[dim] == false) continue;

		if(promise.isCanceled())
			return false;

		// Make sure all vertices are located inside the periodic box.
		for(Point3& p : output.vertices()) {
			OVITO_ASSERT(std::isfinite(p[dim]));
			p[dim] -= floor(p[dim]);
			OVITO_ASSERT(p[dim] >= FloatType(0) && p[dim] <= FloatType(1));
		}

		// Split triangle faces at periodic boundaries.
		int oldFaceCount = output.faceCount();
		int oldVertexCount = output.vertexCount();
		std::vector<Point3> newVertices;
		std::map<std::pair<int,int>,std::pair<int,int>> newVertexLookupMap;
		for(int findex = 0; findex < oldFaceCount; findex++) {
			if(!splitFace(output, findex, oldVertexCount, newVertices, newVertexLookupMap, cell, dim))
				return false;
		}

		// Insert newly created vertices into mesh.
		output.setVertexCount(oldVertexCount + (int)newVertices.size());
		std::copy(newVertices.cbegin(), newVertices.cend(), output.vertices().begin() + oldVertexCount);
	}

	// Check for early abortion.
	if(promise.isCanceled())
		return false;

	// Convert vertex positions back from reduced coordinates to absolute coordinates.
	AffineTransformation cellMatrix = cell.matrix();
	for(Point3& p : output.vertices())
		p = cellMatrix * p;

	// Clip mesh at cutting planes.
	for(const Plane3& plane : cuttingPlanes) {
		if(promise.isCanceled())
			return false;

		output.clipAtPlane(plane);
	}

	output.invalidateVertices();
	output.invalidateFaces();

	return !promise.isCanceled();
}

/******************************************************************************
* Splits a triangle face at a periodic boundary.
******************************************************************************/
bool SlipSurfaceVis::splitFace(TriMesh& output, int faceIndex, int oldVertexCount, std::vector<Point3>& newVertices,
		std::map<std::pair<int,int>,std::pair<int,int>>& newVertexLookupMap, const SimulationCell& cell, size_t dim)
{
	TriMeshFace& face = output.face(faceIndex);

	// Make sure the face is not degenerate.
	OVITO_ASSERT(face.vertex(0) != face.vertex(1));
	OVITO_ASSERT(face.vertex(1) != face.vertex(2));
	OVITO_ASSERT(face.vertex(2) != face.vertex(0));

	FloatType z[3];
	for(int v = 0; v < 3; v++)
		z[v] = output.vertex(face.vertex(v))[dim];
	FloatType zd[3] = { z[1] - z[0], z[2] - z[1], z[0] - z[2] };

	OVITO_ASSERT(z[1] - z[0] == -(z[0] - z[1]));
	OVITO_ASSERT(z[2] - z[1] == -(z[1] - z[2]));
	OVITO_ASSERT(z[0] - z[2] == -(z[2] - z[0]));

	if(std::abs(zd[0]) < FloatType(0.5) && std::abs(zd[1]) < FloatType(0.5) && std::abs(zd[2]) < FloatType(0.5))
		return true;	// Face is not crossing the periodic boundary.

	// Create four new vertices (or use existing ones created during splitting of adjacent faces).
	int properEdge = -1;
	int newVertexIndices[3][2];
	for(int i = 0; i < 3; i++) {
		if(std::abs(zd[i]) < FloatType(0.5)) {
			if(properEdge != -1)
				return false;		// The simulation box may be too small or invalid.
			properEdge = i;
			continue;
		}
		int vi1 = face.vertex(i);
		int vi2 = face.vertex((i+1)%3);
		int oi1, oi2;
		if(zd[i] <= FloatType(-0.5)) {
			std::swap(vi1, vi2);
			oi1 = 1; oi2 = 0;
		}
		else {
			oi1 = 0; oi2 = 1;
		}
		auto entry = newVertexLookupMap.find(std::make_pair(vi1, vi2));
		if(entry != newVertexLookupMap.end()) {
			newVertexIndices[i][oi1] = entry->second.first;
			newVertexIndices[i][oi2] = entry->second.second;
		}
		else {
			Vector3 delta = output.vertex(vi2) - output.vertex(vi1);
			delta[dim] -= 1.0f;
			for(size_t d = dim + 1; d < 3; d++) {
				if(cell.pbcFlags()[d])
					delta[d] -= floor(delta[d] + FloatType(0.5));
			}
			FloatType t;
			if(delta[dim] != 0)
				t = output.vertex(vi1)[dim] / (-delta[dim]);
			else
				t = FloatType(0.5);
			OVITO_ASSERT(std::isfinite(t));
			Point3 p = delta * t + output.vertex(vi1);
			newVertexIndices[i][oi1] = oldVertexCount + (int)newVertices.size();
			newVertexIndices[i][oi2] = oldVertexCount + (int)newVertices.size() + 1;
			newVertexLookupMap.insert(std::make_pair(std::pair<int,int>(vi1, vi2), std::pair<int,int>(newVertexIndices[i][oi1], newVertexIndices[i][oi2])));
			newVertices.push_back(p);
			p[dim] += FloatType(1.0);
			newVertices.push_back(p);
		}
	}
	OVITO_ASSERT(properEdge != -1);

	// Build output triangles.
	int originalVertices[3] = { face.vertex(0), face.vertex(1), face.vertex(2) };
	face.setVertices(originalVertices[properEdge], originalVertices[(properEdge+1)%3], newVertexIndices[(properEdge+2)%3][1]);

	int materialIndex = face.materialIndex();
	output.setFaceCount(output.faceCount() + 2);
	TriMeshFace& newFace1 = output.face(output.faceCount() - 2);
	TriMeshFace& newFace2 = output.face(output.faceCount() - 1);
	newFace1.setVertices(originalVertices[(properEdge+1)%3], newVertexIndices[(properEdge+1)%3][0], newVertexIndices[(properEdge+2)%3][1]);
	newFace2.setVertices(newVertexIndices[(properEdge+1)%3][1], originalVertices[(properEdge+2)%3], newVertexIndices[(properEdge+2)%3][0]);
	newFace1.setMaterialIndex(materialIndex);
	newFace2.setMaterialIndex(materialIndex);

	return true;
}

}	// End of namespace
}	// End of namespace
}	// End of namespace