///////////////////////////////////////////////////////////////////////////////
//
//  Copyright (2017) Alexander Stukowski
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

#include <ovito/mesh/Mesh.h>
#include <ovito/stdobj/simcell/SimulationCellObject.h>
#include <ovito/core/dataset/DataSet.h>
#include <ovito/core/dataset/pipeline/ModifierApplication.h>
#include "SurfaceMeshAffineTransformationModifierDelegate.h"

namespace Ovito { namespace Mesh {

IMPLEMENT_OVITO_CLASS(SurfaceMeshAffineTransformationModifierDelegate);

/******************************************************************************
* Applies the modifier operation to the data in a pipeline flow state.
******************************************************************************/
PipelineStatus SurfaceMeshAffineTransformationModifierDelegate::apply(Modifier* modifier, PipelineFlowState& state, TimePoint time, ModifierApplication* modApp, const std::vector<std::reference_wrapper<const PipelineFlowState>>& additionalInputs)
{
	AffineTransformationModifier* mod = static_object_cast<AffineTransformationModifier>(modifier);
	if(mod->selectionOnly())
		return PipelineStatus::Success;

	AffineTransformation tm;
	if(mod->relativeMode())
		tm = mod->transformationTM();
	else
		tm = mod->targetCell() * state.expectObject<SimulationCellObject>()->cellMatrix().inverse();

	for(const DataObject* obj : state.data()->objects()) {
		if(const SurfaceMesh* existingSurface = dynamic_object_cast<SurfaceMesh>(obj)) {
			// Create a copy of the SurfaceMesh.
			SurfaceMesh* newSurface = state.makeMutable(existingSurface);
			// Create a copy of the vertices sub-object (no need to copy the topology when only moving vertices).
			SurfaceMeshVertices* newVertices = newSurface->makeVerticesMutable();
			// Create a copy of the vertex coordinates array.
			PropertyObject* positionProperty = newVertices->expectMutableProperty(SurfaceMeshVertices::PositionProperty);
			// Apply transformation to the vertices coordinates.
			for(Point3& p : positionProperty->point3Range())
				p = tm * p;
			// Apply transformation to the cutting planes attached to the surface mesh.
			QVector<Plane3> cuttingPlanes = newSurface->cuttingPlanes();
			for(Plane3& plane : cuttingPlanes)
				plane = tm * plane;
			newSurface->setCuttingPlanes(std::move(cuttingPlanes));
		}
	}

	return PipelineStatus::Success;
}

}	// End of namespace
}	// End of namespace