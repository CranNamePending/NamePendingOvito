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

#include <ovito/mesh/Mesh.h>
#include <ovito/mesh/surface/SurfaceMesh.h>
#include <ovito/stdobj/properties/PropertyExpressionEvaluator.h>
#include "SurfaceMeshExpressionSelectionModifierDelegate.h"

namespace Ovito { namespace Mesh {

IMPLEMENT_OVITO_CLASS(SurfaceMeshRegionsExpressionSelectionModifierDelegate);

/******************************************************************************
* Indicates which data objects in the given input data collection the modifier
* delegate is able to operate on.
******************************************************************************/
QVector<DataObjectReference> SurfaceMeshRegionsExpressionSelectionModifierDelegate::OOMetaClass::getApplicableObjects(const DataCollection& input) const
{
	// Gather list of all surface mesh regions in the input data collection.
	QVector<DataObjectReference> objects;
	for(const ConstDataObjectPath& path : input.getObjectsRecursive(SurfaceMeshRegions::OOClass())) {
		objects.push_back(path);
	}
	return objects;
}

/******************************************************************************
* Creates and initializes the expression evaluator object.
******************************************************************************/
std::unique_ptr<PropertyExpressionEvaluator> SurfaceMeshRegionsExpressionSelectionModifierDelegate::initializeExpressionEvaluator(const QStringList& expressions, const PipelineFlowState& inputState, const DataObjectPath& objectPath, int animationFrame)
{
	const PropertyContainer* container = static_object_cast<PropertyContainer>(objectPath.back());
	std::unique_ptr<PropertyExpressionEvaluator> evaluator = std::make_unique<PropertyExpressionEvaluator>();
	evaluator->initialize(expressions, inputState, container, animationFrame);
	return evaluator;
}

}	// End of namespace
}	// End of namespace
