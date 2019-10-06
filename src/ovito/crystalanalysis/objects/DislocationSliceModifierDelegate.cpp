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

#include <ovito/crystalanalysis/CrystalAnalysis.h>
#include <ovito/stdobj/simcell/SimulationCellObject.h>
#include <ovito/core/dataset/pipeline/ModifierApplication.h>
#include <ovito/core/dataset/DataSet.h>
#include "DislocationSliceModifierDelegate.h"

namespace Ovito { namespace CrystalAnalysis {

IMPLEMENT_OVITO_CLASS(DislocationSliceModifierDelegate);

/******************************************************************************
* Indicates which data objects in the given input data collection the modifier
* delegate is able to operate on.
******************************************************************************/
QVector<DataObjectReference> DislocationSliceModifierDelegate::OOMetaClass::getApplicableObjects(const DataCollection& input) const
{
	if(input.containsObject<DislocationNetworkObject>())
		return { DataObjectReference(&DislocationNetworkObject::OOClass()) };
	return {};
}

/******************************************************************************
* Performs the actual rejection of particles.
******************************************************************************/
PipelineStatus DislocationSliceModifierDelegate::apply(Modifier* modifier, PipelineFlowState& state, TimePoint time, ModifierApplication* modApp, const std::vector<std::reference_wrapper<const PipelineFlowState>>& additionalInputs)
{
	SliceModifier* mod = static_object_cast<SliceModifier>(modifier);
	if(mod->createSelection())
		return PipelineStatus::Success;

	// Obtain modifier parameter values.
	Plane3 plane;
	FloatType sliceWidth;
	std::tie(plane, sliceWidth) = mod->slicingPlane(time, state.mutableStateValidity());

	for(const DataObject* obj : state.data()->objects()) {
		if(const DislocationNetworkObject* inputDislocations = dynamic_object_cast<DislocationNetworkObject>(obj)) {
			QVector<Plane3> planes = inputDislocations->cuttingPlanes();
			if(sliceWidth <= 0) {
				planes.push_back(plane);
			}
			else {
				planes.push_back(Plane3( plane.normal,  plane.dist + sliceWidth/2));
				planes.push_back(Plane3(-plane.normal, -plane.dist + sliceWidth/2));
			}
			DislocationNetworkObject* outputDislocations = state.makeMutable(inputDislocations);
			outputDislocations->setCuttingPlanes(std::move(planes));
		}
	}

	return PipelineStatus::Success;
}

}	// End of namespace
}	// End of namespace
