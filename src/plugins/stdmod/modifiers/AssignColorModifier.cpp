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

#include <plugins/stdmod/StdMod.h>
#include <plugins/stdobj/properties/PropertyStorage.h>
#include <plugins/stdobj/properties/PropertyObject.h>
#include <plugins/stdobj/properties/PropertyContainer.h>
#include <core/dataset/DataSet.h>
#include <core/dataset/pipeline/ModifierApplication.h>
#include <core/dataset/animation/controller/Controller.h>
#include <core/app/PluginManager.h>
#include "AssignColorModifier.h"

namespace Ovito { namespace StdMod {

IMPLEMENT_OVITO_CLASS(AssignColorModifierDelegate);
DEFINE_PROPERTY_FIELD(AssignColorModifierDelegate, containerPath);

IMPLEMENT_OVITO_CLASS(AssignColorModifier);
DEFINE_REFERENCE_FIELD(AssignColorModifier, colorController);
DEFINE_PROPERTY_FIELD(AssignColorModifier, keepSelection);
SET_PROPERTY_FIELD_LABEL(AssignColorModifier, colorController, "Color");
SET_PROPERTY_FIELD_LABEL(AssignColorModifier, keepSelection, "Keep selection");

/******************************************************************************
* Constructs the modifier object.
******************************************************************************/
AssignColorModifier::AssignColorModifier(DataSet* dataset) : DelegatingModifier(dataset), 
	_keepSelection(true)
{
	setColorController(ControllerManager::createColorController(dataset));
	colorController()->setColorValue(0, Color(0.3f, 0.3f, 1.0f));

	// Let this modifier operate on particles by default.
	createDefaultModifierDelegate(AssignColorModifierDelegate::OOClass(), QStringLiteral("ParticlesAssignColorModifierDelegate"));
}

/******************************************************************************
* Loads the user-defined default values of this object's parameter fields from the
* application's settings store.
******************************************************************************/
void AssignColorModifier::loadUserDefaults()
{
	Modifier::loadUserDefaults();

	// In the graphical program environment, we clear the 
	// selection by default to make the assigned colors visible.
	setKeepSelection(false);
}

/******************************************************************************
* Asks the modifier for its validity interval at the given time.
******************************************************************************/
TimeInterval AssignColorModifier::modifierValidity(TimePoint time)
{
	TimeInterval interval = DelegatingModifier::modifierValidity(time);
	if(colorController()) interval.intersect(colorController()->validityInterval(time));
	return interval;
}

/******************************************************************************
* Applies the modifier operation to the data in a pipeline flow state.
******************************************************************************/
PipelineStatus AssignColorModifierDelegate::apply(Modifier* modifier, PipelineFlowState& state, TimePoint time, ModifierApplication* modApp, const std::vector<std::reference_wrapper<const PipelineFlowState>>& additionalInputs)
{
	const AssignColorModifier* mod = static_object_cast<AssignColorModifier>(modifier);
	if(!mod->colorController())
		return PipelineStatus::Success;

	// Look up the property container object and make sure we can safely modify it.
   	DataObjectPath objectPath = state.expectMutableObject(containerClass(), containerPath());
	PropertyContainer* container = static_object_cast<PropertyContainer>(objectPath.back());
 
	// Get the input selection property.
	ConstPropertyPtr selProperty;
	if(const PropertyObject* selPropertyObj = container->getProperty(PropertyStorage::GenericSelectionProperty)) {
		selProperty = selPropertyObj->storage();

		// Clear selection if requested.
		if(!mod->keepSelection()) {
			container->removeProperty(selPropertyObj);
		}
	}

	// Get modifier's color parameter value.
	Color color;
	mod->colorController()->getColorValue(time, color, state.mutableStateValidity());

	// Create the color output property.
    PropertyObject* colorProperty = container->createProperty(outputColorPropertyId(), (bool)selProperty, objectPath);
	if(!selProperty) {
		// Assign color to all elements.
		std::fill(colorProperty->dataColor(), colorProperty->dataColor() + colorProperty->size(), color);
	}
	else {
		// Assign color only to selected elements.
		const int* sel = selProperty->constDataInt();
		for(Color& c : colorProperty->colorRange()) {
			if(*sel++) c = color;
		}
	}
	
	return PipelineStatus::Success;
}

}	// End of namespace
}	// End of namespace
