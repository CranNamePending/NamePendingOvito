///////////////////////////////////////////////////////////////////////////////
// 
//  Copyright (2013) Alexander Stukowski
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

#include <core/Core.h>
#include <core/scene/objects/SceneObject.h>

namespace Ovito {

IMPLEMENT_SERIALIZABLE_OVITO_OBJECT(Core, SceneObject, RefTarget)
DEFINE_PROPERTY_FIELD(SceneObject, _saveWithScene, "SaveWithScene")
DEFINE_VECTOR_REFERENCE_FIELD(SceneObject, _displayObjects, "DisplayObjects", DisplayObject)
SET_PROPERTY_FIELD_LABEL(SceneObject, _saveWithScene, "Save data with scene")
SET_PROPERTY_FIELD_LABEL(SceneObject, _displayObjects, "Display objects")

/******************************************************************************
* Constructor.
******************************************************************************/
SceneObject::SceneObject(DataSet* dataset) : RefTarget(dataset), _revisionNumber(0), _saveWithScene(true)
{
	INIT_PROPERTY_FIELD(SceneObject::_saveWithScene);
	INIT_PROPERTY_FIELD(SceneObject::_displayObjects);
}

/******************************************************************************
* Sends an event to all dependents of this RefTarget.
******************************************************************************/
void SceneObject::notifyDependents(ReferenceEvent& event)
{
	// Automatically increment revision counter each time the object changes.
	if(event.type() == ReferenceEvent::TargetChanged)
		_revisionNumber++;

	RefTarget::notifyDependents(event);
}

/******************************************************************************
* Handles reference events sent by reference targets of this object.
******************************************************************************/
bool SceneObject::referenceEvent(RefTarget* source, ReferenceEvent* event)
{
	if(event->type() == ReferenceEvent::TargetChanged) {

		// Do not propagate messages generated by the display objects.
		if(displayObjects().contains(static_cast<DisplayObject*>(source)))
			return false;

		// Automatically increment revision counter each time a sub-object of this object changes.
		_revisionNumber++;
	}

	return RefTarget::referenceEvent(source, event);
}

/******************************************************************************
* Saves the class' contents to the given stream.
******************************************************************************/
void SceneObject::saveToStream(ObjectSaveStream& stream)
{
	RefTarget::saveToStream(stream);
	stream.beginChunk(0x02);
	stream.endChunk();
}

/******************************************************************************
* Loads the class' contents from the given stream.
******************************************************************************/
void SceneObject::loadFromStream(ObjectLoadStream& stream)
{
	RefTarget::loadFromStream(stream);
	int formatVersion = stream.expectChunkRange(0, 0x02);
	if(formatVersion == 0x01) {
		OORef<DisplayObject> displayObject = stream.loadObject<DisplayObject>();
		if(displayObject) {
			_displayObjects.clear();
			addDisplayObject(displayObject);
		}
	}
	stream.closeChunk();
}

};
