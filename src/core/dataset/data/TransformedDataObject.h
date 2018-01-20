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

#pragma once


#include <core/Core.h>
#include <core/dataset/data/DataObject.h>
#include <core/dataset/data/VersionedDataObjectRef.h>
#include <core/dataset/data/TransformingDisplayObject.h>

namespace Ovito { OVITO_BEGIN_INLINE_NAMESPACE(ObjectSystem) OVITO_BEGIN_INLINE_NAMESPACE(Scene)
	
/**
 * \brief Base class for transient data objects that are dynamically generated from other data objects
 *        by an asynchronous DisplayObject implementation.
 */
class OVITO_CORE_EXPORT TransformedDataObject : public DataObject
{
	Q_OBJECT
	OVITO_CLASS(TransformedDataObject)
	
public:

	/// \brief Standard constructor.
	TransformedDataObject(DataSet* dataset) : DataObject(dataset) {}

	/// \brief Initialization constructor.
	TransformedDataObject(TransformingDisplayObject* creator, DataObject* sourceData) :
		DataObject(creator->dataset()),
		_sourceDataObject(sourceData),
		_generatorDisplayObjectRevision(creator->revisionNumber()) {
			setDisplayObject(creator);
		}

	/// Indicate that this transient data object cannot be edited.
	virtual bool isSubObjectEditable() const override { return false; }

private:

	/// Stores a weak reference to + revision version number of the original DataObject 
	/// this TransformedDataObject was derived from. 
	/// We use this detected changes to the source object and avoid unnecessary regeneration 
	/// of the transient data object.
	DECLARE_RUNTIME_PROPERTY_FIELD(VersionedDataObjectRef, sourceDataObject, setSourceDataObject);

	/// Stores a revision version number of the TransformingDisplayObject that created this TransformedDataObject. 
	/// We use this detected changes to the TransformingDisplayObject's parameters that would require a re-generation of the
	/// transient data object.
	DECLARE_RUNTIME_PROPERTY_FIELD(unsigned int, generatorDisplayObjectRevision, setGeneratorDisplayObjectRevision);
};

OVITO_END_INLINE_NAMESPACE
OVITO_END_INLINE_NAMESPACE
}	// End of namespace