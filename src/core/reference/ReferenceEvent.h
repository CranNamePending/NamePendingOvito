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

#ifndef __OVITO_REFERENCE_EVENT_H
#define __OVITO_REFERENCE_EVENT_H

#include <core/Core.h>

namespace Ovito { OVITO_BEGIN_INLINE_NAMESPACE(ObjectSystem)

/**
 * \brief Generic base class for events generated by a RefTarget object.
 */
class ReferenceEvent
{
public:

	/// Types of events generated by RefTarget objects.
	enum Type {

		/// This event is generated by a reference target when its internal state or parameters have changed in some way.
		TargetChanged,

		/// This notification event is generated by a reference target if has been deleted.
		/// This event is automatically handled by the RefMaker class.
		TargetDeleted,

		/// This event is generated by a RefMaker when one of its reference fields changed.
		ReferenceChanged,

		/// This event is generated by a RefMaker when a new reference has been added to one of its list reference fields.
		ReferenceAdded,

		/// This event is generated by a RefMaker when a reference has been removed from one of its list reference fields.
		ReferenceRemoved,

		/// This event is generated by a RefTarget when its display title changed.
		TitleChanged,

		/// This event is generated by a SceneNode when its transformation controller has generated a TargetChanged event
		/// or the transformation controller has been replaced with a different controller, or if the transformation of a
		/// parent node has changed.
		TransformationChanged,

		/// This event is generated by a ModifierApplication if it has been enabled or disabled.
		TargetEnabledOrDisabled,

		/// This event is generated by a RefTarget when its list of editable sub-objects has changed.
		/// It will be used by the modifier stack widget to update the list of sub-entries.
		SubobjectListChanged,

		/// This event is generated by a data object or modifier when its status has changed.
		ObjectStatusChanged,

		/// This event is sent by an object or modifier in the modification pipeline if pending
		/// results have become available or if the pending operation has been canceled.
		PendingStateChanged,
	};

public:

	/// \brief Constructs the message.
	/// \param type Identifies the type of the notification message.
	/// \param sender The object that generated the event.
	explicit ReferenceEvent(Type type, RefTarget* sender) : _type(type), _sender(sender) {}

	/// \brief Returns the type of the event.
	Type type() const { return _type; }

	/// \brief Returns the RefTarget that has generated this message.
	/// \return The sender of this notification message.
	RefTarget* sender() const { return _sender; }

	/// \brief Returns a flag that indicates whether this type of event should be propagated
	///        by a receiver to its respective dependents.
	bool shouldPropagate() const {
		return type() == ReferenceEvent::TargetChanged ||
			   type() == ReferenceEvent::PendingStateChanged;
	}

private:

	/// The type of event.
	Type _type;

	/// The RefTarget object that has generated the event.
	RefTarget* _sender;
};

/**
 * \brief This type of event is generated by a RefMaker when the pointer stored in one of its reference
 *        fields has been replaced, removed or added.
 */
class ReferenceFieldEvent : public ReferenceEvent
{
public:

	/// Constructor.
	ReferenceFieldEvent(ReferenceEvent::Type type, RefTarget* sender, const PropertyFieldDescriptor& field, RefTarget* oldTarget, RefTarget* newTarget, int index = -1) :
		ReferenceEvent(type, sender), _field(field), _oldvalue(oldTarget), _newvalue(newTarget), _index(index) {}

	/// \brief Returns the reference field that has changed.
	/// \return The descriptor of the changed reference field in the RefTarget that
	///         has generated the message.
	const PropertyFieldDescriptor& field() const { return _field; }

	/// \brief Returns the old target that was stored in the reference field.
	RefTarget* oldTarget() const { return _oldvalue; }

	/// \brief Returns the new target that is now stored in the reference field.
	RefTarget* newTarget() const { return _newvalue; }

	/// \brief The index that has been added or removed if the changed reference field is a vector field.
	/// \return The index into the VectorReferenceField where the entry has been added or removed.
	///         Returns -1 if the reference field is not a vector reference field.
	int index() const { return _index; }

private:

	const PropertyFieldDescriptor& _field;
	RefTarget* _oldvalue;
	RefTarget* _newvalue;
	int _index;
};

OVITO_END_INLINE_NAMESPACE
}	// End of namespace

#endif // __OVITO_REFERENCE_EVENT_H
