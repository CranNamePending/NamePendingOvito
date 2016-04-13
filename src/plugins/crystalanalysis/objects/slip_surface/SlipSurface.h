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

#ifndef __OVITO_SLIP_SURFACE_H
#define __OVITO_SLIP_SURFACE_H

#include <plugins/crystalanalysis/CrystalAnalysis.h>
#include <core/scene/objects/DataObjectWithSharedStorage.h>
#include <core/utilities/mesh/HalfEdgeMesh.h>
#include <plugins/particles/data/SimulationCell.h>
#include <core/utilities/concurrent/FutureInterface.h>
#include <plugins/crystalanalysis/data/ClusterVector.h>

namespace Ovito { namespace Plugins { namespace CrystalAnalysis {

struct SlipSurfaceFace
{
	/// The local slip vector.
	ClusterVector slipVector = Vector3::Zero();
};

using SlipSurfaceData = HalfEdgeMesh<EmptyHalfEdgeMeshStruct, SlipSurfaceFace, EmptyHalfEdgeMeshStruct>;

/**
 * \brief A triangle mesh representing the slipped surfaces in a deformed crystal.
 */
class OVITO_CRYSTALANALYSIS_EXPORT SlipSurface : public DataObjectWithSharedStorage<SlipSurfaceData>
{
public:

	/// \brief Constructor.
	Q_INVOKABLE SlipSurface(DataSet* dataset, SlipSurfaceData* data = nullptr);

	/// Returns the title of this object.
	virtual QString objectTitle() override { return tr("Slip surface"); }

	/// \brief Returns whether this object, when returned as an editable sub-object by another object,
	///        should be displayed in the modification stack.
	///
	/// Return false because this object cannot be edited.
	virtual bool isSubObjectEditable() const override { return false; }

	/// Returns the planar cuts applied to this mesh.
	const QVector<Plane3>& cuttingPlanes() const { return _cuttingPlanes; }

	/// Sets the planar cuts applied to this mesh.
	void setCuttingPlanes(const QVector<Plane3>& planes) {
		_cuttingPlanes = planes;
		notifyDependents(ReferenceEvent::TargetChanged);
	}

protected:

	/// Creates a copy of this object.
	virtual OORef<RefTarget> clone(bool deepCopy, CloneHelper& cloneHelper) override;

private:

	/// The planar cuts applied to this mesh.
	QVector<Plane3> _cuttingPlanes;

	Q_OBJECT
	OVITO_OBJECT
};

}	// End of namespace
}	// End of namespace
}	// End of namespace

#endif // __OVITO_SLIP_SURFACE_H