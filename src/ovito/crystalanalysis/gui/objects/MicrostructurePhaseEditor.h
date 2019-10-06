///////////////////////////////////////////////////////////////////////////////
//
//  Copyright (2019) Alexander Stukowski
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


#include <ovito/crystalanalysis/CrystalAnalysis.h>
#include <ovito/gui/properties/PropertiesEditor.h>

namespace Ovito { namespace CrystalAnalysis {

/**
 * \brief A properties editor for the MicrostructurePhase class.
 */
class MicrostructurePhaseEditor : public PropertiesEditor
{
	Q_OBJECT
	OVITO_CLASS(MicrostructurePhaseEditor)

public:

	/// Default constructor.
	Q_INVOKABLE MicrostructurePhaseEditor() {}

protected:

	/// Creates the user interface controls for the editor.
	virtual void createUI(const RolloutInsertionParameters& rolloutParams) override;

protected Q_SLOTS:

	/// Is called when the user has double-clicked on one of the entries in the list widget.
	void onDoubleClickBurgersFamily(const QModelIndex& index);

private:

	RefTargetListParameterUI* familiesListUI;
};

}	// End of namespace
}	// End of namespace
