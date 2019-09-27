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

#pragma once


#include <ovito/particles/gui/ParticlesGui.h>
#include <ovito/particles/import/xyz/XYZImporter.h>
#include <ovito/gui/dataset/io/FileImporterEditor.h>

namespace Ovito { namespace Particles { OVITO_BEGIN_INLINE_NAMESPACE(Import) OVITO_BEGIN_INLINE_NAMESPACE(Formats) OVITO_BEGIN_INLINE_NAMESPACE(Internal)

/**
 * \brief A properties editor for the XYZImporter class.
 */
class XYZImporterEditor : public FileImporterEditor
{
	OVITO_CLASS(XYZImporterEditor)
	Q_OBJECT

public:

	/// Constructor.
	Q_INVOKABLE XYZImporterEditor() {}

	/// This is called by the system when the user has selected a new file to import.
	virtual bool inspectNewFile(FileImporter* importer, const QUrl& sourceFile, QWidget* parent) override;

	/// Displays a dialog box that allows the user to edit the custom file column to particle property mapping.
	bool showEditColumnMappingDialog(XYZImporter* importer, const QUrl& sourceFile, QWidget* parent);

protected:

	/// Creates the user interface controls for the editor.
	virtual void createUI(const RolloutInsertionParameters& rolloutParams) override;

protected Q_SLOTS:

	/// Is called when the user pressed the "Edit column mapping" button.
	void onEditColumnMapping();
};

OVITO_END_INLINE_NAMESPACE
OVITO_END_INLINE_NAMESPACE
OVITO_END_INLINE_NAMESPACE
}	// End of namespace
}	// End of namespace

