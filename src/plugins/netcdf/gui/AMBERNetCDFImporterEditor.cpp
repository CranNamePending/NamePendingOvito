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

///////////////////////////////////////////////////////////////////////////////
//
//  This module implements import of AMBER-style NetCDF trajectory files.
//  For specification documents see <http://ambermd.org/netcdf/>.
//
//  Extensions to this specification are supported through OVITO's manual
//  column mappings.
//
//  A LAMMPS dump style for this file format can be found at
//  <https://github.com/pastewka/lammps-netcdf>.
//
//  An ASE trajectory container is found in ase.io.netcdftrajectory.
//  <https://wiki.fysik.dtu.dk/ase/epydoc/ase.io.netcdftrajectory-module.html>.
//
//  Please contact Lars Pastewka <lars.pastewka@iwm.fraunhofer.de> for
//  questions and suggestions.
//
///////////////////////////////////////////////////////////////////////////////

#include <plugins/particles/gui/ParticlesGui.h>
#include <plugins/particles/gui/import/InputColumnMappingDialog.h>
#include <plugins/netcdf/AMBERNetCDFImporter.h>
#include <gui/properties/BooleanParameterUI.h>
#include <gui/properties/BooleanRadioButtonParameterUI.h>
#include <gui/mainwin/MainWindow.h>
#include <core/dataset/io/FileSource.h>
#include "AMBERNetCDFImporterEditor.h"

namespace Ovito { namespace Particles { OVITO_BEGIN_INLINE_NAMESPACE(Import) OVITO_BEGIN_INLINE_NAMESPACE(Formats) OVITO_BEGIN_INLINE_NAMESPACE(Internal)

IMPLEMENT_OVITO_CLASS(AMBERNetCDFImporterEditor);
SET_OVITO_OBJECT_EDITOR(AMBERNetCDFImporter, AMBERNetCDFImporterEditor);

/******************************************************************************
 * Displays a dialog box that allows the user to edit the custom file column to particle
 * property mapping.
 *****************************************************************************/
bool AMBERNetCDFImporterEditor::showEditColumnMappingDialog(AMBERNetCDFImporter* importer, const QUrl& sourceFile, QWidget* parent)
{
	Future<InputColumnMapping> inspectFuture = importer->inspectFileHeader(FileSourceImporter::Frame(sourceFile));
	if(!importer->dataset()->taskManager().waitForTask(inspectFuture))
		return false;
	InputColumnMapping mapping = inspectFuture.result();

	if(!importer->customColumnMapping().empty()) {
		InputColumnMapping customMapping = importer->customColumnMapping();
		customMapping.resize(mapping.size());
		for(size_t i = 0; i < customMapping.size(); i++)
			customMapping[i].columnName = mapping[i].columnName;
		mapping = customMapping;
	}

	InputColumnMappingDialog dialog(mapping, parent);
	if(dialog.exec() == QDialog::Accepted) {
		importer->setCustomColumnMapping(dialog.mapping());
		importer->setUseCustomColumnMapping(true);
		return true;
	}
	else {
		return false;
	}
}

/******************************************************************************
* Sets up the UI widgets of the editor.
******************************************************************************/
void AMBERNetCDFImporterEditor::createUI(const RolloutInsertionParameters& rolloutParams)
{
	// Create a rollout.
	QWidget* rollout = createRollout(tr("NetCDF file"), rolloutParams);

    // Create the rollout contents.
	QVBoxLayout* layout = new QVBoxLayout(rollout);
	layout->setContentsMargins(4,4,4,4);
	layout->setSpacing(4);

	QGroupBox* optionsBox = new QGroupBox(tr("Options"), rollout);
	QVBoxLayout* sublayout = new QVBoxLayout(optionsBox);
	sublayout->setContentsMargins(4,4,4,4);
	layout->addWidget(optionsBox);

	// Sort particles
	BooleanParameterUI* sortParticlesUI = new BooleanParameterUI(this, PROPERTY_FIELD(ParticleImporter::sortParticles));
	sublayout->addWidget(sortParticlesUI->checkBox());

	QGroupBox* columnMappingBox = new QGroupBox(tr("File columns"), rollout);
	sublayout = new QVBoxLayout(columnMappingBox);
	sublayout->setContentsMargins(4,4,4,4);
	layout->addWidget(columnMappingBox);

	BooleanRadioButtonParameterUI* useCustomMappingUI = new BooleanRadioButtonParameterUI(this, PROPERTY_FIELD(AMBERNetCDFImporter::useCustomColumnMapping));
	useCustomMappingUI->buttonFalse()->setText(tr("Automatic mapping"));
	sublayout->addWidget(useCustomMappingUI->buttonFalse());
	useCustomMappingUI->buttonTrue()->setText(tr("User-defined mapping to particle properties"));
	sublayout->addWidget(useCustomMappingUI->buttonTrue());
	connect(useCustomMappingUI->buttonFalse(), &QRadioButton::clicked, this, [this]() {
		if(AMBERNetCDFImporter* importer = static_object_cast<AMBERNetCDFImporter>(editObject()))
			importer->requestReload();
	}, Qt::QueuedConnection);

	QPushButton* editMappingButton = new QPushButton(tr("Edit column mapping..."));
	sublayout->addWidget(editMappingButton);
	connect(editMappingButton, &QPushButton::clicked, this, &AMBERNetCDFImporterEditor::onEditColumnMapping);
}

/******************************************************************************
* Is called when the user pressed the "Edit column mapping" button.
******************************************************************************/
void AMBERNetCDFImporterEditor::onEditColumnMapping()
{
	if(AMBERNetCDFImporter* importer = static_object_cast<AMBERNetCDFImporter>(editObject())) {

		// Determine URL of current input file.
		FileSource* fileSource = nullptr;
		for(RefMaker* refmaker : importer->dependents()) {
			fileSource = dynamic_object_cast<FileSource>(refmaker);
			if(fileSource) break;
		}
		if(!fileSource || fileSource->frames().empty()) return;

		QUrl sourceUrl;
		if(fileSource->storedFrameIndex() >= 0)
			sourceUrl = fileSource->frames()[fileSource->storedFrameIndex()].sourceFile;
		else
			sourceUrl = fileSource->frames().front().sourceFile;

		UndoableTransaction::handleExceptions(importer->dataset()->undoStack(), tr("Change file column mapping"), [this, &sourceUrl, importer]() {
			if(showEditColumnMappingDialog(importer, sourceUrl, mainWindow())) {
				importer->requestReload();
			}
		});
	}
}

OVITO_END_INLINE_NAMESPACE
OVITO_END_INLINE_NAMESPACE
OVITO_END_INLINE_NAMESPACE
}	// End of namespace
}	// End of namespace
