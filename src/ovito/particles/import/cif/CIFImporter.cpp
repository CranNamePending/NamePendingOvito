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

#include <ovito/particles/Particles.h>
#include <ovito/particles/import/ParticleFrameData.h>
#include <ovito/core/utilities/io/CompressedTextReader.h>
#include "CIFImporter.h"

#include <3rdparty/gemmi/cif.hpp>
#include <3rdparty/gemmi/smcif.hpp>	// for reading small molecules

namespace Ovito { namespace Particles { OVITO_BEGIN_INLINE_NAMESPACE(Import) OVITO_BEGIN_INLINE_NAMESPACE(Formats)

namespace cif = gemmi::cif;

IMPLEMENT_OVITO_CLASS(CIFImporter);

/******************************************************************************
* Checks if the given file has format that can be read by this importer.
******************************************************************************/
bool CIFImporter::OOMetaClass::checkFileFormat(QFileDevice& input, const QUrl& sourceLocation) const
{
	// Open input file.
	CompressedTextReader stream(input, sourceLocation.path());

	// Read the first N lines of the file which are not comments.
	int maxLines = 12;
	bool foundData = false;
	for(int i = 0; i < maxLines && !stream.eof(); i++) {
		// Note: Maximum line length of CIF files is 2048 characters.
		const char* line = stream.readLine(2048);

		if(stream.lineStartsWith("#", true)) {
			maxLines++;
			continue;
		}
		else if(stream.lineStartsWith("data_", false)) {
			// Make sure the "data_XXX" block appears.
			foundData = true;
		}
		else if(stream.lineStartsWith("_", false)) {
			// Make sure at least one "_XXX" block appears.
			return foundData;
		}
	}
	return false;
}

/******************************************************************************
* Parses the given input file.
******************************************************************************/
FileSourceImporter::FrameDataPtr CIFImporter::FrameLoader::loadFile(QFile& file)
{
	// Open file for reading.
	CompressedTextReader stream(file, frame().sourceFile.path());
	setProgressText(tr("Reading CIF file %1").arg(frame().sourceFile.toString(QUrl::RemovePassword | QUrl::PreferLocalFile | QUrl::PrettyDecoded)));

	// Jump to byte offset.
	if(frame().byteOffset != 0)
		stream.seek(frame().byteOffset, frame().lineNumber);

	// Create the destination container for loaded data.
	std::shared_ptr<ParticleFrameData> frameData = std::make_shared<ParticleFrameData>();

	// Map the whole file into memory for parsing.
	const char* buffer_start;
	const char* buffer_end;
	std::tie(buffer_start, buffer_end) = stream.mmap();
	if(!buffer_start)
		throw Exception(tr("Could not map CIF file into memory."));

	try {
		// Parse the CIF file's contents.
		cif::Document doc = cif::read_memory(buffer_start, buffer_end - buffer_start, qPrintable(frame().sourceFile.path()));

		// Unmap the input file from memory.
		stream.munmap();
		if(isCanceled()) return {};

		// Parse the CIF data into an atomic structure representation.
		const cif::Block& block = doc.sole_block();
		gemmi::AtomicStructure structure = gemmi::make_atomic_structure_from_block(block);
		if(isCanceled()) return {};

		// Parse list of atomic sites.
		std::vector<gemmi::AtomicStructure::Site> sites = structure.get_all_unit_cell_sites();
		PropertyPtr posProperty = ParticlesObject::OOClass().createStandardStorage(sites.size(), ParticlesObject::PositionProperty, true);
		frameData->addParticleProperty(posProperty);
		PropertyPtr typeProperty = ParticlesObject::OOClass().createStandardStorage(sites.size(), ParticlesObject::TypeProperty, true);
		frameData->addParticleProperty(typeProperty);
		ParticleFrameData::TypeList* typeList = frameData->propertyTypesList(typeProperty);
		Point3* posIter = posProperty->dataPoint3();
		int* typeIter = typeProperty->dataInt();
		for(const gemmi::AtomicStructure::Site& site : sites) {
			gemmi::Position pos = structure.cell.orthogonalize(site.fract.wrap_to_unit());
			posIter->x() = pos.x;
			posIter->y() = pos.y;
			posIter->z() = pos.z;
			++posIter;
			*typeIter++ = typeList->addTypeName(site.type_symbol.empty() ? site.label.c_str() : site.type_symbol.c_str());
		}
		if(isCanceled()) return {};

		// Since we created particle types on the go while reading the particles, the assigned particle type IDs
		// depend on the storage order of particles in the file We rather want a well-defined particle type ordering, that's
		// why we sort them now.
		typeList->sortTypesByName(typeProperty);

		// Parse unit cell.
		if(structure.cell.is_crystal()) {
			// Process periodic unit cell definition.
			AffineTransformation cell = AffineTransformation::Identity();
			if(structure.cell.alpha == 90 && structure.cell.beta == 90 && structure.cell.gamma == 90) {
				cell(0,0) = structure.cell.a;
				cell(1,1) = structure.cell.b;
				cell(2,2) = structure.cell.c;
			}
			else if(structure.cell.alpha == 90 && structure.cell.beta == 90) {
				FloatType gamma = qDegreesToRadians(structure.cell.gamma);
				cell(0,0) = structure.cell.a;
				cell(0,1) = structure.cell.b * std::cos(gamma);
				cell(1,1) = structure.cell.b * std::sin(gamma);
				cell(2,2) = structure.cell.c;
			}
			else {
				FloatType alpha = qDegreesToRadians(structure.cell.alpha);
				FloatType beta = qDegreesToRadians(structure.cell.beta);
				FloatType gamma = qDegreesToRadians(structure.cell.gamma);
				FloatType v = structure.cell.a * structure.cell.b * structure.cell.c * sqrt(1.0 - std::cos(alpha)*std::cos(alpha) - std::cos(beta)*std::cos(beta) - std::cos(gamma)*std::cos(gamma) + 2.0 * std::cos(alpha) * std::cos(beta) * std::cos(gamma));
				cell(0,0) = structure.cell.a;
				cell(0,1) = structure.cell.b * std::cos(gamma);
				cell(1,1) = structure.cell.b * std::sin(gamma);
				cell(0,2) = structure.cell.c * std::cos(beta);
				cell(1,2) = structure.cell.c * (std::cos(alpha) - std::cos(beta)*std::cos(gamma)) / std::sin(gamma);
				cell(2,2) = v / (structure.cell.a * structure.cell.b * std::sin(gamma));
			}
			frameData->simulationCell().setMatrix(cell);
		}
		else if(posProperty->size() != 0) {
			// Use bounding box of atomic coordinates as non-periodic simulation cell.
			Box3 boundingBox;
			boundingBox.addPoints(posProperty->constDataPoint3(), posProperty->size());
			frameData->simulationCell().setPbcFlags(false, false, false);
			frameData->simulationCell().setMatrix(AffineTransformation(
					Vector3(boundingBox.sizeX(), 0, 0),
					Vector3(0, boundingBox.sizeY(), 0),
					Vector3(0, 0, boundingBox.sizeZ()),
					boundingBox.minc - Point3::Origin()));
		}

		frameData->setStatus(tr("Number of atoms: %1").arg(posProperty->size()));
	}
	catch(const std::exception& e) {
		throw Exception(tr("CIF file reader: %1").arg(e.what()));
	}

	return frameData;
}

OVITO_END_INLINE_NAMESPACE
OVITO_END_INLINE_NAMESPACE
}	// End of namespace
}	// End of namespace