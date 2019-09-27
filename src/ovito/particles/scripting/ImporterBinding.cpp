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

#include <ovito/particles/Particles.h>
#include <ovito/pyscript/binding/PythonBinding.h>
#include <ovito/stdobj/properties/PropertyStorage.h>
#include <ovito/stdobj/simcell/SimulationCellObject.h>
#include <ovito/particles/import/InputColumnMapping.h>
#include <ovito/particles/import/ParticleImporter.h>
#include <ovito/particles/import/cfg/CFGImporter.h>
#include <ovito/particles/import/imd/IMDImporter.h>
#include <ovito/particles/import/parcas/ParcasFileImporter.h>
#include <ovito/particles/import/vasp/POSCARImporter.h>
#include <ovito/particles/import/xyz/XYZImporter.h>
#include <ovito/particles/import/pdb/PDBImporter.h>
#include <ovito/particles/import/lammps/LAMMPSTextDumpImporter.h>
#include <ovito/particles/import/lammps/LAMMPSBinaryDumpImporter.h>
#include <ovito/particles/import/lammps/LAMMPSDataImporter.h>
#include <ovito/particles/import/fhi_aims/FHIAimsImporter.h>
#include <ovito/particles/import/fhi_aims/FHIAimsLogFileImporter.h>
#include <ovito/particles/import/gsd/GSDImporter.h>
#include <ovito/particles/import/xsf/XSFImporter.h>
#include <ovito/particles/import/cube/GaussianCubeImporter.h>
#include <ovito/particles/import/castep/CastepCellImporter.h>
#include <ovito/particles/import/castep/CastepMDImporter.h>
#include <ovito/particles/import/dl_poly/DLPOLYImporter.h>
#include <ovito/particles/import/quantumespresso/QuantumEspressoImporter.h>
#include "PythonBinding.h"

namespace Ovito { namespace Particles { OVITO_BEGIN_INLINE_NAMESPACE(Internal)

using namespace PyScript;

void defineImportersSubmodule(py::module m)
{
	ovito_abstract_class<ParticleImporter, FileSourceImporter>(m)
		.def_property("multiple_frames", &ParticleImporter::isMultiTimestepFile, &ParticleImporter::setMultiTimestepFile)
		.def_property("sort_particles", &ParticleImporter::sortParticles, &ParticleImporter::setSortParticles)
	;

	ovito_class<XYZImporter, ParticleImporter>(m)
		.def_property("columns", &XYZImporter::columnMapping, &XYZImporter::setColumnMapping)
		.def_property("rescale_reduced_coords", &XYZImporter::autoRescaleCoordinates, &XYZImporter::setAutoRescaleCoordinates)
	;

	ovito_class<LAMMPSTextDumpImporter, ParticleImporter>(m)
		.def_property("columns", &LAMMPSTextDumpImporter::customColumnMapping, [](LAMMPSTextDumpImporter& imp, const InputColumnMapping& mapping) {
				imp.setCustomColumnMapping(mapping);
				imp.setUseCustomColumnMapping(true);
		})
	;

	auto LAMMPSDataImporter_py = ovito_class<LAMMPSDataImporter, ParticleImporter>(m)
		.def_property("_atom_style", &LAMMPSDataImporter::atomStyle, &LAMMPSDataImporter::setAtomStyle)
	;
	py::enum_<LAMMPSDataImporter::LAMMPSAtomStyle>(LAMMPSDataImporter_py, "LAMMPSAtomStyle")
		.value("unknown", LAMMPSDataImporter::AtomStyle_Unknown)
		.value("angle", LAMMPSDataImporter::AtomStyle_Angle)
		.value("atomic", LAMMPSDataImporter::AtomStyle_Atomic)
		.value("body", LAMMPSDataImporter::AtomStyle_Body)
		.value("bond", LAMMPSDataImporter::AtomStyle_Bond)
		.value("charge", LAMMPSDataImporter::AtomStyle_Charge)
		.value("full", LAMMPSDataImporter::AtomStyle_Full)
		.value("dipole", LAMMPSDataImporter::AtomStyle_Dipole)
		.value("molecular", LAMMPSDataImporter::AtomStyle_Molecular)
		.value("sphere", LAMMPSDataImporter::AtomStyle_Sphere)
	;

	ovito_class<LAMMPSBinaryDumpImporter, ParticleImporter>(m)
		.def_property("columns", &LAMMPSBinaryDumpImporter::columnMapping, &LAMMPSBinaryDumpImporter::setColumnMapping)
	;

	ovito_class<CFGImporter, ParticleImporter>{m}
	;

	ovito_class<IMDImporter, ParticleImporter>{m}
	;

	ovito_class<ParcasFileImporter, ParticleImporter>{m}
	;

	ovito_class<PDBImporter, ParticleImporter>{m}
	;

	ovito_class<POSCARImporter, ParticleImporter>{m}
	;

	ovito_class<FHIAimsImporter, ParticleImporter>{m}
	;

	ovito_class<FHIAimsLogFileImporter, ParticleImporter>{m}
	;

	ovito_class<GSDImporter, ParticleImporter>{m}
		.def_property("resolution", &GSDImporter::roundingResolution, &GSDImporter::setRoundingResolution)
	;

	ovito_class<CastepCellImporter, ParticleImporter>{m}
	;

	ovito_class<CastepMDImporter, ParticleImporter>{m}
	;

	ovito_class<GaussianCubeImporter, ParticleImporter>{m}
	;

	ovito_class<XSFImporter, ParticleImporter>{m}
	;

	ovito_class<DLPOLYImporter, ParticleImporter>{m}
	;

	ovito_class<QuantumEspressoImporter, ParticleImporter>{m}
	;
}

OVITO_END_INLINE_NAMESPACE
}	// End of namespace
}	// End of namespace