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


#include <plugins/particles/Particles.h>
#include <plugins/particles/modifier/analysis/StructureIdentificationModifier.h>
#include <plugins/particles/objects/ParticlesObject.h>
#include <plugins/stdobj/series/DataSeriesObject.h>

namespace Ovito { namespace Particles { OVITO_BEGIN_INLINE_NAMESPACE(Modifiers) OVITO_BEGIN_INLINE_NAMESPACE(Analysis)

/**
 * \brief A modifier that uses the Polyhedral Template Matching (PTM) method to identify
 *        local coordination structures.
 */
class OVITO_PARTICLES_EXPORT PolyhedralTemplateMatchingModifier : public StructureIdentificationModifier
{
	Q_OBJECT
	OVITO_CLASS(PolyhedralTemplateMatchingModifier)

	Q_CLASSINFO("DisplayName", "Polyhedral template matching");
	Q_CLASSINFO("ModifierCategory", "Structure identification");

public:

#ifndef Q_CC_MSVC
	/// The maximum number of neighbor atoms taken into account for the PTM analysis.
	static constexpr int MAX_NEIGHBORS = 19;
#else
	enum { MAX_NEIGHBORS = 19 };
#endif

	/// The structure types recognized by the PTM library.
	enum StructureType {
		OTHER = 0,			//< Unidentified structure
		FCC,				//< Face-centered cubic
		HCP,				//< Hexagonal close-packed
		BCC,				//< Body-centered cubic
		ICO,				//< Icosahedral structure
		SC,				//< Simple cubic structure
		CUBIC_DIAMOND,			//< Cubic diamond structure
		HEX_DIAMOND,			//< Hexagonal diamond structure
		GRAPHENE,			//< Hexagonal diamond structure

		NUM_STRUCTURE_TYPES 	//< This just counts the number of defined structure types.
	};
	Q_ENUMS(StructureType);

	/// The lattice ordering types recognized by the PTM library.
	enum OrderingType {
		ORDERING_NONE = 0,
		ORDERING_PURE = 1,
		ORDERING_L10 = 2,
		ORDERING_L12_A = 3,
		ORDERING_L12_B = 4,
		ORDERING_B2 = 5,
		ORDERING_ZINCBLENDE_WURTZITE = 6,
		ORDERING_BORON_NITRIDE = 7,

		NUM_ORDERING_TYPES 	//< This just counts the number of defined ordering types.
	};
	Q_ENUMS(OrderingType);

public:

	/// Constructor.
	Q_INVOKABLE PolyhedralTemplateMatchingModifier(DataSet* dataset);

	/// This method indicates whether cached computation results of the modifier should be discarded whenever
	/// a parameter of the modifier changes.
	virtual bool discardResultsOnModifierChange(const PropertyFieldEvent& event) const override { 
		// Avoid a recomputation from scratch if the RMSD cutoff has been changed.
		if(event.field() == &PROPERTY_FIELD(rmsdCutoff)) return false;
		return StructureIdentificationModifier::discardResultsOnModifierChange(event);
	}

protected:

	/// Creates a computation engine that will compute the modifier's results.
	virtual Future<ComputeEnginePtr> createEngine(TimePoint time, ModifierApplication* modApp, const PipelineFlowState& input) override;
	
	/// Is called when the value of a property of this object has changed.
	virtual void propertyChanged(const PropertyFieldDescriptor& field) override;
	
private:
	
	/// Analysis engine that performs the PTM.
	class PTMEngine : public StructureIdentificationEngine
	{
	public:

		/// Constructor.
		PTMEngine(ConstPropertyPtr positions, ParticleOrderingFingerprint fingerprint, ConstPropertyPtr particleTypes, const SimulationCell& simCell,
				QVector<bool> typesToIdentify, ConstPropertyPtr selection,
				bool outputInteratomicDistance, bool outputOrientation, bool outputStandardOrientations, bool outputDeformationGradient, bool outputOrderingTypes) :
			StructureIdentificationEngine(std::move(fingerprint), positions, simCell, std::move(typesToIdentify), std::move(selection)),
			_particleTypes(std::move(particleTypes)),
			_rmsd(std::make_shared<PropertyStorage>(positions->size(), PropertyStorage::Float, 1, 0, tr("RMSD"), false)),
			_interatomicDistances(outputInteratomicDistance ? std::make_shared<PropertyStorage>(positions->size(), PropertyStorage::Float, 1, 0, tr("Interatomic Distance"), true) : nullptr),
			_orientations(outputOrientation ? ParticlesObject::OOClass().createStandardStorage(positions->size(), ParticlesObject::OrientationProperty, true) : nullptr),
			_deformationGradients(outputDeformationGradient ? ParticlesObject::OOClass().createStandardStorage(positions->size(), ParticlesObject::ElasticDeformationGradientProperty, true) : nullptr),
			_orderingTypes(outputOrderingTypes ? std::make_shared<PropertyStorage>(positions->size(), PropertyStorage::Int, 1, 0, tr("Ordering Type"), true) : nullptr),
			_outputStandardOrientations(outputStandardOrientations) {}

		/// This method is called by the system after the computation was successfully completed.
		virtual void cleanup() override {
			_particleTypes.reset();
			StructureIdentificationEngine::cleanup();
		}

		/// Computes the modifier's results.
		virtual void perform() override;

		/// Injects the computed results into the data pipeline.
		virtual void emitResults(TimePoint time, ModifierApplication* modApp, PipelineFlowState& state) override;

		const PropertyPtr& rmsd() const { return _rmsd; }
		const PropertyPtr& interatomicDistances() const { return _interatomicDistances; }
		const PropertyPtr& orientations() const { return _orientations; }
		const PropertyPtr& deformationGradients() const { return _deformationGradients; }
		const PropertyPtr& orderingTypes() const { return _orderingTypes; }
		
		/// Returns the RMSD value range of the histogram.
		FloatType rmsdHistogramRange() const { return _rmsdHistogramRange; }

		/// Returns the histogram of computed RMSD values.
		const PropertyPtr& rmsdHistogram() const { return _rmsdHistogram; }

	protected:
		
		/// Post-processes the per-particle structure types before they are output to the data pipeline.
		virtual PropertyPtr postProcessStructureTypes(TimePoint time, ModifierApplication* modApp, const PropertyPtr& structures) override;

	private:

		ConstPropertyPtr _particleTypes;
		const PropertyPtr _rmsd;
		const PropertyPtr _interatomicDistances;
		const PropertyPtr _orientations;
		const PropertyPtr _deformationGradients;
		const PropertyPtr _orderingTypes;
		PropertyPtr _rmsdHistogram;
		FloatType _rmsdHistogramRange;
		bool _outputStandardOrientations;
	};

private:

	/// The RMSD cutoff.
	DECLARE_MODIFIABLE_PROPERTY_FIELD_FLAGS(FloatType, rmsdCutoff, setRmsdCutoff, PROPERTY_FIELD_MEMORIZE);

	/// Controls the output of the orientation type.
	DECLARE_MODIFIABLE_PROPERTY_FIELD_FLAGS(bool, outputStandardOrientations, setOutputStandardOrientations, PROPERTY_FIELD_MEMORIZE);

	/// Controls the output of the per-particle RMSD values.
	DECLARE_MODIFIABLE_PROPERTY_FIELD(bool, outputRmsd, setOutputRmsd);

	/// Controls the output of local interatomic distances.
	DECLARE_MODIFIABLE_PROPERTY_FIELD_FLAGS(bool, outputInteratomicDistance, setOutputInteratomicDistance, PROPERTY_FIELD_MEMORIZE);

	/// Controls the output of local orientations.
	DECLARE_MODIFIABLE_PROPERTY_FIELD_FLAGS(bool, outputOrientation, setOutputOrientation, PROPERTY_FIELD_MEMORIZE);

	/// Controls the output of elastic deformation gradients.
	DECLARE_MODIFIABLE_PROPERTY_FIELD(bool, outputDeformationGradient, setOutputDeformationGradient);

	/// Controls the output of alloy ordering types.
	DECLARE_MODIFIABLE_PROPERTY_FIELD_FLAGS(bool, outputOrderingTypes, setOutputOrderingTypes, PROPERTY_FIELD_MEMORIZE);

	/// Contains the list of ordering types recognized by this analysis modifier.
	DECLARE_MODIFIABLE_VECTOR_REFERENCE_FIELD(ElementType, orderingTypes, setOrderingTypes);	
};


OVITO_END_INLINE_NAMESPACE
OVITO_END_INLINE_NAMESPACE
}	// End of namespace
}	// End of namespace
