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
#include <core/viewport/Viewport.h>
#include <core/viewport/ViewportManager.h>
#include <core/animation/AnimManager.h>
#include <core/gui/properties/BooleanParameterUI.h>
#include <core/utilities/concurrent/ParallelFor.h>
#include <viz/util/TreeNeighborListBuilder.h>

#include "CommonNeighborAnalysisModifier.h"

namespace Viz {

IMPLEMENT_SERIALIZABLE_OVITO_OBJECT(Viz, CommonNeighborAnalysisModifier, StructureIdentificationModifier)
IMPLEMENT_OVITO_OBJECT(Viz, CommonNeighborAnalysisModifierEditor, ParticleModifierEditor)
SET_OVITO_OBJECT_EDITOR(CommonNeighborAnalysisModifier, CommonNeighborAnalysisModifierEditor)

// The maximum number of neighbor atoms taken into account for the common neighbor analysis.
#define CNA_MAX_PATTERN_NEIGHBORS 16

/******************************************************************************
* Constructs the modifier object.
******************************************************************************/
CommonNeighborAnalysisModifier::CommonNeighborAnalysisModifier()
{
	// Create the structure types.
	createStructureType(OTHER, tr("Other"), Color(0.95f, 0.95f, 0.95f));
	createStructureType(FCC, tr("FCC - Face-centered cubic"), Color(0.4f, 1.0f, 0.4f));
	createStructureType(HCP, tr("HCP - Hexagonal close-packed"), Color(1.0f, 0.4f, 0.4f));
	createStructureType(BCC, tr("BCC - Body-centered cubic"), Color(0.4f, 0.4f, 1.0f));
	createStructureType(ICO, tr("ICO - Icosahedral"), Color(0.95f, 0.8f, 0.2f));
	createStructureType(DIA, tr("DIA - Cubic diamond"), Color(0.2f, 0.95f, 0.8f));
}

/******************************************************************************
* Creates and initializes a computation engine that will compute the modifier's results.
******************************************************************************/
std::shared_ptr<AsynchronousParticleModifier::Engine> CommonNeighborAnalysisModifier::createEngine(TimePoint time)
{
	if(structureTypes().size() != NUM_STRUCTURE_TYPES)
		throw Exception(tr("The number of structure types has changed. Please remove this modifier from the modification pipeline and insert it again."));

	// Get modifier input.
	ParticlePropertyObject* posProperty = expectStandardProperty(ParticleProperty::PositionProperty);
	SimulationCell* simCell = expectSimulationCell();

	return std::make_shared<CommonNeighborAnalysisEngine>(posProperty->storage(), simCell->data());
}

/******************************************************************************
* Performs the actual analysis. This method is executed in a worker thread.
******************************************************************************/
void CommonNeighborAnalysisModifier::CommonNeighborAnalysisEngine::compute(FutureInterfaceBase& futureInterface)
{
	size_t particleCount = positions()->size();
	futureInterface.setProgressText(tr("Performing common neighbor analysis"));

	// Prepare the neighbor list.
	TreeNeighborListBuilder neighborListBuilder(14);
	if(!neighborListBuilder.prepare(positions(), cell()) || futureInterface.isCanceled())
		return;

	// Create output storage.
	ParticleProperty* output = structures();

	// Perform analysis on each particle.
	parallelFor(particleCount, futureInterface, [&neighborListBuilder, output](size_t index) {
		output->setInt(index, determineStructure(neighborListBuilder, index));
	});
}

/// Pair of neighbor atoms that form a bond (bit-wise storage).
typedef unsigned int CNAPairBond;

/**
 * A bit-flag array indicating which pairs of neighbors are bonded
 * and which are not.
 */
struct NeighborBondArray
{
	/// Default constructor.
	NeighborBondArray() {
		memset(neighborArray, 0, sizeof(neighborArray));
	}

	/// Two-dimensional bit array that stores the bonds between neighbors.
	unsigned int neighborArray[CNA_MAX_PATTERN_NEIGHBORS];

	/// Returns whether two nearest neighbors have a bond between them.
	inline bool neighborBond(int neighborIndex1, int neighborIndex2) const {
		OVITO_ASSERT(neighborIndex1 < CNA_MAX_PATTERN_NEIGHBORS);
		OVITO_ASSERT(neighborIndex2 < CNA_MAX_PATTERN_NEIGHBORS);
		return (neighborArray[neighborIndex1] & (1<<neighborIndex2));
	}

	/// Sets whether two nearest neighbors have a bond between them.
	void setNeighborBond(int neighborIndex1, int neighborIndex2, bool bonded) {
		OVITO_ASSERT(neighborIndex1 < CNA_MAX_PATTERN_NEIGHBORS);
		OVITO_ASSERT(neighborIndex2 < CNA_MAX_PATTERN_NEIGHBORS);
		if(bonded) {
			neighborArray[neighborIndex1] |= (1<<neighborIndex2);
			neighborArray[neighborIndex2] |= (1<<neighborIndex1);
		}
		else {
			neighborArray[neighborIndex1] &= ~(1<<neighborIndex2);
			neighborArray[neighborIndex2] &= ~(1<<neighborIndex1);
		}
	}
};

/******************************************************************************
* Find all atoms that are nearest neighbors of the given pair of atoms.
******************************************************************************/
static int findCommonNeighbors(const NeighborBondArray& neighborArray, int neighborIndex, unsigned int& commonNeighbors, int numNeighbors)
{
	commonNeighbors = neighborArray.neighborArray[neighborIndex];
#ifndef Q_CC_MSVC
	// Count the number of bits set in neighbor bit field.
	return __builtin_popcount(commonNeighbors);
#else
	// Count the number of bits set in neighbor bit field.
	unsigned int v = commonNeighbors - ((commonNeighbors >> 1) & 0x55555555);
	v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
	return ((v + (v >> 4) & 0xF0F0F0F) * 0x1010101) >> 24;
#endif
}

/******************************************************************************
* Finds all bonds between common nearest neighbors.
******************************************************************************/
static int findNeighborBonds(const NeighborBondArray& neighborArray, unsigned int commonNeighbors, int numNeighbors, CNAPairBond* neighborBonds)
{
	int numBonds = 0;

	unsigned int nib[CNA_MAX_PATTERN_NEIGHBORS];
	int nibn = 0;
	unsigned int ni1b = 1;
	for(int ni1 = 0; ni1 < numNeighbors; ni1++, ni1b <<= 1) {
		if(commonNeighbors & ni1b) {
			unsigned int b = commonNeighbors & neighborArray.neighborArray[ni1];
			for(int n = 0; n < nibn; n++) {
				if(b & nib[n]) {
					OVITO_ASSERT(numBonds < CNA_MAX_PATTERN_NEIGHBORS*CNA_MAX_PATTERN_NEIGHBORS);
					neighborBonds[numBonds++] = ni1b | nib[n];
				}
			}

			nib[nibn++] = ni1b;
		}
	}
	return numBonds;
}

/******************************************************************************
* Find all chains of bonds.
******************************************************************************/
static int getAdjacentBonds(unsigned int atom, CNAPairBond* bondsToProcess, int& numBonds, unsigned int& atomsToProcess, unsigned int& atomsProcessed)
{
    int adjacentBonds = 0;
	for(int b = numBonds - 1; b >= 0; b--) {
		if(atom & *bondsToProcess) {
            ++adjacentBonds;
   			atomsToProcess |= *bondsToProcess & (~atomsProcessed);
   			memmove(bondsToProcess, bondsToProcess + 1, sizeof(CNAPairBond) * b);
   			numBonds--;
		}
		else ++bondsToProcess;
	}
	return adjacentBonds;
}

/******************************************************************************
* Find all chains of bonds between common neighbors and determine the length
* of the longest continuous chain.
******************************************************************************/
static int calcMaxChainLength(CNAPairBond* neighborBonds, int numBonds)
{
    // Group the common bonds into clusters.
	int maxChainLength = 0;
	while(numBonds) {
        // Make a new cluster starting with the first remaining bond to be processed.
		numBonds--;
        unsigned int atomsToProcess = neighborBonds[numBonds];
        unsigned int atomsProcessed = 0;
		int clusterSize = 1;
        do {
#ifndef Q_CC_MSVC
        	// Determine the number of trailing 0-bits in atomsToProcess, starting at the least significant bit position.
			int nextAtomIndex = __builtin_ctz(atomsToProcess);
#else
			unsigned long nextAtomIndex;
			_BitScanForward(&nextAtomIndex, atomsToProcess);
			OVITO_ASSERT(nextAtomIndex >= 0 && nextAtomIndex < 32);
#endif
			unsigned int nextAtom = 1 << nextAtomIndex;
        	atomsProcessed |= nextAtom;
			atomsToProcess &= ~nextAtom;
			clusterSize += getAdjacentBonds(nextAtom, neighborBonds, numBonds, atomsToProcess, atomsProcessed);
		}
        while(atomsToProcess);
        if(clusterSize > maxChainLength)
        	maxChainLength = clusterSize;
	}
	return maxChainLength;
}

/******************************************************************************
* Determines the coordination structure of a single particle using the common neighbor analysis method.
******************************************************************************/
CommonNeighborAnalysisModifier::StructureType CommonNeighborAnalysisModifier::determineStructure(TreeNeighborListBuilder& neighList, size_t particleIndex)
{
	// Create neighbor list finder.
	TreeNeighborListBuilder::Locator<CNA_MAX_PATTERN_NEIGHBORS> loc(neighList);

	// Find N nearest neighbor of current atom.
	loc.findNeighbors(neighList.particlePos(particleIndex));

	// Early rejection of under-coordinated atoms:
	int numNeighbors = loc.results().size();

	{ /////////// 12 neighbors ///////////

	// Number of neighbors to analyze.
	int nn = 12; // For FCC, HCP and Icosahedral atoms

	// Early rejection of under-coordinated atoms:
	if(numNeighbors < nn)
		return OTHER;

	// Compute scaling factor.
	FloatType localScaling = 0;
	for(int n = 0; n < nn; n++)
		localScaling += sqrt(loc.results()[n].distanceSq);
	FloatType localCutoff = localScaling / nn * (1.0 + sqrt(2.0)) / 2;
	FloatType localCutoffSquared =  localCutoff * localCutoff;

	// Compute common neighbor bit-flag array.
	NeighborBondArray neighborArray;
	for(int ni1 = 0; ni1 < nn; ni1++) {
		neighborArray.setNeighborBond(ni1, ni1, false);
		for(int ni2 = ni1+1; ni2 < nn; ni2++)
			neighborArray.setNeighborBond(ni1, ni2, (loc.results()[ni1].delta - loc.results()[ni2].delta).squaredLength() <= localCutoffSquared);
	}

	int n421 = 0;
	int n422 = 0;
	int n555 = 0;
	for(int ni = 0; ni < nn; ni++) {

		// Determine number of neighbors the two atoms have in common.
		unsigned int commonNeighbors;
		int numCommonNeighbors = findCommonNeighbors(neighborArray, ni, commonNeighbors, nn);
		if(numCommonNeighbors != 4 && numCommonNeighbors != 5)
			break;

		// Determine the number of bonds among the common neighbors.
		CNAPairBond neighborBonds[CNA_MAX_PATTERN_NEIGHBORS*CNA_MAX_PATTERN_NEIGHBORS];
		int numNeighborBonds = findNeighborBonds(neighborArray, commonNeighbors, nn, neighborBonds);
		if(numNeighborBonds != 2 && numNeighborBonds != 5)
			break;

		// Determine the number of bonds in the longest continuous chain.
		int maxChainLength = calcMaxChainLength(neighborBonds, numNeighborBonds);
		if(numCommonNeighbors == 4 && numNeighborBonds == 2) {
			if(maxChainLength == 1) n421++;
			else if(maxChainLength == 2) n422++;
			else break;
		}
		else if(numCommonNeighbors == 5 && numNeighborBonds == 5 && maxChainLength == 5) n555++;
		else break;
	}
	if(n421 == 12) return FCC;
	else if(n421 == 6 && n422 == 6) return HCP;
	else if(n555 == 12) return ICO;

	}

	{ /////////// 14 neighbors ///////////

	// Number of neighbors to analyze.
	int nn = 14; // For BCC atoms

	// Early rejection of under-coordinated atoms:
	if(numNeighbors < nn)
		return OTHER;

	// Compute scaling factor.
	FloatType localScaling = 0;
	for(int n = 0; n < 8; n++)
		localScaling += sqrt(loc.results()[n].distanceSq / (3.0/4.0));
	for(int n = 8; n < 14; n++)
		localScaling += sqrt(loc.results()[n].distanceSq);
	FloatType localCutoff = localScaling / nn * 1.207;
	FloatType localCutoffSquared =  localCutoff * localCutoff;

	// Compute common neighbor bit-flag array.
	NeighborBondArray neighborArray;
	for(int ni1 = 0; ni1 < nn; ni1++) {
		neighborArray.setNeighborBond(ni1, ni1, false);
		for(int ni2 = ni1+1; ni2 < nn; ni2++)
			neighborArray.setNeighborBond(ni1, ni2, (loc.results()[ni1].delta - loc.results()[ni2].delta).squaredLength() <= localCutoffSquared);
	}

	int n444 = 0;
	int n666 = 0;
	for(int ni = 0; ni < nn; ni++) {

		// Determine number of neighbors the two atoms have in common.
		unsigned int commonNeighbors;
		int numCommonNeighbors = findCommonNeighbors(neighborArray, ni, commonNeighbors, nn);
		if(numCommonNeighbors != 4 && numCommonNeighbors != 6)
			break;

		// Determine the number of bonds among the common neighbors.
		CNAPairBond neighborBonds[CNA_MAX_PATTERN_NEIGHBORS*CNA_MAX_PATTERN_NEIGHBORS];
		int numNeighborBonds = findNeighborBonds(neighborArray, commonNeighbors, nn, neighborBonds);
		if(numNeighborBonds != 4 && numNeighborBonds != 6)
			break;

		// Determine the number of bonds in the longest continuous chain.
		int maxChainLength = calcMaxChainLength(neighborBonds, numNeighborBonds);
		if(numCommonNeighbors == 4 && numNeighborBonds == 4 && maxChainLength == 4) n444++;
		else if(numCommonNeighbors == 6 && numNeighborBonds == 6 && maxChainLength == 6) n666++;
		else break;
	}
	if(n666 == 8 && n444 == 6) return BCC;

	}

	{ /////////// 16 neighbors ///////////

	// Number of neighbors to analyze.
	int nn = 16; // For BCC atoms

	// Early rejection of under-coordinated atoms:
	if(numNeighbors < nn)
		return OTHER;

	// Compute scaling factor.
	FloatType localScaling = 0;
	for(int n = 0; n < 4; n++)
		localScaling += sqrt(loc.results()[n].distanceSq / (3.0/16.0));
	for(int n = 4; n < 16; n++)
		localScaling += sqrt(loc.results()[n].distanceSq / (2.0/4.0));
	FloatType localCutoff = localScaling / nn * 0.7681;
	FloatType localCutoffSquared =  localCutoff * localCutoff;

	// Compute common neighbor bit-flag array.
	NeighborBondArray neighborArray;
	for(int ni1 = 0; ni1 < nn; ni1++) {
		neighborArray.setNeighborBond(ni1, ni1, false);
		for(int ni2 = ni1+1; ni2 < nn; ni2++)
			neighborArray.setNeighborBond(ni1, ni2, (loc.results()[ni1].delta - loc.results()[ni2].delta).squaredLength() <= localCutoffSquared);
	}

	int n543 = 0;
	int n663 = 0;
	for(int ni = 0; ni < nn; ni++) {

		// Determine number of neighbors the two atoms have in common.
		unsigned int commonNeighbors;
		int numCommonNeighbors = findCommonNeighbors(neighborArray, ni, commonNeighbors, nn);
		if(numCommonNeighbors != 5 && numCommonNeighbors != 6)
			break;

		// Determine the number of bonds among the common neighbors.
		CNAPairBond neighborBonds[CNA_MAX_PATTERN_NEIGHBORS*CNA_MAX_PATTERN_NEIGHBORS];
		int numNeighborBonds = findNeighborBonds(neighborArray, commonNeighbors, nn, neighborBonds);
		if(numNeighborBonds != 4 && numNeighborBonds != 6)
			break;

		// Determine the number of bonds in the longest continuous chain.
		int maxChainLength = calcMaxChainLength(neighborBonds, numNeighborBonds);
		if(numCommonNeighbors == 5 && numNeighborBonds == 4 && maxChainLength == 3) n543++;
		else if(numCommonNeighbors == 6 && numNeighborBonds == 6 && maxChainLength == 3) n663++;
		else break;
	}
	if(n543 == 12 && n663 == 4) return DIA;

	}

	return OTHER;
}

/******************************************************************************
* Sets up the UI widgets of the editor.
******************************************************************************/
void CommonNeighborAnalysisModifierEditor::createUI(const RolloutInsertionParameters& rolloutParams)
{
	// Create a rollout.
	QWidget* rollout = createRollout(tr("Common neighbor analysis"), rolloutParams);

    // Create the rollout contents.
	QVBoxLayout* layout1 = new QVBoxLayout(rollout);
#ifndef Q_WS_MAC
	layout1->setContentsMargins(4,4,4,4);
	layout1->setSpacing(0);
#endif

	BooleanParameterUI* autoUpdateUI = new BooleanParameterUI(this, PROPERTY_FIELD(AsynchronousParticleModifier::_autoUpdate));
	layout1->addWidget(autoUpdateUI->checkBox());

	// Status label.
	layout1->addSpacing(10);
	layout1->addWidget(statusLabel());

	StructureListParameterUI* structureTypesPUI = new StructureListParameterUI(this);
	layout1->addSpacing(10);
	layout1->addWidget(new QLabel(tr("Structure types:")));
	layout1->addWidget(structureTypesPUI->tableWidget());
	layout1->addWidget(new QLabel(tr("(Double-click to change colors)")));
}


};	// End of namespace