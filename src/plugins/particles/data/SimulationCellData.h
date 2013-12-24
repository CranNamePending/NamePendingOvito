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

#ifndef __OVITO_SIMULATION_CELL_DATA_H
#define __OVITO_SIMULATION_CELL_DATA_H

#include <plugins/particles/Particles.h>

namespace Particles {

using namespace Ovito;

/**
* \brief Stores the geometry and boundary conditions of a simulation box.
 *
 * The simulation box geometry is a parallelepiped defined by three edge vectors.
 * A fourth vector specifies the origin of the simulation box in space.
 */
class OVITO_PARTICLES_EXPORT SimulationCellData
{
public:

	/// Returns the current simulation cell matrix.
	const AffineTransformation& matrix() const { return _simulationCell; }

	/// Sets the simulation cell matrix.
	void setMatrix(const AffineTransformation& cellMatrix) {
		_simulationCell = cellMatrix;
		if(!cellMatrix.inverse(_reciprocalSimulationCell))
			_reciprocalSimulationCell.setIdentity();
	}

	/// Returns the PBC flags.
	const std::array<bool,3>& pbcFlags() const { return _pbcFlags; }

	/// Sets the PBC flags.
	void setPbcFlags(const std::array<bool,3>& flags) { _pbcFlags = flags; }

	/// Sets the PBC flags.
	void setPbcFlags(bool pbcX, bool pbcY, bool pbcZ) { _pbcFlags[0] = pbcX; _pbcFlags[1] = pbcY; _pbcFlags[2] = pbcZ; }

	/// Computes the (positive) volume of the cell.
	FloatType volume() const {
		return std::abs(_simulationCell.determinant());
	}

	/// Checks if two simulation cells are identical.
	bool operator==(const SimulationCellData& other) const {
		return (_simulationCell == other._simulationCell && _pbcFlags == other._pbcFlags);
	}

	/// Converts a point given in reduced cell coordinates to a point in absolute coordinates.
	Point3 reducedToAbsolute(const Point3& reducedPoint) const { return _simulationCell * reducedPoint; }

	/// Converts a point given in absolute coordinates to a point in reduced cell coordinates.
	Point3 absoluteToReduced(const Point3& absPoint) const { return _reciprocalSimulationCell * absPoint; }

	/// Converts a vector given in reduced cell coordinates to a vector in absolute coordinates.
	Vector3 reducedToAbsolute(const Vector3& reducedVec) const { return _simulationCell * reducedVec; }

	/// Converts a vector given in absolute coordinates to a point in vector cell coordinates.
	Vector3 absoluteToReduced(const Vector3& absVec) const { return _reciprocalSimulationCell * absVec; }

	/// Wraps a point at the periodic boundaries of the cell.
	Point3 wrapPoint(const Point3& p) const {
		Point3 pout = p;
		for(size_t dim = 0; dim < 3; dim++) {
			if(_pbcFlags[dim]) {
				if(FloatType s = floor(_reciprocalSimulationCell.prodrow(p, dim)))
					pout -= s * _simulationCell.column(dim);
			}
		}
		return pout;
	}

	/// Wraps a vector at the periodic boundaries of the cell using minimum image convention.
	Vector3 wrapVector(const Vector3& v) const {
		Vector3 vout = v;
		for(size_t dim = 0; dim < 3; dim++) {
			if(_pbcFlags[dim]) {
				if(FloatType s = floor(_reciprocalSimulationCell.prodrow(v, dim) + FloatType(0.5)))
					vout -= s * _simulationCell.column(dim);
			}
		}
		return vout;
	}

	/// Calculates the normal vector of the given simulation cell side.
	Vector3 cellNormalVector(size_t dim) const {
		Vector3 normal = _simulationCell.column((dim+1)%3).cross(_simulationCell.column((dim+2)%3));
		// Flip normal if necessary.
		if(normal.dot(_simulationCell.column(dim)) < 0.0f)
			return normal / (-normal.length());
		else
			return normal.normalized();
	}

	/// Tests if a vector so long that it would be wrapped at a periodic boundary when using the minimum image convention.
	bool isWrappedVector(const Vector3& v) const {
		for(size_t dim = 0; dim < 3; dim++) {
			if(_pbcFlags[dim]) {
				if(std::abs(_reciprocalSimulationCell.prodrow(v, dim)) >= 0.5f)
					return true;
			}
		}
		return false;
	}

private:

	/// The geometry of the cell.
	AffineTransformation _simulationCell = AffineTransformation::Zero();

	/// The reciprocal cell matrix.
	AffineTransformation _reciprocalSimulationCell = AffineTransformation::Zero();

	/// PBC flags.
	std::array<bool,3> _pbcFlags = {{ true, true, true }};
};

};

#endif // __OVITO_SIMULATION_CELL_DATA_H