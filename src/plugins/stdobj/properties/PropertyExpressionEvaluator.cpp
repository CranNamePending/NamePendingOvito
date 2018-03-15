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

#include <plugins/stdobj/StdObj.h>
#include <plugins/stdobj/properties/PropertyObject.h>
#include <plugins/stdobj/simcell/SimulationCellObject.h>
#include <core/app/Application.h>
#include "PropertyExpressionEvaluator.h"

#include <QtConcurrent>

namespace Ovito { namespace StdObj {

/// List of characters allowed in variable names.
QByteArray PropertyExpressionEvaluator::_validVariableNameChars("0123456789_abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ.");

/******************************************************************************
* Specifies the expressions to be evaluated for each data element and create the
* list of input variables.
******************************************************************************/
void PropertyExpressionEvaluator::initialize(const QStringList& expressions, const PipelineFlowState& inputState, const PropertyClass& propertyClass, int animationFrame)
{
	// Build list of particle properties.
	std::vector<ConstPropertyPtr> inputProperties;
	for(DataObject* obj : inputState.objects()) {
		if(PropertyObject* prop = dynamic_object_cast<PropertyObject>(obj)) {
			if(propertyClass.isMember(prop)) {
				inputProperties.push_back(prop->storage());
			}
		}
	}

	// Get simulation cell.
	SimulationCell simCell;
	SimulationCellObject* simCellObj = inputState.findObject<SimulationCellObject>();
	if(simCellObj) simCell = simCellObj->data();

	// Call overloaded function.
	initialize(expressions, inputProperties, simCellObj ? &simCell : nullptr, inputState.attributes(), animationFrame);
}

/******************************************************************************
* Specifies the expressions to be evaluated for each data element and create the
* list of input variables.
******************************************************************************/
void PropertyExpressionEvaluator::initialize(const QStringList& expressions, const std::vector<ConstPropertyPtr>& inputProperties, const SimulationCell* simCell, const QVariantMap& attributes, int animationFrame)
{
	// Create list of input variables.
	createInputVariables(inputProperties, simCell, attributes, animationFrame);

	// Copy expression strings into internal array.
	_expressions.resize(expressions.size());
	std::transform(expressions.begin(), expressions.end(), _expressions.begin(), [](const QString& s) -> std::string { return s.toStdString(); });

	// Determine number of input element.
	_elementCount = inputProperties.empty() ? 0 : inputProperties.front()->size();
	_isTimeDependent = false;
}

/******************************************************************************
* Initializes the list of input variables from the given input state.
******************************************************************************/
void PropertyExpressionEvaluator::createInputVariables(const std::vector<ConstPropertyPtr>& inputProperties, const SimulationCell* simCell, const QVariantMap& attributes, int animationFrame)
{
	_inputVariables.clear();

	int propertyIndex = 1;
	size_t elementCount = 0;
	for(const ConstPropertyPtr& property : inputProperties) {
		ExpressionVariable v;

		// Properties with custom data type are not supported by this modifier.
		if(property->dataType() == PropertyStorage::Int)
			v.type = INT_PROPERTY;
		else if(property->dataType() == PropertyStorage::Int64)
			v.type = INT64_PROPERTY;
		else if(property->dataType() == PropertyStorage::Float)
			v.type = FLOAT_PROPERTY;
		else
			continue;
		v.property = property;
		elementCount = property->size();

		// Derive a valid variable name from the property name by removing all invalid characters.
		QString propertyName = property->name();
		// If the name is empty, generate one.
		if(propertyName.isEmpty())
			propertyName = QString("Property%1").arg(propertyIndex);
		// If the name starts with a number, prepend an underscore.
		else if(propertyName[0].isDigit())
			propertyName.prepend(QChar('_'));

		for(size_t k = 0; k < property->componentCount(); k++) {

			QString fullPropertyName = propertyName;
			if(property->componentNames().size() == property->componentCount())
				fullPropertyName += "." + property->componentNames()[k];

			// Filter out invalid characters.
			v.name = fullPropertyName.toStdString();

			// Initialize data pointer into particle property storage.
			if(property->dataType() == PropertyStorage::Int)
				v.dataPointer = reinterpret_cast<const char*>(property->constDataInt() + k);
			else if(property->dataType() == PropertyStorage::Int64)
				v.dataPointer = reinterpret_cast<const char*>(property->constDataInt64() + k);
			else
				v.dataPointer = reinterpret_cast<const char*>(property->constDataFloat() + k);
			v.stride = property->stride();

			addVariable(ExpressionVariable(v));
		}

		propertyIndex++;
	}

	// Create index variable.
	if(!_indexVarName.empty()) {
		ExpressionVariable pindexVar;
		pindexVar.name = _indexVarName;
		pindexVar.type = ELEMENT_INDEX;
		pindexVar.description = tr("zero-based");
		addVariable(std::move(pindexVar));
	}

	// Create constant variables.
	ExpressionVariable constVar;

	// Number of element
	registerGlobalParameter("N", elementCount, tr("number of particles"));

	// Animation frame
	registerGlobalParameter("Frame", animationFrame, tr("animation frame number"));

	// Global attributes.
	for(auto entry = attributes.constBegin(); entry != attributes.constEnd(); ++entry) {
		if(entry.value().canConvert<double>())
			registerGlobalParameter(entry.key(), entry.value().toDouble());
		else if(entry.value().canConvert<long>())
			registerGlobalParameter(entry.key(), entry.value().value<long>());
	}

	if(simCell) {
		// Cell volume
		registerGlobalParameter("CellVolume", simCell->volume3D(), tr("simulation cell volume"));

		// Cell size
		registerGlobalParameter("CellSize.X", std::abs(simCell->matrix().column(0).x()), tr("size along X"));
		registerGlobalParameter("CellSize.Y", std::abs(simCell->matrix().column(1).y()), tr("size along Y"));
		registerGlobalParameter("CellSize.Z", std::abs(simCell->matrix().column(2).z()), tr("size along Z"));
	}

	// Constant pi.
	registerConstant("pi", M_PI, QStringLiteral("%1...").arg(M_PI));
}

/******************************************************************************
* Registers an input variable if the name does not exist yet.
******************************************************************************/
void PropertyExpressionEvaluator::addVariable(ExpressionVariable&& v)
{
	// Replace invalid characters in variable name with an underscore.
	std::string filteredName;
	filteredName.reserve(v.name.size());
	for(char c : v.name) {
		if(c != ' ')
			filteredName.push_back(_validVariableNameChars.contains(c) ? c : '_');
	}
	if(filteredName.empty()) return;
	v.name.swap(filteredName);
	
	// Check if name is unique.
	if(std::none_of(_inputVariables.begin(), _inputVariables.end(), [&v](const ExpressionVariable& v2) -> bool { return v2.name == v.name; }))
		_inputVariables.push_back(std::move(v));
}

/******************************************************************************
* Returns the list of available input variables.
******************************************************************************/
QStringList PropertyExpressionEvaluator::inputVariableNames() const
{
	QStringList vlist;
	for(const ExpressionVariable& v : _inputVariables)
		vlist << QString::fromStdString(v.name);
	return vlist;
}

/******************************************************************************
* Initializes the parser object and evaluates the expressions for every data element
******************************************************************************/
void PropertyExpressionEvaluator::evaluate(const std::function<void(size_t,size_t,double)>& callback, const std::function<bool(size_t)>& filter)
{
	// Make sure initialize() has been called.
	OVITO_ASSERT(!_inputVariables.empty());

	// Determine the number of parallel threads to use.
	size_t nthreads = Application::instance()->idealThreadCount();
	if(maxThreadCount() != 0 && nthreads > maxThreadCount()) nthreads = maxThreadCount();
	if(elementCount() == 0)
		return;
	else if(elementCount() < 100)
		nthreads = 1;
	else if(nthreads > elementCount())
		nthreads = elementCount();

	if(nthreads == 1) {
		Worker worker(*this);
		worker.run(0, elementCount(), callback, filter);
		if(worker._errorMsg.isEmpty() == false)
			throw Exception(worker._errorMsg);
	}
	else if(nthreads > 1) {
		std::vector<std::unique_ptr<Worker>> workers;
		workers.reserve(nthreads);
		for(size_t i = 0; i < nthreads; i++)
			workers.emplace_back(new Worker(*this));

		// Spawn worker threads.
		QFutureSynchronizer<void> synchronizer;
		size_t chunkSize = elementCount() / nthreads;
		OVITO_ASSERT(chunkSize > 0);
		for(size_t i = 0; i < workers.size(); i++) {
			// Setup data range.
			size_t startIndex = chunkSize * i;
			size_t endIndex = startIndex + chunkSize;
			if(i == workers.size() - 1) endIndex = elementCount();
			OVITO_ASSERT(endIndex > startIndex);
			OVITO_ASSERT(endIndex <= elementCount());
			synchronizer.addFuture(QtConcurrent::run(workers[i].get(), &Worker::run, startIndex, endIndex, callback, filter));
		}
		synchronizer.waitForFinished();

		// Check for errors.
		for(auto& worker : workers) {
			if(worker->_errorMsg.isEmpty() == false)
				throw Exception(worker->_errorMsg);
		}
	}
}

/******************************************************************************
* Initializes the parser objects of this thread.
******************************************************************************/
PropertyExpressionEvaluator::Worker::Worker(PropertyExpressionEvaluator& evaluator)
{
	_parsers.resize(evaluator._expressions.size());
	_inputVariables = evaluator._inputVariables;

	// The list of used variables.
	std::set<std::string> usedVariables;

	auto parser = _parsers.begin();
	auto expr = evaluator._expressions.cbegin();
	for(size_t i = 0; i < evaluator._expressions.size(); i++, ++parser, ++expr) {

		if(expr->empty()) {
			if(evaluator._expressions.size() > 1)
				throw Exception(tr("Expression %1 is empty.").arg(i+1));
			else
				throw Exception(tr("Expression is empty."));
		}

		try {

			// Configure parser to accept alpha-numeric characters and '.' in variable names.
			parser->DefineNameChars(_validVariableNameChars.constData());

			// Define some extra math functions.
			parser->DefineFun("fmod", static_cast<double (*)(double,double)>(fmod), false);

			// Let the muParser process the math expression.
			parser->SetExpr(*expr);

			// Register input variables.
			for(auto& v : _inputVariables)
				parser->DefineVar(v.name, &v.value);

			// Query list of used variables.
			for(const auto& vname : parser->GetUsedVar())
				usedVariables.insert(vname.first);
		}
		catch(mu::Parser::exception_type& ex) {
			throw Exception(QString::fromStdString(ex.GetMsg()));
		}
	}

	// If the current animation time is used in the math expression then we have to
	// reduce the validity interval to the current time only.
	if(usedVariables.find("Frame") != usedVariables.end() || usedVariables.find("Timestep") != usedVariables.end())
		evaluator._isTimeDependent = true;

	// Remove unused variables so they don't get updated unnecessarily.
	for(ExpressionVariable& var : _inputVariables) {
		if(usedVariables.find(var.name) != usedVariables.end())
			_activeVariables.push_back(&var);
	}
}

/******************************************************************************
* The worker routine.
******************************************************************************/
void PropertyExpressionEvaluator::Worker::run(size_t startIndex, size_t endIndex, std::function<void(size_t,size_t,double)> callback, std::function<bool(size_t)> filter)
{
	try {
		for(size_t i = startIndex; i < endIndex; i++) {
			if(filter && !filter(i))
				continue;

			for(size_t j = 0; j < _parsers.size(); j++) {
				// Evaluate expression for the current data element.
				callback(i, j, evaluate(i, j));
			}
		}
	}
	catch(const Exception& ex) {
		_errorMsg = ex.message();
	}
}

/******************************************************************************
* The innermost evaluation routine.
******************************************************************************/
double PropertyExpressionEvaluator::Worker::evaluate(size_t elementIndex, size_t component)
{
	OVITO_ASSERT(component < _parsers.size());
	try {
		if(elementIndex != _lastElementIndex) {
			_lastElementIndex = elementIndex;

			// Update variable values for the current data element.
			for(ExpressionVariable* v : _activeVariables) {
				if(v->type == FLOAT_PROPERTY) {
					v->value = *reinterpret_cast<const FloatType*>(v->dataPointer + v->stride * elementIndex);
				}
				else if(v->type == INT_PROPERTY) {
					v->value = *reinterpret_cast<const int*>(v->dataPointer + v->stride * elementIndex);
				}
				else if(v->type == INT64_PROPERTY) {
					v->value = *reinterpret_cast<const qlonglong*>(v->dataPointer + v->stride * elementIndex);
				}
				else if(v->type == ELEMENT_INDEX) {
					v->value = elementIndex;
				}
				else if(v->type == DERIVED_PROPERTY) {
					v->value = v->function(elementIndex);
				}
			}
		}

		// Evaluate expression for the current data element.
		return _parsers[component].Eval();
	}
	catch(const mu::Parser::exception_type& ex) {
		throw Exception(QString::fromStdString(ex.GetMsg()));
	}
}

/******************************************************************************
* Returns a human-readable text listing the input variables.
******************************************************************************/
QString PropertyExpressionEvaluator::inputVariableTable() const
{
	QString str(tr("<p>Available input variables:</p><p><b>Particle properties:</b><ul>"));
	for(const ExpressionVariable& v : _inputVariables) {
		if(v.type == FLOAT_PROPERTY || v.type == INT_PROPERTY || v.type == INT64_PROPERTY || v.type == ELEMENT_INDEX || v.type == DERIVED_PROPERTY) {
			if(v.description.isEmpty())
				str.append(QStringLiteral("<li>%1</li>").arg(QString::fromStdString(v.name)));
			else
				str.append(QStringLiteral("<li>%1 (<i style=\"color: #555;\">%2</i>)</li>").arg(QString::fromStdString(v.name)).arg(v.description));
		}
	}
	str.append(QStringLiteral("</ul></p><p><b>Global parameters:</b><ul>"));
	for(const ExpressionVariable& v : _inputVariables) {
		if(v.type == GLOBAL_PARAMETER) {
			if(v.description.isEmpty())
				str.append(QStringLiteral("<li>%1</li>").arg(QString::fromStdString(v.name)));
			else
				str.append(QStringLiteral("<li>%1 (<i style=\"color: #555;\">%2</i>)</li>").arg(QString::fromStdString(v.name)).arg(v.description));
		}
	}
	str.append(QStringLiteral("</ul></p><p><b>Constants:</b><ul>"));
	for(const ExpressionVariable& v : _inputVariables) {
		if(v.type == CONSTANT) {
			if(v.description.isEmpty())
				str.append(QStringLiteral("<li>%1</li>").arg(QString::fromStdString(v.name)));
			else
				str.append(QStringLiteral("<li>%1 (<i style=\"color: #555;\">%2</i>)</li>").arg(QString::fromStdString(v.name)).arg(v.description));
		}
	}
	str.append(QStringLiteral("</ul></p>"));
	return str;
}

}	// End of namespace
}	// End of namespace