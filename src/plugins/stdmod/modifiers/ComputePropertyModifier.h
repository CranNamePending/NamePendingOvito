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


#include <plugins/stdmod/StdMod.h>
#include <plugins/stdobj/simcell/SimulationCell.h>
#include <plugins/stdobj/properties/PropertyContainer.h>
#include <plugins/stdobj/properties/PropertyReference.h>
#include <plugins/stdobj/properties/PropertyExpressionEvaluator.h>
#include <core/dataset/pipeline/AsynchronousDelegatingModifier.h>
#include <core/dataset/pipeline/AsynchronousModifierApplication.h>

namespace Ovito { namespace StdMod {

/**
 * \brief Base class for modifier delegates used by the ComputePropertyModifier class.
 */
class OVITO_STDMOD_EXPORT ComputePropertyModifierDelegate : public AsynchronousModifierDelegate
{
	Q_OBJECT
	OVITO_CLASS(ComputePropertyModifierDelegate)

protected:

	/// Constructor.
	using AsynchronousModifierDelegate::AsynchronousModifierDelegate;

	/// Asynchronous compute engine that does the actual work in a separate thread.
	class PropertyComputeEngine : public AsynchronousModifier::ComputeEngine
	{
	public:

		/// Constructor.
		PropertyComputeEngine(const TimeInterval& validityInterval, 
				TimePoint time, 
				const PipelineFlowState& input,
				const PropertyContainer* container,
				PropertyPtr outputProperty, 
				ConstPropertyPtr selectionProperty, 
				QStringList expressions, 
				int frameNumber,
				std::unique_ptr<PropertyExpressionEvaluator> evaluator);
				
		/// This method is called by the system after the computation was successfully completed.
		virtual void cleanup() override {
			_selection.reset();
			_expressions.clear();
			_evaluator.reset();
			ComputeEngine::cleanup();
		}

		/// Returns the property storage that contains the input element selection.
		const ConstPropertyPtr& selection() const { return _selection; }

		/// Returns the list of available input variables.
		virtual QStringList inputVariableNames() const;

		/// Returns the list of available input variables for the expressions managed by the delegate.
		virtual QStringList delegateInputVariableNames() const { return {}; }

		/// Returns a human-readable text listing the input variables.
		virtual QString inputVariableTable() const {
			if(_evaluator) return _evaluator->inputVariableTable();
			return {};
		}

		/// Injects the computed results into the data pipeline.
		virtual PipelineFlowState emitResults(TimePoint time, ModifierApplication* modApp, const PipelineFlowState& input) override;
		
		/// Returns the property storage that will receive the computed values.
		const PropertyPtr& outputProperty() const { return _outputProperty; }

		/// Determines whether any of the math expressions is explicitly time-dependent.
		virtual bool isTimeDependent() { return _evaluator->isTimeDependent(); }
	
	protected:

		const int _frameNumber;
		QStringList _expressions;
		ConstPropertyPtr _selection;
		std::unique_ptr<PropertyExpressionEvaluator> _evaluator;
		const PropertyPtr _outputProperty;
	};

public:

	/// \brief Returns the class of property containers this delegate operates on.
	virtual const PropertyContainerClass& containerClass() const = 0;

	/// \brief Returns a reference to the property container being operated on by this delegate.
	PropertyContainerReference subject() const {
		return PropertyContainerReference(&containerClass(), containerPath());
	}

	/// \brief Sets the number of vector components of the property to compute.
	/// \param componentCount The number of vector components.
	/// \undoable
	virtual void setComponentCount(int componentCount) {}

	/// Creates a computation engine that will compute the property values.
	virtual std::shared_ptr<PropertyComputeEngine> createEngine(
				TimePoint time, 
				const PipelineFlowState& input, 
				const PropertyContainer* container,
				PropertyPtr outputProperty, 
				ConstPropertyPtr selectionProperty,
				QStringList expressions) = 0;

private:

	/// Specifies the ID container object the modifier should operate on.
	DECLARE_MODIFIABLE_PROPERTY_FIELD(QString, containerPath, setContainerPath);
};

/**
 * \brief Computes the values of a property from a user-defined math expression.
 */
class OVITO_STDMOD_EXPORT ComputePropertyModifier : public AsynchronousDelegatingModifier
{
	/// Give this modifier class its own metaclass.
	class ComputePropertyModifierClass : public AsynchronousDelegatingModifier::OOMetaClass 
	{
	public:

		/// Inherit constructor from base class.
		using AsynchronousDelegatingModifier::OOMetaClass::OOMetaClass;

		/// Return the metaclass of delegates for this modifier type. 
		virtual const AsynchronousModifierDelegate::OOMetaClass& delegateMetaclass() const override { return ComputePropertyModifierDelegate::OOClass(); }
	};
	
	Q_OBJECT
	OVITO_CLASS_META(ComputePropertyModifier, ComputePropertyModifierClass)

	Q_CLASSINFO("DisplayName", "Compute property");
	Q_CLASSINFO("ModifierCategory", "Modification");
	
public:

	/// \brief Constructs a new instance of this class.
	Q_INVOKABLE ComputePropertyModifier(DataSet* dataset);

	/// \brief Returns the current delegate of this ComputePropertyModifier.
	ComputePropertyModifierDelegate* delegate() const { return static_object_cast<ComputePropertyModifierDelegate>(AsynchronousDelegatingModifier::delegate()); }

	/// \brief Sets the math expression that is used to calculate the values of one of the new property's components.
	/// \param index The property component for which the expression should be set.
	/// \param expression The math formula.
	/// \undoable
	void setExpression(const QString& expression, int index = 0) {
		if(index < 0 || index >= expressions().size())
			throwException("Property component index is out of range.");
		QStringList copy = _expressions;
		copy[index] = expression;
		setExpressions(std::move(copy));
	}

	/// \brief Returns the math expression that is used to calculate the values of one of the new property's components.
	/// \param index The property component for which the expression should be returned.
	/// \return The math formula used to calculates the channel's values.
	/// \undoable
	const QString& expression(int index = 0) const {
		if(index < 0 || index >= expressions().size())
			throwException("Property component index is out of range.");
		return expressions()[index];
	}

	/// \brief Returns the number of vector components of the property to create.
	/// \return The number of vector components.
	/// \sa setPropertyComponentCount()
	int propertyComponentCount() const { return expressions().size(); }

	/// \brief Sets the number of vector components of the property to create.
	/// \param newComponentCount The number of vector components.
	/// \undoable
	void setPropertyComponentCount(int newComponentCount);

	/// This method indicates whether cached computation results of the modifier should be discarded whenever
	/// a parameter of the modifier changes.
	virtual bool discardResultsOnModifierChange(const PropertyFieldEvent& event) const override { 
		if(event.field() == &PROPERTY_FIELD(useMultilineFields)) return false;
		return AsynchronousDelegatingModifier::discardResultsOnModifierChange(event);
	}

protected:

	/// \brief Is called when the value of a reference field of this RefMaker changes.
	virtual void referenceReplaced(const PropertyFieldDescriptor& field, RefTarget* oldTarget, RefTarget* newTarget) override;

	/// Creates a computation engine that will compute the modifier's results.
	virtual Future<ComputeEnginePtr> createEngine(TimePoint time, ModifierApplication* modApp, const PipelineFlowState& input) override;

protected:

	/// The math expressions for calculating the property values. One for every vector component.
	DECLARE_MODIFIABLE_PROPERTY_FIELD(QStringList, expressions, setExpressions);

	/// Specifies the output property that will receive the computed per-particles values.
	DECLARE_MODIFIABLE_PROPERTY_FIELD(PropertyReference, outputProperty, setOutputProperty);

	/// Controls whether the math expression is evaluated and output only for selected elements.
	DECLARE_MODIFIABLE_PROPERTY_FIELD(bool, onlySelectedElements, setOnlySelectedElements);

	/// Controls whether multi-line input fields are shown in the UI for the expressions.
	DECLARE_MODIFIABLE_PROPERTY_FIELD(bool, useMultilineFields, setUseMultilineFields);
};

/**
 * Used by the ComputePropertyModifier to store working data.
 */
class OVITO_STDMOD_EXPORT ComputePropertyModifierApplication : public AsynchronousModifierApplication
{
	OVITO_CLASS(ComputePropertyModifierApplication)
	Q_OBJECT

public:

	/// Constructor.
	Q_INVOKABLE ComputePropertyModifierApplication(DataSet* dataset) : AsynchronousModifierApplication(dataset) {}

private:

	/// The cached visualization elements that are attached to the output property.
	DECLARE_MODIFIABLE_VECTOR_REFERENCE_FIELD_FLAGS(DataVis, cachedVisElements, setCachedVisElements, PROPERTY_FIELD_NEVER_CLONE_TARGET | PROPERTY_FIELD_NO_CHANGE_MESSAGE | PROPERTY_FIELD_NO_UNDO | PROPERTY_FIELD_NO_SUB_ANIM);

	/// The list of input variables during the last evaluation.
	DECLARE_RUNTIME_PROPERTY_FIELD_FLAGS(QStringList, inputVariableNames, setInputVariableNames, PROPERTY_FIELD_NO_CHANGE_MESSAGE);

	/// The list of input variables for the expressions managed by the delegate during the last evaluation.
	DECLARE_RUNTIME_PROPERTY_FIELD_FLAGS(QStringList, delegateInputVariableNames, setDelegateInputVariableNames, PROPERTY_FIELD_NO_CHANGE_MESSAGE);

	/// Human-readable text listing the input variables during the last evaluation.
	DECLARE_RUNTIME_PROPERTY_FIELD_FLAGS(QString, inputVariableTable, setInputVariableTable, PROPERTY_FIELD_NO_CHANGE_MESSAGE);
};

}	// End of namespace
}	// End of namespace