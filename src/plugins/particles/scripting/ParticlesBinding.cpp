///////////////////////////////////////////////////////////////////////////////
//
//  Copyright (2013) Alexander Stukowski, Tobias Brink
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

#include <plugins/particles/Particles.h>
#include <plugins/particles/modifier/ParticleModifier.h>
#include <plugins/particles/modifier/coloring/ColorCodingModifier.h>
#include <plugins/particles/importer/ParticleImporter.h>
#include <plugins/particles/importer/InputColumnMapping.h>
#include <plugins/particles/exporter/OutputColumnMapping.h>
#include <plugins/particles/exporter/ParticleExporter.h>
#include <plugins/scripting/engine/ScriptEngine.h>
#include "ParticlesBinding.h"

namespace Particles {

IMPLEMENT_OVITO_OBJECT(Particles, ParticlesBinding, ScriptBinding);

/******************************************************************************
* Sets up the global object of the script engine.
******************************************************************************/
void ParticlesBinding::setupBinding(ScriptEngine& engine)
{
	// Register marshalling functions for ParticlePropertyReference.
	qScriptRegisterMetaType<ParticlePropertyReference>(&engine, fromParticlePropertyReference, toParticlePropertyReference);

	// Register marshalling functions for InputColumnMapping.
	qScriptRegisterMetaType<InputColumnMapping>(&engine, fromInputColumnMapping, toInputColumnMapping);

	// Register marshalling functions for OutputColumnMapping.
	qScriptRegisterMetaType<OutputColumnMapping>(&engine, fromOutputColumnMapping, toOutputColumnMapping);

	// Register important plugin classes.
	engine.registerOvitoObjectType<ParticleImporter>();
	engine.registerOvitoObjectType<ParticleExporter>();
	engine.registerOvitoObjectType<ParticleModifier>();
	engine.registerOvitoObjectType<ColorCodingModifier>();
	engine.registerOvitoObjectType<ColorCodingGradient>();
}

/******************************************************************************
* Creates a QScriptValue from a ParticlePropertyReference.
******************************************************************************/
QScriptValue ParticlesBinding::fromParticlePropertyReference(QScriptEngine* engine, const ParticlePropertyReference& pref)
{
	if(pref.isNull())
		return QScriptValue(QScriptValue::NullValue);
	else if(pref.type() == ParticleProperty::UserProperty) {
		int component = pref.vectorComponent();
		if(component < 0)
			return pref.name();
		else
			return pref.name() + QStringLiteral(".") + QString::number(component);
	}
	else {
		QString name = ParticleProperty::standardPropertyName(pref.type());
		if(pref.vectorComponent() < 0)
			return name;
		else {
			QStringList components = ParticleProperty::standardPropertyComponentNames(pref.type());
			if(pref.vectorComponent() >= components.length())
				return name + QStringLiteral(".") + QString::number(pref.vectorComponent());
			else
				return name + QStringLiteral(".") + components[pref.vectorComponent()];
		}
	}
}

/******************************************************************************
* Converts a QScriptValue to a ParticlePropertyReference.
******************************************************************************/
void ParticlesBinding::toParticlePropertyReference(const QScriptValue& obj, ParticlePropertyReference& pref)
{
	QScriptContext* context = obj.engine()->currentContext();
	if(obj.isNull()) {
		pref = ParticlePropertyReference();
		return;
	}
	QStringList parts = obj.toString().split(QChar('.'));
	if(parts.length() > 2) {
		context->throwError("Too many dots in particle property name string.");
		return;
	}
	else if(parts.length() == 0 || parts[0].isEmpty()) {
		context->throwError("Particle property name string is empty.");
		return;
	}
	// Determine property type.
	QString name = parts[0];
	ParticleProperty::Type type = ParticleProperty::standardPropertyList().value(name, ParticleProperty::UserProperty);

	// Determine vector component.
	int component = -1;
	if(parts.length() == 2) {
		// First try to convert component to integer.
		bool ok;
		component = parts[1].toInt(&ok);
		if(!ok) {
			if(type == ParticleProperty::UserProperty) {
				context->throwError(tr("Invalid component name or index for particle property '%1': %2").arg(parts[0]).arg(parts[1]));
				return;
			}

			// Perhaps the name was used instead of an integer.
			const QString componentName = parts[1].toUpper();
			QStringList standardNames = ParticleProperty::standardPropertyComponentNames(type);
			component = standardNames.indexOf(componentName);
			if(component < 0) {
				context->throwError(tr("Unknown component name '%1' for particle property '%2'. Possible components are: %3").arg(parts[1]).arg(parts[0]).arg(standardNames.join(',')));
				return;
			}
		}
	}

	// Construct object.
	if(type == Particles::ParticleProperty::UserProperty)
		pref = ParticlePropertyReference(name, component);
	else
		pref = ParticlePropertyReference(type, component);
}

/******************************************************************************
* Creates a QScriptValue from a InputColumnMapping.
******************************************************************************/
QScriptValue ParticlesBinding::fromInputColumnMapping(QScriptEngine* engine, const InputColumnMapping& mapping)
{
	QScriptValue result = engine->newArray(mapping.size());
	for(int i = 0; i < mapping.size(); i++) {
		result.setProperty(i, engine->toScriptValue(mapping[i].property));
	}
	return result;
}

/******************************************************************************
* Converts a QScriptValue to a InputColumnMapping.
******************************************************************************/
void ParticlesBinding::toInputColumnMapping(const QScriptValue& obj, InputColumnMapping& mapping)
{
	QScriptContext* context = obj.engine()->currentContext();
	if(obj.isArray() == false) {
		context->throwError("Column mapping must be specified as an array of strings.");
		return;
	}

	int columnCount = obj.property("length").toInt32();
	mapping.resize(columnCount);
	for(int i = 0; i < columnCount; i++) {
		ParticlePropertyReference pref;
		toParticlePropertyReference(obj.property(i), pref);
		if(!pref.isNull()) {
			if(pref.type() != ParticleProperty::UserProperty)
				mapping[i].mapStandardColumn(pref.type(), pref.vectorComponent());
			else
				mapping[i].mapCustomColumn(pref.name(), qMetaTypeId<FloatType>(), pref.vectorComponent());
		}
	}
}

/******************************************************************************
* Creates a QScriptValue from a OutputColumnMapping.
******************************************************************************/
QScriptValue ParticlesBinding::fromOutputColumnMapping(QScriptEngine* engine, const OutputColumnMapping& mapping)
{
	QScriptValue result = engine->newArray(mapping.size());
	for(int i = 0; i < mapping.size(); i++) {
		ParticlePropertyReference pref(mapping[i].type(), mapping[i].name(), mapping[i].vectorComponent());
		result.setProperty(i, engine->toScriptValue(pref));
	}
	return result;
}

/******************************************************************************
* Converts a QScriptValue to a OutputColumnMapping.
******************************************************************************/
void ParticlesBinding::toOutputColumnMapping(const QScriptValue& obj, OutputColumnMapping& mapping)
{
	QScriptContext* context = obj.engine()->currentContext();
	if(obj.isArray() == false) {
		context->throwError("Column mapping must be specified as an array of strings.");
		return;
	}

	int columnCount = obj.property("length").toInt32();
	for(int i = 0; i < columnCount; i++) {
		ParticlePropertyReference pref;
		toParticlePropertyReference(obj.property(i), pref);
		mapping.push_back(pref);
	}
}

};
