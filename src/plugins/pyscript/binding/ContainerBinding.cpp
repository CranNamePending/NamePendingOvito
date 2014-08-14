///////////////////////////////////////////////////////////////////////////////
//
//  Copyright (2014) Alexander Stukowski
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

#include <plugins/pyscript/PyScript.h>
#include <core/scene/pipeline/ModifierApplication.h>
#include <core/scene/objects/SceneObject.h>
#include <core/scene/SceneNode.h>
#include <core/scene/ObjectNode.h>
#include <core/scene/display/DisplayObject.h>
#include "PythonBinding.h"

namespace PyScript {

using namespace boost::python;
using namespace Ovito;

void setupContainerBinding()
{
	class_<QVector<DisplayObject*>, boost::noncopyable>("QVectorDisplayObject", no_init)
		.def(QVector_OO_readonly_indexing_suite<DisplayObject>())
	;
	python_to_container_conversion<QVector<DisplayObject*>>();

	class_<QVector<SceneNode*>, boost::noncopyable>("QVectorSceneNode", no_init)
		.def(QVector_OO_readonly_indexing_suite<SceneNode>())
	;
	python_to_container_conversion<QVector<SceneNode*>>();

	class_<QVector<SceneObject*>, boost::noncopyable>("QVectorSceneObject", no_init)
		.def(QVector_OO_readonly_indexing_suite<SceneObject>())
	;
	python_to_container_conversion<QVector<SceneObject*>>();

	class_<QVector<OORef<SceneObject>>, boost::noncopyable>("QVectorOORefSceneObject", no_init)
		.def(QVector_OO_readonly_indexing_suite<SceneObject, QVector<OORef<SceneObject>>>())
	;
	class_<QVector<ModifierApplication*>, boost::noncopyable>("QVectorModifierApplication", no_init)
		.def(QVector_OO_readonly_indexing_suite<ModifierApplication>())
	;
	class_<QVector<int>>("QVectorInt")
		.def(array_indexing_suite<QVector<int>>())
	;
	class_<QList<int>>("QListInt")
		.def(array_indexing_suite<QList<int>>())
	;
}

};
