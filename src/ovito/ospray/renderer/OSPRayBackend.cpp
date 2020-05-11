////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright 2017 Alexander Stukowski
//
//  This file is part of OVITO (Open Visualization Tool).
//
//  OVITO is free software; you can redistribute it and/or modify it either under the
//  terms of the GNU General Public License version 3 as published by the Free Software
//  Foundation (the "GPL") or, at your option, under the terms of the MIT License.
//  If you do not alter this notice, a recipient may use your version of this
//  file under either the GPL or the MIT License.
//
//  You should have received a copy of the GPL along with this program in a
//  file LICENSE.GPL.txt.  You should have received a copy of the MIT License along
//  with this program in a file LICENSE.MIT.txt
//
//  This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND,
//  either express or implied. See the GPL or the MIT License for the specific language
//  governing rights and limitations.
//
////////////////////////////////////////////////////////////////////////////////////////

#include <ovito/core/Core.h>
#include <ovito/core/utilities/units/UnitsManager.h>
#include "OSPRayBackend.h"

#include <ospray/ospray_cpp.h>

namespace Ovito { namespace OSPRay {

IMPLEMENT_OVITO_CLASS(OSPRayBackend);

IMPLEMENT_OVITO_CLASS(OSPRaySciVisBackend);
DEFINE_PROPERTY_FIELD(OSPRaySciVisBackend, shadowsEnabled);
DEFINE_PROPERTY_FIELD(OSPRaySciVisBackend, ambientOcclusionEnabled);
DEFINE_PROPERTY_FIELD(OSPRaySciVisBackend, ambientOcclusionSamples);
SET_PROPERTY_FIELD_LABEL(OSPRaySciVisBackend, shadowsEnabled, "Shadows");
SET_PROPERTY_FIELD_LABEL(OSPRaySciVisBackend, ambientOcclusionEnabled, "Ambient occlusion");
SET_PROPERTY_FIELD_LABEL(OSPRaySciVisBackend, ambientOcclusionSamples, "Ambient occlusion samples");
SET_PROPERTY_FIELD_UNITS_AND_RANGE(OSPRaySciVisBackend, ambientOcclusionSamples, IntegerParameterUnit, 1, 100);

IMPLEMENT_OVITO_CLASS(OSPRayPathTracerBackend);
DEFINE_PROPERTY_FIELD(OSPRayPathTracerBackend, rouletteDepth);
SET_PROPERTY_FIELD_LABEL(OSPRayPathTracerBackend, rouletteDepth, "Roulette depth");
SET_PROPERTY_FIELD_UNITS_AND_RANGE(OSPRayPathTracerBackend, rouletteDepth, IntegerParameterUnit, 1, 100);

/******************************************************************************
* Constructor.
******************************************************************************/
OSPRaySciVisBackend::OSPRaySciVisBackend(DataSet* dataset) : OSPRayBackend(dataset),
	_shadowsEnabled(true),
	_ambientOcclusionEnabled(true),
	_ambientOcclusionSamples(12)
{
}

/******************************************************************************
* Creates the OSPRay renderer object and configures it.
******************************************************************************/
OSPRenderer OSPRaySciVisBackend::createOSPRenderer(const Color& backgroundColor)
{
	auto renderer = ospNewRenderer("scivis");
	//renderer.set("shadowsEnabled", shadowsEnabled()); //TODO ?
	//renderer.set("aoSamples", ambientOcclusionEnabled() ? ambientOcclusionSamples() : 0);
	int aoSamples = (ambientOcclusionEnabled() ? ambientOcclusionSamples() : 0);
	ospSetInt(renderer, "aoSamples", aoSamples);
	//renderer.set"aoTransparencyEnabled", true); //TODO ?
	//renderer.set("bgColor", backgroundColor.r(), backgroundColor.g(), backgroundColor.b(), 0.0);
	//renderer.setParam("backgroundColor", ospray::cpp::vec4f{(float)backgroundColor.r(), (float)backgroundColor.g(), (float)backgroundColor.b(), 0.0});
	auto color = ospcommon::math::vec4f((float)backgroundColor.r(), (float)backgroundColor.g(), (float)backgroundColor.b(), 0.0);
    ospSetParam(renderer,"", OSP_VEC4F, color);
	return renderer;
}

/******************************************************************************
* Creates an OSPRay material.
******************************************************************************/
OSPMaterial OSPRaySciVisBackend::createOSPMaterial(const char* type)
{
	return ospNewMaterial("scivis", type);
}

/******************************************************************************
* Creates an OSPRay light.
******************************************************************************/
OSPLight OSPRaySciVisBackend::createOSPLight(const char* type)
{
	return ospNewLight(type);
}

/******************************************************************************
* Constructor.
******************************************************************************/
OSPRayPathTracerBackend::OSPRayPathTracerBackend(DataSet* dataset) : OSPRayBackend(dataset),
	_rouletteDepth(5)
{
}

/******************************************************************************
* Creates the OSPRay renderer object and configures it.
******************************************************************************/
OSPRenderer OSPRayPathTracerBackend::createOSPRenderer(const Color& backgroundColor)
{
	//ospray::cpp::Renderer renderer("pathtracer");
	auto renderer = ospNewRenderer("pathtracer");
	//renderer.set("rouletteDepth", rouletteDepth());
	ospSetInt(renderer, "roulettePathLength",rouletteDepth());
	return renderer;
}

/******************************************************************************
* Creates an OSPRay material.
******************************************************************************/
OSPMaterial OSPRayPathTracerBackend::createOSPMaterial(const char* type)
{
	return ospNewMaterial("pathtracer", type);
}

/******************************************************************************
* Creates an OSPRay light.
******************************************************************************/
OSPLight OSPRayPathTracerBackend::createOSPLight(const char* type)
{
	return ospNewLight(type);
}

}	// End of namespace
}	// End of namespace
