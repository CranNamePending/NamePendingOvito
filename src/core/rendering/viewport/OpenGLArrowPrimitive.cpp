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
#include "OpenGLArrowPrimitive.h"
#include "ViewportSceneRenderer.h"

namespace Ovito {

/******************************************************************************
* Constructor.
******************************************************************************/
OpenGLArrowPrimitive::OpenGLArrowPrimitive(ViewportSceneRenderer* renderer, ArrowPrimitive::Shape shape, ShadingMode shadingMode, RenderingQuality renderingQuality) :
	ArrowPrimitive(shape, shadingMode, renderingQuality),
	_contextGroup(QOpenGLContextGroup::currentContextGroup()),
	_elementCount(-1), _cylinderSegments(16), _verticesPerElement(0),
	_mappedBuffer(nullptr)
{
	OVITO_ASSERT(renderer->glcontext()->shareGroup() == _contextGroup);

	if(!_glGeometryBuffer.create())
		throw Exception(QStringLiteral("Failed to create OpenGL vertex buffer."));
	_glGeometryBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);

	// Initialize OpenGL shaders.

	_shadedShader = renderer->loadShaderProgram(
			"arrow_shaded",
			":/core/glsl/arrows/shaded.vs",
			":/core/glsl/arrows/shaded.fs");

	_shadedPickingShader = renderer->loadShaderProgram(
			"arrow_shaded_picking",
			":/core/glsl/arrows/picking/shaded.vs",
			":/core/glsl/arrows/picking/shaded.fs");

	_flatShader = renderer->loadShaderProgram(
			"arrow_flat",
			":/core/glsl/arrows/flat.vs",
			":/core/glsl/arrows/flat.fs");

	_flatPickingShader = renderer->loadShaderProgram(
			"arrow_flat_picking",
			":/core/glsl/arrows/picking/flat.vs",
			":/core/glsl/arrows/picking/flat.fs");

	_raytracedCylinderShader = renderer->loadShaderProgram(
			"cylinder_raytraced",
			":/core/glsl/cylinder/cylinder_raytraced.vs",
			":/core/glsl/cylinder/cylinder_raytraced.fs");

	_raytracedCylinderPickingShader = renderer->loadShaderProgram(
			"cylinder_raytraced_picking",
			":/core/glsl/cylinder/picking/cylinder_raytraced.vs",
			":/core/glsl/cylinder/picking/cylinder_raytraced.fs");
}

/******************************************************************************
* Allocates a particle buffer with the given number of elements.
******************************************************************************/
void OpenGLArrowPrimitive::startSetElements(int elementCount)
{
	OVITO_ASSERT(_glGeometryBuffer.isCreated());
	OVITO_ASSERT(elementCount >= 0);
	OVITO_ASSERT(QOpenGLContextGroup::currentContextGroup() == _contextGroup);
	OVITO_ASSERT(_mappedBuffer == nullptr);

	if(!_glGeometryBuffer.bind())
		throw Exception(QStringLiteral("Failed to bind OpenGL vertex buffer."));

	_elementCount = elementCount;
	size_t bytesPerVertex = 0;
	if(shadingMode() == NormalShading) {
		int cylinderVertexCount = _cylinderSegments * 2 + 2;
		int discVertexCount = _cylinderSegments;
		int cylinderCount, discCount;
		bytesPerVertex = sizeof(ColoredVertexWithNormal);
		if(shape() == ArrowShape) {
			cylinderCount = 2;
			discCount = 2;
		}
		else {
			cylinderCount = 1;
			discCount = 2;
			if(renderingQuality() == HighQuality) {
				cylinderVertexCount = 14;
				discCount = discVertexCount = 0;
				bytesPerVertex = sizeof(ColoredVertexWithElementInfo);
			}
		}
		_verticesPerElement = cylinderCount * cylinderVertexCount + discCount * discVertexCount;
		_elementRenderCount = std::min(_elementCount, std::numeric_limits<int>::max() / (int)bytesPerVertex / _verticesPerElement);

		// Prepare arrays to be passed to the glMultiDrawArrays() function.
		_stripPrimitiveVertexStarts.resize(_elementRenderCount * cylinderCount);
		_stripPrimitiveVertexCounts.resize(_elementRenderCount * cylinderCount);
		_fanPrimitiveVertexStarts.resize(_elementRenderCount * discCount);
		_fanPrimitiveVertexCounts.resize(_elementRenderCount * discCount);
		std::fill(_stripPrimitiveVertexCounts.begin(), _stripPrimitiveVertexCounts.end(), cylinderVertexCount);
		std::fill(_fanPrimitiveVertexCounts.begin(), _fanPrimitiveVertexCounts.end(), discVertexCount);
		auto ps_strip = _stripPrimitiveVertexStarts.begin();
		auto ps_fan = _fanPrimitiveVertexStarts.begin();
		GLint baseIndex = 0;
		for(GLint index = 0; index < _elementRenderCount; index++) {
			for(int p = 0; p < cylinderCount; p++, baseIndex += cylinderVertexCount)
				*ps_strip++ = baseIndex;
			for(int p = 0; p < discCount; p++, baseIndex += discVertexCount)
				*ps_fan++ = baseIndex;
		}
	}
	else if(shadingMode() == FlatShading) {
		if(shape() == ArrowShape)
			_verticesPerElement = 7;
		else
			_verticesPerElement = 4;
		bytesPerVertex = sizeof(ColoredVertexWithVector);
		_elementRenderCount = std::min(_elementCount, std::numeric_limits<int>::max() / (int)bytesPerVertex / _verticesPerElement);

		// Prepare arrays to be passed to the glMultiDrawArrays() function.
		_fanPrimitiveVertexStarts.resize(_elementRenderCount);
		GLint startIndex = 0;
		for(auto& i : _fanPrimitiveVertexStarts) {
			i = startIndex;
			startIndex += _verticesPerElement;
		}
		_fanPrimitiveVertexCounts.resize(_elementRenderCount);
		std::fill(_fanPrimitiveVertexCounts.begin(), _fanPrimitiveVertexCounts.end(), _verticesPerElement);
		_stripPrimitiveVertexStarts.clear();
		_stripPrimitiveVertexCounts.clear();
	}
	else OVITO_ASSERT(false);

	// Allocate vertex buffer memory.
	_glGeometryBuffer.allocate(_elementRenderCount * _verticesPerElement * bytesPerVertex);
	if(_elementRenderCount) {
		_mappedBuffer = _glGeometryBuffer.map(QOpenGLBuffer::WriteOnly);
		OVITO_CHECK_POINTER(_mappedBuffer);
		if(!_mappedBuffer)
			throw Exception(QStringLiteral("Failed to map OpenGL vertex buffer to memory."));
	}

	// Precompute cos() and sin() functions.
	if(shadingMode() == NormalShading) {
		_cosTable.resize(_cylinderSegments+1);
		_sinTable.resize(_cylinderSegments+1);
		for(int i = 0; i <= _cylinderSegments; i++) {
			float angle = (FLOATTYPE_PI * 2 / _cylinderSegments) * i;
			_cosTable[i] = std::cos(angle);
			_sinTable[i] = std::sin(angle);
		}
	}
	OVITO_CHECK_OPENGL();
}

/******************************************************************************
* Sets the properties of a single element.
******************************************************************************/
void OpenGLArrowPrimitive::setElement(int index, const Point3& pos, const Vector3& dir, const ColorA& color, FloatType width)
{
	OVITO_ASSERT(index >= 0 && index < _elementCount);
	OVITO_ASSERT(_mappedBuffer != nullptr);

	if(index >= _elementRenderCount)
		return;		// Skip elements if there are too many.

	if(shape() == ArrowShape)
		createArrowElement(index, pos, dir, color, width);
	else
		createCylinderElement(index, pos, dir, color, width);
}

/******************************************************************************
* Creates the geometry for a single cylinder element.
******************************************************************************/
void OpenGLArrowPrimitive::createCylinderElement(int index, const Point3& pos, const Vector3& dir, const ColorA& color, FloatType width)
{
	if(shadingMode() == NormalShading) {

		// Build local coordinate system.
		Vector_3<float> t, u, v;
		float length = dir.length();
		if(length != 0) {
			t = dir / length;
			if(dir.y() != 0 || dir.x() != 0)
				u = Vector_3<float>(dir.y(), -dir.x(), 0);
			else
				u = Vector_3<float>(-dir.z(), 0, dir.x());
			u.normalize();
			v = u.cross(t);
		}
		else {
			t.setZero();
			u.setZero();
			v.setZero();
		}

		ColorAT<float> c = color;
		Point_3<float> v1 = pos;
		Point_3<float> v2 = v1 + dir;

		if(renderingQuality() != HighQuality) {
			ColoredVertexWithNormal* vertex = static_cast<ColoredVertexWithNormal*>(_mappedBuffer) + (index * _verticesPerElement);

			// Generate vertices for cylinder mantle.
			for(int i = 0; i <= _cylinderSegments; i++) {
				Vector_3<float> n = _cosTable[i] * u + _sinTable[i] * v;
				Vector_3<float> d = n * width;
				vertex->pos = v1 + d;
				vertex->normal = n;
				vertex->color = c;
				vertex++;
				vertex->pos = v2 + d;
				vertex->normal = n;
				vertex->color = c;
				vertex++;
			}

			// Generate vertices for first cylinder cap.
			for(int i = 0; i < _cylinderSegments; i++) {
				Vector_3<float> n = _cosTable[i] * u + _sinTable[i] * v;
				Vector_3<float> d = n * width;
				vertex->pos = v1 + d;
				vertex->normal = Vector_3<float>(0,0,-1);
				vertex->color = c;
				vertex++;
			}

			// Generate vertices for second cylinder cap.
			for(int i = _cylinderSegments - 1; i >= 0; i--) {
				Vector_3<float> n = _cosTable[i] * u + _sinTable[i] * v;
				Vector_3<float> d = n * width;
				vertex->pos = v2 + d;
				vertex->normal = Vector_3<float>(0,0,1);
				vertex->color = c;
				vertex++;
			}
		}
		else {
			// Create bounding box geometry around cylinder for raytracing.
			ColoredVertexWithElementInfo* vertex = static_cast<ColoredVertexWithElementInfo*>(_mappedBuffer) + (index * _verticesPerElement);
			u *= width;
			v *= width;
			Point3 corners[8] = {
					v1 - u - v,
					v1 - u + v,
					v1 + u - v,
					v1 + u + v,
					v2 - u - v,
					v2 - u + v,
					v2 + u + v,
					v2 + u - v
			};
			const static size_t stripIndices[14] = { 3,2,6,7,4,2,0,3,1,6,5,4,1,0 };
			for(int i = 0; i < 14; i++, vertex++) {
				vertex->pos = corners[stripIndices[i]];
				vertex->base = v1;
				vertex->dir = dir;
				vertex->color = c;
				vertex->radius = width;
			}
		}
	}
	else if(shadingMode() == FlatShading) {

		Vector_3<float> t;
		float length = dir.length();
		if(length != 0)
			t = dir / length;
		else
			t.setZero();

		ColorAT<float> c = color;
		Point_3<float> base = pos;

		ColoredVertexWithVector* vertices = static_cast<ColoredVertexWithVector*>(_mappedBuffer) + (index * _verticesPerElement);
		vertices[0].pos = Point_3<float>(0, width, 0);
		vertices[1].pos = Point_3<float>(0, -width, 0);
		vertices[2].pos = Point_3<float>(length, -width, 0);
		vertices[3].pos = Point_3<float>(length, width, 0);
		for(int i = 0; i < _verticesPerElement; i++, ++vertices) {
			vertices->base = base;
			vertices->dir = t;
			vertices->color = c;
		}
	}
}

/******************************************************************************
* Creates the geometry for a single arrow element.
******************************************************************************/
void OpenGLArrowPrimitive::createArrowElement(int index, const Point3& pos, const Vector3& dir, const ColorA& color, FloatType width)
{
	const float arrowHeadRadius = width * 2.5f;
	const float arrowHeadLength = arrowHeadRadius * 1.8f;

	if(shadingMode() == NormalShading) {

		// Build local coordinate system.
		Vector_3<float> t, u, v;
		float length = dir.length();
		if(length != 0) {
			t = dir / length;
			if(dir.y() != 0 || dir.x() != 0)
				u = Vector_3<float>(dir.y(), -dir.x(), 0);
			else
				u = Vector_3<float>(-dir.z(), 0, dir.x());
			u.normalize();
			v = u.cross(t);
		}
		else {
			t.setZero();
			u.setZero();
			v.setZero();
		}

		ColorAT<float> c = color;
		Point_3<float> v1 = pos;
		Point_3<float> v2;
		Point_3<float> v3 = v1 + dir;
		float r;
		if(length > arrowHeadLength) {
			v2 = v1 + t * (length - arrowHeadLength);
			r = arrowHeadRadius;
		}
		else {
			v2 = v1;
			r = arrowHeadRadius * length / arrowHeadLength;
		}

		ColoredVertexWithNormal* vertex = static_cast<ColoredVertexWithNormal*>(_mappedBuffer) + (index * _verticesPerElement);

		// Generate vertices for cylinder.
		for(int i = 0; i <= _cylinderSegments; i++) {
			Vector_3<float> n = _cosTable[i] * u + _sinTable[i] * v;
			Vector_3<float> d = n * width;
			vertex->pos = v1 + d;
			vertex->normal = n;
			vertex->color = c;
			vertex++;
			vertex->pos = v2 + d;
			vertex->normal = n;
			vertex->color = c;
			vertex++;
		}

		// Generate vertices for head cone.
		for(int i = 0; i <= _cylinderSegments; i++) {
			Vector_3<float> n = _cosTable[i] * u + _sinTable[i] * v;
			Vector_3<float> d = n * r;
			vertex->pos = v2 + d;
			vertex->normal = n;
			vertex->color = c;
			vertex++;
			vertex->pos = v3;
			vertex->normal = n;
			vertex->color = c;
			vertex++;
		}

		// Generate vertices for cylinder cap.
		for(int i = 0; i < _cylinderSegments; i++) {
			Vector_3<float> n = _cosTable[i] * u + _sinTable[i] * v;
			Vector_3<float> d = n * width;
			vertex->pos = v1 + d;
			vertex->normal = Vector_3<float>(0,0,-1);
			vertex->color = c;
			vertex++;
		}

		// Generate vertices for cone cap.
		for(int i = 0; i < _cylinderSegments; i++) {
			Vector_3<float> n = _cosTable[i] * u + _sinTable[i] * v;
			Vector_3<float> d = n * r;
			vertex->pos = v2 + d;
			vertex->normal = Vector_3<float>(0,0,-1);
			vertex->color = c;
			vertex++;
		}
	}
	else if(shadingMode() == FlatShading) {

		Vector_3<float> t;
		float length = dir.length();
		if(length != 0)
			t = dir / length;
		else
			t.setZero();

		ColorAT<float> c = color;
		Point_3<float> base = pos;

		ColoredVertexWithVector* vertices = static_cast<ColoredVertexWithVector*>(_mappedBuffer) + (index * _verticesPerElement);
		if(length > arrowHeadLength) {
			vertices[0].pos = Point_3<float>(length, 0, 0);
			vertices[1].pos = Point_3<float>(length - arrowHeadLength, arrowHeadRadius, 0);
			vertices[2].pos = Point_3<float>(length - arrowHeadLength, width, 0);
			vertices[3].pos = Point_3<float>(0, width, 0);
			vertices[4].pos = Point_3<float>(0, -width, 0);
			vertices[5].pos = Point_3<float>(length - arrowHeadLength, -width, 0);
			vertices[6].pos = Point_3<float>(length - arrowHeadLength, -arrowHeadRadius, 0);
		}
		else {
			float r = arrowHeadRadius * length / arrowHeadLength;
			vertices[0].pos = Point_3<float>(length, 0, 0);
			vertices[1].pos = Point_3<float>(0, r, 0);
			vertices[2].pos = Point_3<float>::Origin();
			vertices[3].pos = Point_3<float>::Origin();
			vertices[4].pos = Point_3<float>::Origin();
			vertices[5].pos = Point_3<float>::Origin();
			vertices[6].pos = Point_3<float>(0, -r, 0);
		}
		for(int i = 0; i < _verticesPerElement; i++, ++vertices) {
			vertices->base = base;
			vertices->dir = t;
			vertices->color = c;
		}
	}
}


/******************************************************************************
* Finalizes the geometry buffer after all elements have been set.
******************************************************************************/
void OpenGLArrowPrimitive::endSetElements()
{
	OVITO_ASSERT(QOpenGLContextGroup::currentContextGroup() == _contextGroup);
	OVITO_ASSERT(_elementCount >= 0 && _elementRenderCount >= 0);
	OVITO_ASSERT(_mappedBuffer != nullptr || _elementRenderCount == 0);

	if(_elementRenderCount)
		_glGeometryBuffer.unmap();
	_glGeometryBuffer.release();
	_mappedBuffer = nullptr;
	OVITO_CHECK_OPENGL();
}

/******************************************************************************
* Returns true if the geometry buffer is filled and can be rendered with the given renderer.
******************************************************************************/
bool OpenGLArrowPrimitive::isValid(SceneRenderer* renderer)
{
	ViewportSceneRenderer* vpRenderer = dynamic_object_cast<ViewportSceneRenderer>(renderer);
	if(!vpRenderer) return false;
	return _glGeometryBuffer.isCreated()
			&& _elementCount >= 0
			&& (_contextGroup == vpRenderer->glcontext()->shareGroup());
}

/******************************************************************************
* Renders the geometry.
******************************************************************************/
void OpenGLArrowPrimitive::render(SceneRenderer* renderer)
{
	OVITO_CHECK_OPENGL();
	OVITO_ASSERT(_glGeometryBuffer.isCreated());
	OVITO_ASSERT(_contextGroup == QOpenGLContextGroup::currentContextGroup());
	OVITO_ASSERT(_elementCount >= 0);
	OVITO_ASSERT(_mappedBuffer == nullptr);

	ViewportSceneRenderer* vpRenderer = dynamic_object_cast<ViewportSceneRenderer>(renderer);

	if(_elementCount <= 0 || !vpRenderer)
		return;

	if(shadingMode() == NormalShading) {
		if(renderingQuality() == HighQuality && shape() == CylinderShape)
			renderRaytracedCylinders(vpRenderer);
		else
			renderShadedTriangles(vpRenderer);
	}
	else if(shadingMode() == FlatShading) {
		renderFlat(vpRenderer);
	}
	OVITO_CHECK_OPENGL();
}

/******************************************************************************
* Renders the arrows in shaded mode.
******************************************************************************/
void OpenGLArrowPrimitive::renderShadedTriangles(ViewportSceneRenderer* renderer)
{
	QOpenGLShaderProgram* shader;
	if(!renderer->isPicking())
		shader = _shadedShader;
	else
		shader = _shadedPickingShader;

	glEnable(GL_CULL_FACE);

	if(!shader->bind())
		throw Exception(QStringLiteral("Failed to bind OpenGL shader."));

	shader->setUniformValue("modelview_projection_matrix",
			(QMatrix4x4)(renderer->projParams().projectionMatrix * renderer->modelViewTM()));
	if(!renderer->isPicking())
		shader->setUniformValue("normal_matrix", (QMatrix3x3)(renderer->modelViewTM().linear().inverse().transposed()));
	else
		shader->setUniformValue("pickingBaseID", (GLint)renderer->registerSubObjectIDs(elementCount()));

	_glGeometryBuffer.bind();
	if(renderer->glformat().majorVersion() < 3) {
		OVITO_CHECK_OPENGL(glEnableClientState(GL_VERTEX_ARRAY));
		OVITO_CHECK_OPENGL(glVertexPointer(3, GL_FLOAT, sizeof(ColoredVertexWithVector), reinterpret_cast<const GLvoid*>(offsetof(ColoredVertexWithVector, pos))));
	}
	shader->enableAttributeArray("vertex_pos");
	shader->setAttributeBuffer("vertex_pos", GL_FLOAT, offsetof(ColoredVertexWithNormal, pos), 3, sizeof(ColoredVertexWithNormal));
	if(!renderer->isPicking()) {
		shader->enableAttributeArray("vertex_normal");
		shader->setAttributeBuffer("vertex_normal", GL_FLOAT, offsetof(ColoredVertexWithNormal, normal), 3, sizeof(ColoredVertexWithNormal));
		shader->enableAttributeArray("vertex_color");
		shader->setAttributeBuffer("vertex_color", GL_FLOAT, offsetof(ColoredVertexWithNormal, color), 4, sizeof(ColoredVertexWithNormal));
	}
	_glGeometryBuffer.release();

	if(renderer->isPicking())
		renderer->activateVertexIDs(shader, _elementCount * _verticesPerElement, true);

	if(renderer->isPicking()) {
		int primitivesPerElement = _stripPrimitiveVertexCounts.size() / _elementCount;
		int stripVerticesPerElement = std::accumulate(_stripPrimitiveVertexCounts.begin(), _stripPrimitiveVertexCounts.begin() + primitivesPerElement, 0);
		OVITO_CHECK_OPENGL(shader->setUniformValue("verticesPerElement", (GLint)stripVerticesPerElement));
	}
	OVITO_CHECK_OPENGL(renderer->glMultiDrawArrays(GL_TRIANGLE_STRIP, _stripPrimitiveVertexStarts.data(), _stripPrimitiveVertexCounts.data(), _stripPrimitiveVertexStarts.size()));

	if(renderer->isPicking()) {
		int primitivesPerElement = _fanPrimitiveVertexCounts.size() / _elementCount;
		int fanVerticesPerElement = std::accumulate(_fanPrimitiveVertexCounts.begin(), _fanPrimitiveVertexCounts.begin() + primitivesPerElement, 0);
		OVITO_CHECK_OPENGL(shader->setUniformValue("verticesPerElement", (GLint)fanVerticesPerElement));
	}
	OVITO_CHECK_OPENGL(renderer->glMultiDrawArrays(GL_TRIANGLE_FAN, _fanPrimitiveVertexStarts.data(), _fanPrimitiveVertexCounts.data(), _fanPrimitiveVertexStarts.size()));

	shader->disableAttributeArray("vertex_pos");
	if(!renderer->isPicking()) {
		shader->disableAttributeArray("vertex_normal");
		shader->disableAttributeArray("vertex_color");
	}
	else {
		renderer->deactivateVertexIDs(shader, true);
	}
	if(renderer->glformat().majorVersion() < 3)
		OVITO_CHECK_OPENGL(glDisableClientState(GL_VERTEX_ARRAY));

	shader->release();
}

/******************************************************************************
* Renders the cylinder elements in using a raytracing hardware shader.
******************************************************************************/
void OpenGLArrowPrimitive::renderRaytracedCylinders(ViewportSceneRenderer* renderer)
{
	QOpenGLShaderProgram* shader;
	if(!renderer->isPicking())
		shader = _raytracedCylinderShader;
	else
		shader = _raytracedCylinderPickingShader;

	glEnable(GL_CULL_FACE);

	if(!shader->bind())
		throw Exception(QStringLiteral("Failed to bind OpenGL shader."));

	shader->setUniformValue("modelview_matrix",
			(QMatrix4x4)renderer->modelViewTM());
	shader->setUniformValue("modelview_uniform_scale", (float)pow(std::abs(renderer->modelViewTM().determinant()), (FloatType(1.0/3.0))));
	shader->setUniformValue("modelview_projection_matrix",
			(QMatrix4x4)(renderer->projParams().projectionMatrix * renderer->modelViewTM()));
	shader->setUniformValue("projection_matrix", (QMatrix4x4)renderer->projParams().projectionMatrix);
	shader->setUniformValue("inverse_projection_matrix", (QMatrix4x4)renderer->projParams().inverseProjectionMatrix);
	shader->setUniformValue("is_perspective", renderer->projParams().isPerspective);
	if(renderer->isPicking()) {
		OVITO_CHECK_OPENGL(shader->setUniformValue("pickingBaseID", (GLint)renderer->registerSubObjectIDs(elementCount())));
		OVITO_CHECK_OPENGL(shader->setUniformValue("verticesPerElement", (GLint)_verticesPerElement));
	}

	GLint viewportCoords[4];
	glGetIntegerv(GL_VIEWPORT, viewportCoords);
	shader->setUniformValue("viewport_origin", (float)viewportCoords[0], (float)viewportCoords[1]);
	shader->setUniformValue("inverse_viewport_size", 2.0f / (float)viewportCoords[2], 2.0f / (float)viewportCoords[3]);

	_glGeometryBuffer.bind();
	if(renderer->glformat().majorVersion() < 3) {
		OVITO_CHECK_OPENGL(glEnableClientState(GL_VERTEX_ARRAY));
		OVITO_CHECK_OPENGL(glVertexPointer(3, GL_FLOAT, sizeof(ColoredVertexWithElementInfo), reinterpret_cast<const GLvoid*>(offsetof(ColoredVertexWithElementInfo, pos))));
	}
	shader->enableAttributeArray("vertex_pos");
	shader->setAttributeBuffer("vertex_pos", GL_FLOAT, offsetof(ColoredVertexWithElementInfo, pos), 3, sizeof(ColoredVertexWithElementInfo));
	if(!renderer->isPicking()) {
		shader->enableAttributeArray("cylinder_color");
		shader->setAttributeBuffer("cylinder_color", GL_FLOAT, offsetof(ColoredVertexWithElementInfo, color), 4, sizeof(ColoredVertexWithElementInfo));
	}
	shader->enableAttributeArray("cylinder_base");
	shader->setAttributeBuffer("cylinder_base", GL_FLOAT, offsetof(ColoredVertexWithElementInfo, base), 3, sizeof(ColoredVertexWithElementInfo));
	shader->enableAttributeArray("cylinder_axis");
	shader->setAttributeBuffer("cylinder_axis", GL_FLOAT, offsetof(ColoredVertexWithElementInfo, dir), 3, sizeof(ColoredVertexWithElementInfo));
	shader->enableAttributeArray("cylinder_radius");
	shader->setAttributeBuffer("cylinder_radius", GL_FLOAT, offsetof(ColoredVertexWithElementInfo, radius), 1, sizeof(ColoredVertexWithElementInfo));
	_glGeometryBuffer.release();

	if(renderer->isPicking())
		renderer->activateVertexIDs(shader, _elementCount * _verticesPerElement, true);

	OVITO_CHECK_OPENGL(renderer->glMultiDrawArrays(GL_TRIANGLE_STRIP, _stripPrimitiveVertexStarts.data(), _stripPrimitiveVertexCounts.data(), _stripPrimitiveVertexStarts.size()));

	shader->disableAttributeArray("vertex_pos");
	if(!renderer->isPicking())
		shader->disableAttributeArray("cylinder_color");
	else
		renderer->deactivateVertexIDs(shader, true);
	shader->disableAttributeArray("cylinder_base");
	shader->disableAttributeArray("cylinder_axis");
	shader->disableAttributeArray("cylinder_radius");
	if(renderer->glformat().majorVersion() < 3)
		OVITO_CHECK_OPENGL(glDisableClientState(GL_VERTEX_ARRAY));

	shader->release();
}

/******************************************************************************
* Renders the arrows in flat mode.
******************************************************************************/
void OpenGLArrowPrimitive::renderFlat(ViewportSceneRenderer* renderer)
{
	QOpenGLShaderProgram* shader;
	if(!renderer->isPicking())
		shader = _flatShader;
	else
		shader = _flatPickingShader;

	if(!shader->bind())
		throw Exception(QStringLiteral("Failed to bind OpenGL shader."));

	shader->setUniformValue("modelview_projection_matrix",
			(QMatrix4x4)(renderer->projParams().projectionMatrix * renderer->modelViewTM()));
	shader->setUniformValue("is_perspective", renderer->projParams().isPerspective);
	AffineTransformation viewModelTM = renderer->modelViewTM().inverse();
	Vector3 eye_pos = viewModelTM.translation();
	shader->setUniformValue("eye_pos", eye_pos.x(), eye_pos.y(), eye_pos.z());
	Vector3 viewDir = viewModelTM * Vector3(0,0,1);
	shader->setUniformValue("parallel_view_dir", viewDir.x(), viewDir.y(), viewDir.z());

	if(renderer->isPicking()) {
		OVITO_CHECK_OPENGL(shader->setUniformValue("pickingBaseID", (GLint)renderer->registerSubObjectIDs(elementCount())));
		OVITO_CHECK_OPENGL(shader->setUniformValue("verticesPerElement", (GLint)_verticesPerElement));
	}

	_glGeometryBuffer.bind();
	if(renderer->glformat().majorVersion() < 3) {
		OVITO_CHECK_OPENGL(glEnableClientState(GL_VERTEX_ARRAY));
		OVITO_CHECK_OPENGL(glVertexPointer(3, GL_FLOAT, sizeof(ColoredVertexWithVector), reinterpret_cast<const GLvoid*>(offsetof(ColoredVertexWithVector, pos))));
	}
	shader->enableAttributeArray("vertex_pos");
	shader->setAttributeBuffer("vertex_pos", GL_FLOAT, offsetof(ColoredVertexWithVector, pos), 3, sizeof(ColoredVertexWithVector));
	shader->enableAttributeArray("vector_base");
	shader->setAttributeBuffer("vector_base", GL_FLOAT, offsetof(ColoredVertexWithVector, base), 3, sizeof(ColoredVertexWithVector));
	shader->enableAttributeArray("vector_dir");
	shader->setAttributeBuffer("vector_dir", GL_FLOAT, offsetof(ColoredVertexWithVector, dir), 3, sizeof(ColoredVertexWithVector));
	if(!renderer->isPicking()) {
		shader->enableAttributeArray("vertex_color");
		shader->setAttributeBuffer("vertex_color", GL_FLOAT, offsetof(ColoredVertexWithVector, color), 4, sizeof(ColoredVertexWithVector));
	}
	_glGeometryBuffer.release();

	if(renderer->isPicking())
		renderer->activateVertexIDs(shader, _elementCount * _verticesPerElement);

	OVITO_CHECK_OPENGL(renderer->glMultiDrawArrays(GL_TRIANGLE_FAN, _fanPrimitiveVertexStarts.data(), _fanPrimitiveVertexCounts.data(), _fanPrimitiveVertexStarts.size()));

	shader->disableAttributeArray("vertex_pos");
	shader->disableAttributeArray("vector_base");
	shader->disableAttributeArray("vector_dir");
	if(!renderer->isPicking())
		shader->disableAttributeArray("vertex_color");
	else
		renderer->deactivateVertexIDs(shader);
	if(renderer->glformat().majorVersion() < 3)
		OVITO_CHECK_OPENGL(glDisableClientState(GL_VERTEX_ARRAY));

	shader->release();
}

};
