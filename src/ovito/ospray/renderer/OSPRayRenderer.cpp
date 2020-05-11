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
#include <ovito/core/rendering/FrameBuffer.h>
#include <ovito/core/rendering/RenderSettings.h>
#include <ovito/core/oo/CloneHelper.h>
#include <ovito/core/app/Application.h>
#include <ovito/core/app/PluginManager.h>
#include <ovito/core/dataset/scene/PipelineSceneNode.h>
#include <ovito/core/utilities/concurrent/Task.h>
#include <ovito/core/utilities/units/UnitsManager.h>
#include "OSPRayRenderer.h"

#include <QtMath>

#include <ospray/ospray_cpp.h>
#include <ospray/version.h>
#if QT_VERSION_CHECK(OSPRAY_VERSION_MAJOR, OSPRAY_VERSION_MINOR, OSPRAY_VERSION_PATCH) < QT_VERSION_CHECK(2,1,0)
	#error "OVITO requires OSPRay version 2.1.0 or newer."
#endif
#include <render/LoadBalancer.h>
#include <ospcommon/tasking/parallel_for.h>
//#include <QtGui/qopengles2ext.h>

namespace Ovito { namespace OSPRay {

IMPLEMENT_OVITO_CLASS(OSPRayRenderer);
DEFINE_REFERENCE_FIELD(OSPRayRenderer, backend);
DEFINE_PROPERTY_FIELD(OSPRayRenderer, refinementIterations);
DEFINE_PROPERTY_FIELD(OSPRayRenderer, samplesPerPixel);
DEFINE_PROPERTY_FIELD(OSPRayRenderer, maxRayRecursion);
DEFINE_PROPERTY_FIELD(OSPRayRenderer, directLightSourceEnabled);
DEFINE_PROPERTY_FIELD(OSPRayRenderer, defaultLightSourceIntensity);
DEFINE_PROPERTY_FIELD(OSPRayRenderer, defaultLightSourceAngularDiameter);
DEFINE_PROPERTY_FIELD(OSPRayRenderer, ambientLightEnabled);
DEFINE_PROPERTY_FIELD(OSPRayRenderer, ambientBrightness);
DEFINE_PROPERTY_FIELD(OSPRayRenderer, depthOfFieldEnabled);
DEFINE_PROPERTY_FIELD(OSPRayRenderer, dofFocalLength);
DEFINE_PROPERTY_FIELD(OSPRayRenderer, dofAperture);
DEFINE_PROPERTY_FIELD(OSPRayRenderer, materialShininess);
DEFINE_PROPERTY_FIELD(OSPRayRenderer, materialSpecularBrightness);
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, backend, "OSPRay backend");
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, refinementIterations, "Refinement passes");
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, samplesPerPixel, "Samples per pixel");
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, maxRayRecursion, "Max ray recursion depth");
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, directLightSourceEnabled, "Direct light");
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, defaultLightSourceIntensity, "Direct light intensity");
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, defaultLightSourceAngularDiameter, "Angular diameter");
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, ambientLightEnabled, "Ambient light");
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, ambientBrightness, "Ambient light brightness");
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, depthOfFieldEnabled, "Depth of field");
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, dofFocalLength, "Focal length");
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, dofAperture, "Aperture");
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, materialShininess, "Shininess");
SET_PROPERTY_FIELD_LABEL(OSPRayRenderer, materialSpecularBrightness, "Specular brightness");
SET_PROPERTY_FIELD_UNITS_AND_RANGE(OSPRayRenderer, refinementIterations, IntegerParameterUnit, 1, 500);
SET_PROPERTY_FIELD_UNITS_AND_RANGE(OSPRayRenderer, samplesPerPixel, IntegerParameterUnit, 1, 500);
SET_PROPERTY_FIELD_UNITS_AND_RANGE(OSPRayRenderer, maxRayRecursion, IntegerParameterUnit, 1, 100);
SET_PROPERTY_FIELD_UNITS_AND_MINIMUM(OSPRayRenderer, defaultLightSourceIntensity, FloatParameterUnit, 0);
SET_PROPERTY_FIELD_UNITS_AND_RANGE(OSPRayRenderer, defaultLightSourceAngularDiameter, AngleParameterUnit, 0, FLOATTYPE_PI/4);
SET_PROPERTY_FIELD_UNITS_AND_MINIMUM(OSPRayRenderer, ambientBrightness, FloatParameterUnit, 0);
SET_PROPERTY_FIELD_UNITS_AND_MINIMUM(OSPRayRenderer, dofFocalLength, WorldParameterUnit, 0);
SET_PROPERTY_FIELD_UNITS_AND_MINIMUM(OSPRayRenderer, dofAperture, WorldParameterUnit, 0);
SET_PROPERTY_FIELD_UNITS_AND_RANGE(OSPRayRenderer, materialShininess, FloatParameterUnit, 2, 10000);
SET_PROPERTY_FIELD_UNITS_AND_RANGE(OSPRayRenderer, materialSpecularBrightness, PercentParameterUnit, 0, 1);

/******************************************************************************
* Default constructor.
******************************************************************************/
OSPRayRenderer::OSPRayRenderer(DataSet* dataset) : NonInteractiveSceneRenderer(dataset),
	_refinementIterations(8),
	_directLightSourceEnabled(true),
	_samplesPerPixel(4),
	_maxRayRecursion(20),
	_defaultLightSourceIntensity(FloatType(3.0)),
	_defaultLightSourceAngularDiameter(0),
	_ambientLightEnabled(true),
	_ambientBrightness(FloatType(0.8)),
	_depthOfFieldEnabled(false),
	_dofFocalLength(40),
	_dofAperture(FloatType(0.5f)),
	_materialShininess(10.0),
	_materialSpecularBrightness(0.05)
{
	// Create an instance of the default OSPRay rendering backend.
	OvitoClassPtr backendClass = PluginManager::instance().findClass("OSPRayRenderer", "OSPRaySciVisBackend");
	if(backendClass == nullptr) {
		QVector<OvitoClassPtr> classList = PluginManager::instance().listClasses(OSPRayBackend::OOClass());
		if(classList.isEmpty() == false) backendClass = classList.front();
	}
	if(backendClass)
		setBackend(static_object_cast<OSPRayBackend>(backendClass->createInstance(dataset)));
}

/******************************************************************************
* Destructor.
******************************************************************************/
OSPRayRenderer::~OSPRayRenderer()
{
	// Release OSPRay device.
	ospShutdown();
}

/******************************************************************************
* Prepares the renderer for rendering of the given scene.
******************************************************************************/
bool OSPRayRenderer::startRender(DataSet* dataset, RenderSettings* settings)
{
	if(!NonInteractiveSceneRenderer::startRender(dataset, settings))
		return false;
//TODO modify this !
	// Create OSPRay device if not created yet.
	ospInit(0, nullptr);
	OSPDevice device = ospGetCurrentDevice();
	//OSPDevice device = ospGetCurrentDevice();
	if(!device) {
		device = ospNewDevice("cpu");
        ospDeviceCommit(device);
		// Load our extension module for OSPRay, which provides raytracing functions
		// for various geometry primitives (discs, cones, quadrics) needed by OVITO.
		if(ospLoadModule("ovito") != OSP_NO_ERROR)
			throwException(tr("Failed to load OSPRay extension module for Ovito: %1").arg(ospDeviceGetLastErrorMsg(device)));
        throwException(tr("Module Loaded : %1").arg(ospDeviceGetLastErrorMsg(device)));
		// Use only the number of parallel rendering threads allowed by the user.
		//ospDeviceSet1i(device, "numThreads", Application::instance()->idealThreadCount());
		int numThreadsPtr = Application::instance()->idealThreadCount();
        ospDeviceSetParam(device, "numThreads", OSP_INT,
                          static_cast<const void *>(&numThreadsPtr));

		ospDeviceCommit(device);
        ospDeviceGetProperty(device, OSP_DEVICE_VERSION);

		// Activate OSPRay device.
		ospSetCurrentDevice(device);
		//printf("Defined First Time");
	}
	else {
        if(ospLoadModule("ovito") != OSP_NO_ERROR)
            throwException(tr("Failed to load OSPRay extension module for Ovito: %1").arg(ospDeviceGetLastErrorMsg(device)));
        //printf("already defined");
    }

	return true;
}

/******************************************************************************
* Renders a single animation frame into the given frame buffer.
******************************************************************************/
bool OSPRayRenderer::renderFrame(FrameBuffer* frameBuffer, StereoRenderingTask stereoTask, SynchronousOperation operation)
{
	if(!backend())
		throwException(tr("No OSPRay rendering backend has been set."));

	operation.setProgressText(tr("Handing scene data to OSPRay renderer"));
	try {

		// Output image size:
		ospcommon::math::vec2i imgSize;
		imgSize.x = renderSettings()->outputImageWidth();
		imgSize.y = renderSettings()->outputImageHeight();

		// Make sure the target frame buffer has the right memory format.
		OVITO_ASSERT(frameBuffer->image().format() == QImage::Format_ARGB32);

		// Make a copy of the original frame buffer contents, because we are going to paint repeatedly on top
		// of the buffer.
		QImage frameBufferContents = frameBuffer->image();

		// Calculate camera information.
		Point3 cam_pos;
		Vector3 cam_dir;
		Vector3 cam_up;
		if(projParams().isPerspective) {
			cam_pos = projParams().inverseProjectionMatrix * Point3(0,0,0);
			cam_dir = projParams().inverseProjectionMatrix * Point3(0,0,0) - Point3::Origin();
			cam_up = projParams().inverseProjectionMatrix * Point3(0,1,0) - cam_pos;
			cam_pos = Point3::Origin() + projParams().inverseViewMatrix.translation();
			cam_dir = (projParams().inverseViewMatrix * cam_dir).normalized();
			cam_up = (projParams().inverseViewMatrix * cam_up).normalized();
		}
		else {
			cam_pos = projParams().inverseProjectionMatrix * Point3(0,0,-1);
			cam_dir = projParams().inverseProjectionMatrix * Point3(0,0,1) - cam_pos;
			cam_up = projParams().inverseProjectionMatrix * Point3(0,1,-1) - cam_pos;
			cam_pos = projParams().inverseViewMatrix * cam_pos;
			cam_dir = (projParams().inverseViewMatrix * cam_dir).normalized();
			cam_up = (projParams().inverseViewMatrix * cam_up).normalized();
		}

		// Create and set up OSPRay camera

		//OSPReferenceWrapper<ospray::cpp::Camera> camera(projParams().isPerspective ? "perspective" : "orthographic");
		auto camera = ospNewCamera(projParams().isPerspective ? "perspective" : "orthographic");
		//OSPReferenceWrapper<osp::Camera> camera(projParams().isPerspective ? "perspective" : "orthographic");
		ospSetFloat(camera,"aspect",imgSize.x / (float)imgSize.y);
		ospSetParam(camera,"position",OSP_VEC3F, (ospray::vec3f){(float) cam_pos.x(), (float) cam_pos.y(), (float) cam_pos.z()});
		ospSetParam(camera,"direction",OSP_VEC3F, (ospray::vec3f){(float) cam_dir.x(), (float) cam_dir.y(), (float) cam_dir.z()});
		ospSetParam(camera,"up",OSP_VEC3F, (ospray::vec3f){(float) cam_up.x(), (float) cam_up.y(), (float) cam_up.z()});
		//camera.setParam("direction", (ospray::vec3f){(float) cam_dir.x(), (float) cam_dir.y(), (float) cam_dir.z()});
		//camera.setParam("up", (ospray::vec3f){(float) cam_up.x(), (float) cam_up.y(), (float) cam_up.z()});
		ospSetFloat(camera,"nearClip",  projParams().znear);
		if(projParams().isPerspective)
			ospSetFloat(camera,"fovy", qRadiansToDegrees(projParams().fieldOfView));
		else
			ospSetFloat(camera,"height", projParams().fieldOfView * 2);
		if(projParams().isPerspective && depthOfFieldEnabled() && dofFocalLength() > 0 && dofAperture() > 0) {
			ospSetFloat(camera,"apertureRadius", dofAperture());
			ospSetFloat(camera, "focusDistance", dofFocalLength());
		}
        ospCommit(camera);

		// Create OSPRay renderer
		//OSPReferenceWrapper<ospray::cpp::Renderer> renderer{backend()->createOSPRenderer(renderSettings()->backgroundColor())};
		auto renderer = backend()->createOSPRenderer(renderSettings()->backgroundColor());
		_ospRenderer = renderer;

		// Create standard material.
		//OSPReferenceWrapper<ospray::cpp::Material> material{backend()->createOSPMaterial("OBJMaterial")};
		OSPMaterial material = backend()->createOSPMaterial("obj");
		ospSetFloat(material,"Ns", materialShininess());
		//material.setParam("Ks", ospray::vec3f {(float) materialSpecularBrightness(),(float) materialSpecularBrightness(),(float) materialSpecularBrightness()});
		ospSetParam(camera, "Ks", OSP_VEC3F, ospray::vec3f {(float) materialSpecularBrightness(),(float) materialSpecularBrightness(),(float) materialSpecularBrightness()});
		//material.commit();
		ospCommit(material);
		_ospMaterial = material;

		// // Transfer renderable geometry from OVITO to OSPRay renderer.
		//Create the scene hierarchy for rendering
		//OSPReferenceWrapper<ospray::cpp::Group> group;
		auto group = ospNewGroup();
        _ospGroup = group;


		if(!renderScene(operation.subOperation()))
			return false;
		//group.commit();


        // Create list of all instances.
        /*std::vector<OSPGeometricModel> geometricModelHandles (geometricModels.size());
        //ospray::cpp::Data lights(lightHandles.size(), OSP_LIGHT, lightHandles.data());
        for(size_t i = 0; i < geometricModels.size(); i++) {
            geometricModelHandles[i] = geometricModels[i].handle();
        }*/
        //ospray::cpp::Data geometricModelsData (geometricModelHandles.size(), OSP_GEOMETRIC_MODEL, geometricModelHandles.data());
        auto geometricModelsData = ospNewSharedData1D(geometricModels.data(),OSP_GEOMETRIC_MODEL, geometricModels.size());//TODO check when empty
        ospCommit(geometricModelsData);
        //group->setParam("geometry",geometricModelsData);
        //ospSetParam(group,"geometry",OSP_GEOMETRIC_MODEL,geometricModelsData);
        //ospSetParam(group,"geometry",OSP_GEOMETRIC_MODEL,geometricModels.data());
        ospSetObject(group,"geometry",geometricModelsData);
        ospCommit(group);

        //OSPReferenceWrapper<ospray::cpp::Instance> instance (group);
        auto instance = ospNewInstance(group);
        _ospInstance = instance;
        //OSPReferenceWrapper<ospray::cpp::World> world;
        auto world = ospNewWorld();
        _ospWorld = world;

		// Create direct light.
		//std::vector<OSPReferenceWrapper<ospray::cpp::Light>> lightSources;
		std::vector<OSPLight > lightSources;
		if(directLightSourceEnabled()) {
			OSPLight light{backend()->createOSPLight("distant")};
			Vector3 lightDir = projParams().inverseViewMatrix * Vector3(0.2f,-0.2f,-1.0f);
			ospSetParam(light,"direction",OSP_VEC3F, ospray::vec3f {(float) lightDir.x(), (float)  lightDir.y(), (float)  lightDir.z()});
			ospSetFloat(light,"intensity", defaultLightSourceIntensity());
			ospSetBool(light,"visible", false);
			ospSetFloat(light,"angularDiameter", qRadiansToDegrees(defaultLightSourceAngularDiameter()));
			lightSources.push_back(std::move(light));
		}

		// Create and setup ambient light source.
		if(ambientLightEnabled()) {
            OSPLight light{backend()->createOSPLight("ambient")};
			ospSetFloat(light,"intensity", ambientBrightness());
			lightSources.push_back(std::move(light));
		}

		// Create list of all light sources.
		//std::vector<OSPLight> lightHandles(lightSources.size());
		for(size_t i = 0; i < lightSources.size(); i++) {
		    ospCommit(lightSources[i]);
            //lightHandles[i] = lightSources[i].handle();
		}
		//OSPReferenceWrapper<ospray::cpp::Data> lights(lightHandles.size(), OSP_LIGHT, lightHandles.data());
		//ospray::cpp::Data lights(lightHandles.size(), OSP_LIGHT, lightHandles.data());
		//auto lights = ospNewSharedData1D(lightHandles.data(),OSP_LIGHT,lightHandles.size());
		auto lights = ospNewSharedData1D(lightSources.data(),OSP_LIGHT,lightSources.size());
		//lights.commit();
		ospCommit(lights);

        // Create list of all instances.
        std::vector<OSPInstance > instancesHandles(1); //TODO ? For now we only have one instance
        /*for(size_t i = 0; i < instancesHandles.size(); i++) {
            lightSources[i].commit();
            lightHandles[i] = lightSources[i].handle();
        }*/
        ospCommit(instance);
        //instancesHandles[0] = instance.handle();
        instancesHandles[0] = instance;

        //ospray::cpp::Data instances(instancesHandles.size(), OSP_INSTANCE, instancesHandles.data());
        auto instances = ospNewSharedData1D(instancesHandles.data(), OSP_INSTANCE, instancesHandles.size());
        ospCommit(instances);
        //instances.commit();

        //ospSetParam(world,"light",OSP_DATA,lights);
        ospSetObject(world,"light",lights);
        //ospSetParam(world,"instance",OSP_DATA,instances);
        ospSetObject(world,"instance",instances);
        //world.setParam("light",lights);
        //world.setParam("instance",instances);

		//renderer.setParam("model",  world);
		//renderer.setParam("camera", camera);
		//renderer.setParam("lights", lights);
		ospSetInt(renderer, "pixelSamples", std::max(samplesPerPixel(), 1));
		ospSetInt(renderer, "maxPathLength", std::max(maxRayRecursion(), 1));
		ospCommit(renderer);
		//renderer.setParam("maxPathLength", std::max(maxRayRecursion(), 1));
		//renderer.commit();

		ospCommit(instance);
		ospCommit(world);
        //instance.commit();
        //world.commit();

		// Create and set up OSPRay framebuffer.
		//OSPReferenceWrapper<ospray::cpp::FrameBuffer> osp_fb(imgSize, OSP_FB_SRGBA, OSP_FB_COLOR | OSP_FB_ACCUM);
		auto osp_fb = ospNewFrameBuffer(imgSize.x,imgSize.y,OSP_FB_SRGBA,OSP_FB_COLOR | OSP_FB_ACCUM);
		//osp_fb.clear(OSP_FB_COLOR| OSP_FB_ACCUM);
        ospResetAccumulation(osp_fb);
		//osp_fb.clear();


		// Define a custom load balancer for OSPRay that performs progressive updates of the frame buffer.
		class OVITOTiledLoadBalancer : public ospray::TiledLoadBalancer
		{
		public:
			void setProgressCallback(std::function<bool(int,int,int,int)> progressCallback) {
				_progressCallback = std::move(progressCallback);
			}

            void  renderFrame(ospray::FrameBuffer *fb, ospray::Renderer *renderer, ospray::Camera *camera, ospray::World *world) override {
				void *perFrameData = renderer->beginFrame(fb,world);
				int tileCount = fb->getTotalTiles();
				for(int taskIndex = 0; taskIndex < tileCount; taskIndex++) {
					const size_t numTiles_x = fb->getNumTiles().x;
					const size_t tile_y = taskIndex / numTiles_x;
					const size_t tile_x = taskIndex - tile_y*numTiles_x;
					const ospray::vec2i tileID(tile_x, tile_y);
					const ospray::int32 accumID = fb->accumID(tileID);

					if(fb->tileError(tileID) <= renderer->errorThreshold)
						continue;
#ifndef MAX_TILE_SIZE
#define MAX_TILE_SIZE 128
#endif
#if TILE_SIZE > MAX_TILE_SIZE
					auto tilePtr = std::make_unique<ospray::Tile>(tileID, fb->size, accumID);
					auto &tile   = *tilePtr;
#else
//					ospray::Tile __aligned(64) tile(tileID, fb->size, accumID);
					ospray::Tile __aligned(64) tile(tileID, fb->getNumPixels(), accumID);
#endif
					ospray::tasking::parallel_for(numJobs(renderer->spp, accumID), [&](size_t tIdx) {
						renderer->renderTile(fb,camera,world,perFrameData, tile, tIdx);
					});
					fb->setTile(tile);
//TODO CHECK HERE FOR PROGRESS CALLBACK
					if(_progressCallback) {
						if(!_progressCallback(tile.region.lower.x,tile.region.lower.y,tile.region.upper.x,tile.region.upper.y))
							break;
					}
				}

				renderer->endFrame(fb,perFrameData);
				return fb->endFrame(renderer->errorThreshold,camera);
			}
			std::string toString() const override {
				return "OVITOTiledLoadBalancer";
			}
		private:
			std::function<bool(int,int,int,int)> _progressCallback;
		};
		auto loadBalancer = std::make_unique<OVITOTiledLoadBalancer>();
		loadBalancer->setProgressCallback([&osp_fb,frameBuffer,&frameBufferContents,imgSize,&operation,this](int x1, int y1, int x2, int y2) {
			// Access framebuffer data and copy it to our own framebuffer.
			//uchar* fb = (uchar*)osp_fb.map(OSP_FB_COLOR);
			uchar* fb = (uchar*)ospMapFrameBuffer(osp_fb,OSP_FB_COLOR);
			OVITO_ASSERT(frameBufferContents.format() == QImage::Format_ARGB32);
			int bperline = renderSettings()->outputImageWidth() * 4;
			for(int y = y1; y < y2; y++) {
				uchar* src1 = frameBufferContents.scanLine(frameBufferContents.height() - 1 - y) + x1 * 4;
				uchar* src2 = fb + y*bperline + x1 * 4;
				uchar* dst = frameBuffer->image().scanLine(frameBuffer->image().height() - 1 - y) + x1 * 4;
				for(int x = x1; x < x2; x++, dst += 4, src1 += 4, src2 += 4) {
					// Compose colors ("source over" mode).
					float srcAlpha = (float)src2[3] / 255.0f;
					float r = (1.0f - srcAlpha) * (float)src1[0] + srcAlpha * (float)src2[2];
					float g = (1.0f - srcAlpha) * (float)src1[1] + srcAlpha * (float)src2[1];
					float b = (1.0f - srcAlpha) * (float)src1[2] + srcAlpha * (float)src2[0];
					float a = (1.0f - srcAlpha) * (float)src1[3] + srcAlpha * (float)src2[3];
					dst[0] = (uchar)(qBound(0.0f, r, 255.0f));
					dst[1] = (uchar)(qBound(0.0f, g, 255.0f));
					dst[2] = (uchar)(qBound(0.0f, b, 255.0f));
					dst[3] = (uchar)(qBound(0.0f, a, 255.0f));
				}
			}
			frameBuffer->update(QRect(x1, frameBuffer->image().height() - y2, x2 - x1, y2 - y1));
			//osp_fb.unmap(fb);
            ospUnmapFrameBuffer(fb,osp_fb);
            if(QCoreApplication::instance() && (QCoreApplication::instance()->thread() == QThread::currentThread())){
                return operation.incrementProgressValue((x2-x1) * (y2-y1));
            }
            return (false);
		});
		ospray::TiledLoadBalancer::instance = std::move(loadBalancer);

		operation.beginProgressSubSteps(refinementIterations());
		for(int iteration = 0; iteration < refinementIterations() && !operation.isCanceled(); iteration++) {
			if(iteration != 0) operation.nextProgressSubStep();
			operation.setProgressText(tr("Rendering image (pass %1 of %2)").arg(iteration+1).arg(refinementIterations()));
			operation.setProgressMaximum(imgSize.x * imgSize.y);
			//OSPRenderer Renderer r;
			//osp_fb.renderFrame(renderer,camera,world);
			ospRenderFrame(osp_fb,renderer,camera,world);
			//renderer.renderFrame(osp_fb, OSP_FB_COLOR | OSP_FB_ACCUM);
		}
		operation.endProgressSubSteps();

		// Execute recorded overlay draw calls.
		QPainter painter(&frameBuffer->image());
		for(const auto& imageCall : _imageDrawCalls) {
			QRectF rect(std::get<1>(imageCall).x(), std::get<1>(imageCall).y(), std::get<2>(imageCall).x(), std::get<2>(imageCall).y());
			painter.drawImage(rect, std::get<0>(imageCall));
			frameBuffer->update(rect.toAlignedRect());
		}
		for(const auto& textCall : _textDrawCalls) {
			QRectF pos(std::get<3>(textCall).x(), std::get<3>(textCall).y(), 0, 0);
			painter.setPen(std::get<1>(textCall));
			painter.setFont(std::get<2>(textCall));
			QRectF boundingRect;
			painter.drawText(pos, std::get<4>(textCall) | Qt::TextSingleLine | Qt::TextDontClip, std::get<0>(textCall), &boundingRect);
			frameBuffer->update(boundingRect.toAlignedRect());
		}
	}
	catch(const std::runtime_error& ex) {
		throwException(tr("OSPRay error: %1").arg(ex.what()));
	}

	return !operation.isCanceled();
}

/******************************************************************************
* Finishes the rendering pass. This is called after all animation frames have been rendered
* or when the rendering operation has been aborted.
******************************************************************************/
void OSPRayRenderer::endRender()
{
	// Release draw call buffers.
	_imageDrawCalls.clear();
	_textDrawCalls.clear();

	NonInteractiveSceneRenderer::endRender();
}

/******************************************************************************
* Renders the line geometry stored in the given buffer.
******************************************************************************/
void OSPRayRenderer::renderLines(const DefaultLinePrimitive& lineBuffer)
{
	// Lines are not supported by this renderer.
}

/******************************************************************************
* Renders the particles stored in the given buffer.
******************************************************************************/
void OSPRayRenderer::renderParticles(const DefaultParticlePrimitive& particleBuffer)
{
	auto p = particleBuffer.positions().cbegin();
	auto p_end = particleBuffer.positions().cend();
	auto c = particleBuffer.colors().cbegin();
	auto r = particleBuffer.radii().cbegin();

	const AffineTransformation tm = modelTM();

	if(particleBuffer.particleShape() == ParticlePrimitive::SphericalShape) {

		// Compile buffer with sphere data in OSPRay format.
		std::vector<ospray::vec3f> spherePosition(particleBuffer.positions().size());
		std::vector<float> sphereRadius(particleBuffer.positions().size());
		std::vector<ospray::vec4f> colorData(particleBuffer.positions().size());
		auto spherePosIter = spherePosition.begin();
		auto sphereRadiusIter = sphereRadius.begin();
		auto colorIter = colorData.begin();
		for(; p != p_end; ++p, ++c, ++r) {
			Point3 tp = tm * (*p);
			(*spherePosIter)[0] = tp.x();
			(*spherePosIter)[1] = tp.y();
			(*spherePosIter)[2] = tp.z();
			(*sphereRadiusIter) = *r;
			(*colorIter)[0] = qBound(0.0f, (float)c->r(), 1.0f);
			(*colorIter)[1] = qBound(0.0f, (float)c->g(), 1.0f);
			(*colorIter)[2] = qBound(0.0f, (float)c->b(), 1.0f);
			(*colorIter)[3] = qBound(0.0f, (float)c->a(), 1.0f);
			++spherePosIter;
			++sphereRadiusIter;
			++colorIter;
		}
		size_t nspheres = spherePosition.size();

		auto spheres = ospNewGeometry("sphere");
		//spheres.setParam("bytes_per_sphere", (int)sizeof(ospray::vec4f));
		//spheres.setParam("offset_radius", (int)sizeof(float) * 3);

		//OSPReferenceWrapper<ospray::cpp::Data> data(nspheres, OSP_VEC4F, sphereData.data());
		//ospray::cpp::Data data(nspheres, OSP_VEC4F, sphereData.data());
        //auto data = ospray::cpp::Data(spheres);
        auto positionData = ospNewSharedData1D(spherePosition.data(), OSP_VEC4F, nspheres);
        auto radiusData = ospNewSharedData1D(sphereRadius.data(), OSP_FLOAT, nspheres);
        ospCommit(positionData);
        ospCommit(radiusData);
		//ospCommit(data);
		//ospSetParam(spheres,"sphere.position",OSP_VEC3F,spherePosition.data());
		//ospSetParam(spheres,"sphere.radius",OSP_VEC3F,sphereRadius.data());
		ospSetObject(spheres,"sphere.position",positionData);
		ospSetObject(spheres,"sphere.radius",radiusData);
		//spheres.setParam("spheres", data);
        ospCommit(spheres);

		//data = ospray::cpp::Data(nspheres, OSP_VEC4F, colorData.data());
        //auto data = ospNewSharedData1D(colorData.data(), OSP_VEC4F, nspheres);
        //ospCommit(data);
		//spheres.setParam("color", data);
        //spheres.commit();

        auto spheresModel = ospNewGeometricModel(spheres);
        //spheres.setMaterial(*_ospMaterial);
        //ospSetObject(spheres.handle(),"material",_ospMaterial->handle());
        //spheresModel.setParam("material",*_ospMaterial);
        auto colorOSPData = ospNewSharedData1D(colorData.data(), OSP_VEC4F, nspheres);
        ospCommit(colorOSPData);
        //ospSetParam(spheresModel,"color",OSP_VEC4F,colorData.data());
        ospSetObject(spheresModel,"color",colorOSPData);
        //ospSetParam(spheresModel,"material",OSP_MATERIAL,_ospMaterial);
        ospSetObject(spheresModel,"material",_ospMaterial);
        //spheresModel.commit();
        ospCommit(spheresModel);
		geometricModels.push_back(spheresModel);
		//_ospGroup->addGeometry(spheres);
	}
	else if(particleBuffer.particleShape() == ParticlePrimitive::SquareCubicShape || particleBuffer.particleShape() == ParticlePrimitive::BoxShape) {
		// Rendering cubic/box particles.

		// We will pass the particle geometry as a triangle mesh to OSPRay.
		//std::vector<Point_3<float>> vertices;
		std::vector<ospray::vec3f> vertices;
		//std::vector<ColorAT<float>> colors;
		std::vector<ospray::vec4f> colors;
		//std::vector<Vector_3<float>> normals;
		std::vector<ospray::vec3f> normals;
		std::vector<int> indices;
		vertices.reserve(particleBuffer.positions().size() * 6 * 4);
		colors.reserve(particleBuffer.positions().size() * 6 * 4);
		normals.reserve(particleBuffer.positions().size() * 6 * 4);
		indices.reserve(particleBuffer.positions().size() * 6 * 2 * 3);

		auto shape = particleBuffer.shapes().begin();
		auto shape_end = particleBuffer.shapes().end();
		auto orientation = particleBuffer.orientations().cbegin();
		auto orientation_end = particleBuffer.orientations().cend();
		for(; p != p_end; ++p, ++c, ++r) {
			if(c->a() <= 0) continue;
			const ColorAT<float> color = (ColorAT<float>)*c;
			for(int i = 0; i < 6*4; i++) {
				//colors.push_back(color);
				colors.push_back(ospray::vec4f(color.r(),color.g(),color.b(),color.a()));
			}
			Point_3<float> tp = (Point_3<float>)(tm * (*p));
			QuaternionT<float> quat(0,0,0,1);
			if(orientation != orientation_end) {
				quat = (QuaternionT<float>)*orientation++;
				// Normalize quaternion.
				float c = sqrt(quat.dot(quat));
				if(c <= 1e-9f)
					quat.setIdentity();
				else
					quat /= c;
			}
			Vector_3<float> s((float)*r);
			if(shape != shape_end) {
				s = (Vector_3<float>)*shape++;
				if(s == Vector_3<float>::Zero())
					s = Vector_3<float>(*r);
			}
			const Point_3<float> corners[8] = {
					tp + quat * Vector_3<float>(-s.x(), -s.y(), -s.z()),
					tp + quat * Vector_3<float>( s.x(), -s.y(), -s.z()),
					tp + quat * Vector_3<float>( s.x(),  s.y(), -s.z()),
					tp + quat * Vector_3<float>(-s.x(),  s.y(), -s.z()),
					tp + quat * Vector_3<float>(-s.x(), -s.y(),  s.z()),
					tp + quat * Vector_3<float>( s.x(), -s.y(),  s.z()),
					tp + quat * Vector_3<float>( s.x(),  s.y(),  s.z()),
					tp + quat * Vector_3<float>(-s.x(),  s.y(),  s.z())
			};
			const Vector_3<float> faceNormals[6] = {
				quat * Vector_3<float>(-1,0,0), quat * Vector_3<float>(1,0,0),
				quat * Vector_3<float>(0,-1,0), quat * Vector_3<float>(0,1,0),
				quat * Vector_3<float>(0,0,-1), quat * Vector_3<float>(0,0,1)
			};
			int baseIndex;

			// -X face
			baseIndex = (int)vertices.size();
			vertices.push_back(ospray::vec3f (corners[0].x(),corners[0].y(),corners[0].z())); vertices.push_back(ospray::vec3f (corners[3].x(),corners[3].y(),corners[3].z())); vertices.push_back(ospray::vec3f (corners[7].x(),corners[7].y(),corners[7].z())); vertices.push_back(ospray::vec3f (corners[4].x(),corners[4].y(),corners[4].z()));
			indices.push_back(baseIndex + 0); indices.push_back(baseIndex + 1); indices.push_back(baseIndex + 2);
			indices.push_back(baseIndex + 0); indices.push_back(baseIndex + 2); indices.push_back(baseIndex + 3);
			for(int i = 0; i < 4; i++) normals.push_back(ospray::vec3f(faceNormals[0].x(),faceNormals[0].y(),faceNormals[0].z()));
			// +X face
			baseIndex = (int)vertices.size();
			//vertices.push_back(corners[1]); vertices.push_back(corners[5]); vertices.push_back(corners[6]); vertices.push_back(corners[2]);
            vertices.push_back(ospray::vec3f (corners[1].x(),corners[1].y(),corners[1].z())); vertices.push_back(ospray::vec3f (corners[5].x(),corners[5].y(),corners[5].z())); vertices.push_back(ospray::vec3f (corners[6].x(),corners[6].y(),corners[6].z())); vertices.push_back(ospray::vec3f (corners[2].x(),corners[2].y(),corners[2].z()));
			indices.push_back(baseIndex + 0); indices.push_back(baseIndex + 1); indices.push_back(baseIndex + 2);
			indices.push_back(baseIndex + 0); indices.push_back(baseIndex + 2); indices.push_back(baseIndex + 3);
			//for(int i = 0; i < 4; i++) normals.push_back(faceNormals[1]);
            for(int i = 0; i < 4; i++) normals.push_back(ospray::vec3f(faceNormals[1].x(),faceNormals[1].y(),faceNormals[1].z()));
			// -Y face
			baseIndex = (int)vertices.size();
			//vertices.push_back(corners[0]); vertices.push_back(corners[4]); vertices.push_back(corners[5]); vertices.push_back(corners[1]);
            vertices.push_back(ospray::vec3f (corners[0].x(),corners[0].y(),corners[0].z())); vertices.push_back(ospray::vec3f (corners[4].x(),corners[4].y(),corners[4].z())); vertices.push_back(ospray::vec3f (corners[5].x(),corners[5].y(),corners[5].z())); vertices.push_back(ospray::vec3f (corners[1].x(),corners[1].y(),corners[1].z()));
			indices.push_back(baseIndex + 0); indices.push_back(baseIndex + 1); indices.push_back(baseIndex + 2);
			indices.push_back(baseIndex + 0); indices.push_back(baseIndex + 2); indices.push_back(baseIndex + 3);
			//for(int i = 0; i < 4; i++) normals.push_back(faceNormals[2]);
            for(int i = 0; i < 4; i++) normals.push_back(ospray::vec3f(faceNormals[2].x(),faceNormals[2].y(),faceNormals[2].z()));
			// +Y face
			baseIndex = (int)vertices.size();
			//vertices.push_back(corners[2]); vertices.push_back(corners[6]); vertices.push_back(corners[7]); vertices.push_back(corners[3]);
            vertices.push_back(ospray::vec3f (corners[2].x(),corners[2].y(),corners[2].z())); vertices.push_back(ospray::vec3f (corners[6].x(),corners[6].y(),corners[6].z())); vertices.push_back(ospray::vec3f (corners[7].x(),corners[7].y(),corners[7].z())); vertices.push_back(ospray::vec3f (corners[3].x(),corners[3].y(),corners[3].z()));
            indices.push_back(baseIndex + 0); indices.push_back(baseIndex + 1); indices.push_back(baseIndex + 2);
			indices.push_back(baseIndex + 0); indices.push_back(baseIndex + 2); indices.push_back(baseIndex + 3);
			//for(int i = 0; i < 4; i++) normals.push_back(faceNormals[3]);
            for(int i = 0; i < 4; i++) normals.push_back(ospray::vec3f(faceNormals[3].x(),faceNormals[3].y(),faceNormals[3].z()));
			// -Z face
			baseIndex = (int)vertices.size();
			//vertices.push_back(corners[0]); vertices.push_back(corners[1]); vertices.push_back(corners[2]); vertices.push_back(corners[3]);
            vertices.push_back(ospray::vec3f (corners[0].x(),corners[0].y(),corners[0].z())); vertices.push_back(ospray::vec3f (corners[1].x(),corners[1].y(),corners[1].z()));  vertices.push_back(ospray::vec3f (corners[2].x(),corners[2].y(),corners[2].z()));vertices.push_back(ospray::vec3f (corners[3].x(),corners[3].y(),corners[3].z()));
			indices.push_back(baseIndex + 0); indices.push_back(baseIndex + 1); indices.push_back(baseIndex + 2);
			indices.push_back(baseIndex + 0); indices.push_back(baseIndex + 2); indices.push_back(baseIndex + 3);
			//for(int i = 0; i < 4; i++) normals.push_back(faceNormals[4]);
            for(int i = 0; i < 4; i++) normals.push_back(ospray::vec3f(faceNormals[4].x(),faceNormals[4].y(),faceNormals[4].z()));
			// +Z face
			baseIndex = (int)vertices.size();
			//vertices.push_back(corners[4]); vertices.push_back(corners[7]); vertices.push_back(corners[6]); vertices.push_back(corners[5]);
            vertices.push_back(ospray::vec3f (corners[4].x(),corners[4].y(),corners[4].z())); vertices.push_back(ospray::vec3f (corners[7].x(),corners[7].y(),corners[7].z()));  vertices.push_back(ospray::vec3f (corners[6].x(),corners[6].y(),corners[6].z())); vertices.push_back(ospray::vec3f (corners[5].x(),corners[5].y(),corners[5].z()));
			indices.push_back(baseIndex + 0); indices.push_back(baseIndex + 1); indices.push_back(baseIndex + 2);
			indices.push_back(baseIndex + 0); indices.push_back(baseIndex + 2); indices.push_back(baseIndex + 3);
			//for(int i = 0; i < 4; i++) normals.push_back(faceNormals[5]);
            for(int i = 0; i < 4; i++) normals.push_back(ospray::vec3f(faceNormals[5].x(),faceNormals[5].y(),faceNormals[5].z()));
		}
		OVITO_ASSERT(normals.size() == colors.size());
		OVITO_ASSERT(normals.size() == vertices.size());

		// Note: This for-loop is a workaround for a bug in OSPRay 1.4.2, which crashes when rendering
		// geometry with a color memory buffer whose size exceeds 2^31 bytes. We split up the geometry
		// into chunks to stay below the 2^31 bytes limit.
		size_t nparticles = (colors.size() / (6 * 4));
		size_t maxChunkSize = ((1ull << 31) / (sizeof(ColorAT<float>) * 6 * 4)) - 1;
		for(size_t chunkOffset = 0; chunkOffset < nparticles; chunkOffset += maxChunkSize) {

			//OSPReferenceWrapper<ospray::cpp::Geometry> triangles("triangles");
			auto triangles = ospNewGeometry("triangles");

			size_t chunkSize = std::min(maxChunkSize, nparticles - chunkOffset);
			//ospray::cpp::Data data(chunkSize * 6 * 4, OSP_VEC3F, vertices.data() + (chunkOffset * 6 * 4));
            auto data = ospNewSharedData1D(vertices.data()  + (chunkOffset * 6 * 4),OSP_VEC3F, chunkSize * 6 * 4);
            //ospray::cpp::Data data(chunkSize * 6 * 4, OSP_VEC3F, vertices.data());
            //ospray::cpp::Data data(vertices);
			//data.commit();
			ospCommit(data);
			//ospSetParam(triangles,"vertex",OSP_VEC3F, data);
			ospSetObject(triangles,"vertex", data);

			//data = ospray::cpp::Data(chunkSize * 6 * 4,OSP_VEC4F, colors.data() + (chunkOffset * 6 * 4));
			data = ospNewSharedData1D(colors.data() + (chunkOffset * 6 * 4),OSP_VEC4F, chunkSize * 6 * 4);
            ospCommit(data);
			//ospSetParam(triangles,"vertex.color",OSP_VEC4F, data);
			ospSetObject(triangles,"vertex.color",data);

			//data = ospray::cpp::Data(chunkSize * 6 * 4, OSP_VEC3F, normals.data() + (chunkOffset * 6 * 4));
            data = ospNewSharedData1D(normals.data() + (chunkOffset * 6 * 4),OSP_VEC3F, chunkSize * 6 * 4);
            ospCommit(data);
			ospSetObject(triangles,"vertex.normal", data);

			//data = ospray::cpp::Data(chunkSize * 6 * 2, OSP_VEC3I, indices.data() + (chunkOffset * 6 * 3 * 2));
            data = ospNewSharedData1D(indices.data() + (chunkOffset * 6 * 3*2),OSP_VEC3I, chunkSize * 6 * 2);
            ospCommit(data);
			ospSetObject(triangles,"index", data);

            ospCommit(triangles);

            OSPGeometricModel trianglesModel = ospNewGeometricModel(triangles);
            //triangles.setMaterial(*_ospMaterial);
            //ospSetObject(triangles.handle(),"material",_ospMaterial->handle());
            ospSetParam(trianglesModel,"material",OSP_MATERIAL, _ospMaterial);
            ospCommit(trianglesModel);
            geometricModels.push_back(trianglesModel);
            //_ospGroup->addGeometry(triangles);
		}
	}
	else if(particleBuffer.particleShape() == ParticlePrimitive::EllipsoidShape) {
		// Rendering ellipsoid particles.
		const Matrix3 linear_tm = tm.linear();
		auto shape = particleBuffer.shapes().cbegin();
		auto shape_end = particleBuffer.shapes().cend();
		auto orientation = particleBuffer.orientations().cbegin();
		auto orientation_end = particleBuffer.orientations().cend();
		std::vector<std::array<float,10>> quadricsData(particleBuffer.positions().size());
		std::vector<ospcommon::math::vec3f> quadricsCenter(particleBuffer.positions().size());
		std::vector<float> quadricsRadius(particleBuffer.positions().size());
		std::vector<ospray::vec4f> colorData(particleBuffer.positions().size());
		auto quadricIter = quadricsData.begin();
		auto quadricCenterIter = quadricsCenter.begin();
		auto quadricRadiusIter = quadricsRadius.begin();
		auto colorIter = colorData.begin();
		for(; p != p_end && shape != shape_end; ++p, ++c, ++shape, ++r) {
			if(c->a() <= 0) continue;
			Point3 tp = tm * (*p);
			Quaternion quat(0,0,0,1);
			if(orientation != orientation_end) {
				quat = *orientation++;
				// Normalize quaternion.
				FloatType c = sqrt(quat.dot(quat));
				if(c == 0)
					quat.setIdentity();
				else
					quat /= c;
			}
			(*quadricCenterIter)[0] = tp.x();
			(*quadricCenterIter)[1] = tp.y();
			(*quadricCenterIter)[2] = tp.z();
			if(shape->x() != 0 && shape->y() != 0 && shape->z() != 0) {
				Matrix3 qmat(FloatType(1)/(shape->x()*shape->x()), 0, 0,
						     0, FloatType(1)/(shape->y()*shape->y()), 0,
						     0, 0, FloatType(1)/(shape->z()*shape->z()));
				Matrix3 rot = linear_tm * Matrix3::rotation(quat);
				Matrix3 quadric = rot * qmat * rot.transposed();
				(*quadricRadiusIter) = std::max(shape->x(), std::max(shape->y(), shape->z()));
				(*quadricIter)[0] = quadric(0,0);
				(*quadricIter)[1] = quadric(0,1);
				(*quadricIter)[2] = quadric(0,2);
				(*quadricIter)[3] = 0;
				(*quadricIter)[4] =	quadric(1,1);
				(*quadricIter)[5] = quadric(1,2);
				(*quadricIter)[6] = 0;
				(*quadricIter)[7] = quadric(2,2);
				(*quadricIter)[8] = 0;
				(*quadricIter)[9] = -1;
			}
			else {
				(*quadricRadiusIter) = *r;
				(*quadricIter)[0] = FloatType(1)/((*r)*(*r));
				(*quadricIter)[1] = 0;
				(*quadricIter)[2] = 0;
				(*quadricIter)[3] = 0;
				(*quadricIter)[4] =	FloatType(1)/((*r)*(*r));
				(*quadricIter)[5] = 0;
				(*quadricIter)[6] = 0;
				(*quadricIter)[7] = FloatType(1)/((*r)*(*r));
				(*quadricIter)[8] = 0;
				(*quadricIter)[9] = -1;
			}
			(*colorIter)[0] = c->r();
			(*colorIter)[1] = c->g();
			(*colorIter)[2] = c->b();
			(*colorIter)[3] = c->a();
			++quadricIter;
			++colorIter;
		}
		size_t nquadrics = quadricIter - quadricsData.begin();
		if(nquadrics == 0) return;

		// Note: This for-loop is a workaround for a bug in OSPRay 1.4.2, which crashes when rendering
		// geometry with a color memory buffer whose size exceeds 2^31 bytes. We split up the geometry
		// into chunks to stay below the 2^31 bytes limit.
		size_t maxChunkSize = ((1ull << 31) / sizeof(std::array<float,14>)) - 1;
		for(size_t chunkOffset = 0; chunkOffset < nquadrics; chunkOffset += maxChunkSize) {

			//OSPReferenceWrapper<ospray::cpp::Geometry> quadrics("quadrics");
			auto quadrics = ospNewGeometry("quadrics");

			size_t chunkSize = std::min(maxChunkSize, nquadrics - chunkOffset);
			//ospray::cpp::Data data(chunkSize * 14, OSP_FLOAT, (float*) quadricsData.data() + chunkOffset);
			//auto data = ospNewSharedData1D((float*) quadricsData.data() + chunkOffset, OSP_FLOAT, chunkSize * 14); //TODO : TO CHECK : Casting pointer ???
			auto data = ospNewSharedData1D((float*) quadricsData.data() + chunkOffset, OSP_FLOAT, chunkSize * 10); //TODO : TO CHECK : Casting pointer ???
			auto centerData = ospNewSharedData1D((float*) quadricsCenter.data() + chunkOffset, OSP_VEC3F, chunkSize);
			auto radiusData = ospNewSharedData1D((float*) quadricsRadius.data() + chunkOffset, OSP_FLOAT, chunkSize);
			//data.commit();
			ospCommit(data);
			ospCommit(centerData);
			ospCommit(radiusData);
			//ospSetParam(quadrics,"quadrics.coeff",OSP_FLOAT, data);
			ospSetObject(quadrics,"quadrics.coeff",data);

			//ospSetParam(quadrics, "quadrics.center",OSP_VEC3F,quadricsCenter.data());
			ospSetObject(quadrics, "quadrics.center",centerData);
			//ospSetParam(quadrics, "quadrics.radius",OSP_FLOAT,quadricsRadius.data());
			ospSetObject(quadrics, "quadrics.radius",radiusData);
            ospCommit(quadrics);

/*
			data = ospray::cpp::Data(chunkSize, OSP_VEC4F, colorData.data() + chunkOffset);
			data.commit();
			quadrics.setParam("color", data);
            quadrics.commit();*/

            //ospray::cpp::GeometricModel quadricsModel(quadrics);
            auto quadricsModel= ospNewGeometricModel(quadrics);
            //quadrics.setMaterial(*_ospMaterial);
            //ospSetObject(quadrics.handle(),"material",_ospMaterial->handle());
            auto colorOSPData = ospNewSharedData1D(colorData.data() + chunkOffset, OSP_VEC4F, chunkSize);
            ospCommit(colorOSPData);
            ospSetObject(quadrics,"color",colorOSPData);
            ospSetParam(quadricsModel,"material",OSP_MATERIAL,_ospMaterial);
            //quadricsModel.commit();
            ospCommit(quadricsModel);
            geometricModels.push_back(quadricsModel);
			//_ospGroup->addGeometry(quadrics);
		}
	}
}

/******************************************************************************
* Renders the arrow elements stored in the given buffer.
******************************************************************************/
void OSPRayRenderer::renderArrows(const DefaultArrowPrimitive& arrowBuffer)
{
	const AffineTransformation tm = modelTM();

	// Compile buffer with cylinder data in OSPRay format.
	std::vector<ospcommon::math::vec3f> cylVertex0Data(arrowBuffer.elements().size());
	std::vector<ospcommon::math::vec3f> cylVertex1Data(arrowBuffer.elements().size());
	std::vector<float> cylRadiusData(arrowBuffer.elements().size());
	std::vector<ospray::vec4f> colorData(arrowBuffer.elements().size());
	std::vector<ospcommon::math::vec3f> discCenterData(arrowBuffer.elements().size()*2);
	std::vector<ospcommon::math::vec3f> discNormalData(arrowBuffer.elements().size()*2);
	std::vector<float> discRadiusData(arrowBuffer.elements().size()*2);
	std::vector<ospray::vec4f> discColorData(discCenterData.size());
	std::vector<ospcommon::math::vec3f> coneCenterData(arrowBuffer.shape() == ArrowPrimitive::CylinderShape ? 0 : arrowBuffer.elements().size());
	std::vector<ospcommon::math::vec3f> coneAxisData(arrowBuffer.shape() == ArrowPrimitive::CylinderShape ? 0 : arrowBuffer.elements().size());
	std::vector<float> coneRadiusData(arrowBuffer.shape() == ArrowPrimitive::CylinderShape ? 0 : arrowBuffer.elements().size());
	std::vector<ospray::vec4f> coneColorData(coneCenterData.size());
	auto cylVertex0Iter = cylVertex0Data.begin();
	auto cylVertex1Iter = cylVertex1Data.begin();
	auto cylRadiusIter = cylRadiusData.begin();
	auto colorIter = colorData.begin();
	auto discNormalIter = discNormalData.begin();
	auto discCenterIter = discCenterData.begin();
	auto discRadiusIter = discRadiusData.begin();
	auto discColorIter = discColorData.begin();
	auto coneCenterIter = coneCenterData.begin();
	auto coneAxisIter = coneAxisData.begin();
	auto coneRadiusIter = coneRadiusData.begin();
	auto coneColorIter = coneColorData.begin();
	for(const DefaultArrowPrimitive::ArrowElement& element : arrowBuffer.elements()) {
		Point3 tp = tm * element.pos;
		Vector3 ta;
		if(arrowBuffer.shape() == ArrowPrimitive::CylinderShape) {
			ta = tm * element.dir;
			Vector3 normal = ta;
			normal.normalizeSafely();
			(*discCenterIter)[0] = tp.x();
			(*discCenterIter)[1] = tp.y();
			(*discCenterIter)[2] = tp.z();
			(*discNormalIter)[0] = -normal.x();
			(*discNormalIter)[1] = -normal.y();
			(*discNormalIter)[2] = -normal.z();
			(*discRadiusIter) = element.width;
			(*discColorIter)[0] = element.color.r();
			(*discColorIter)[1] = element.color.g();
			(*discColorIter)[2] = element.color.b();
			(*discColorIter)[3] = element.color.a();
			++discNormalIter;
			++discCenterIter;
			++discRadiusIter;
			++discColorIter;
			(*discCenterIter)[0] = tp.x() + ta.x();
			(*discCenterIter)[1] = tp.y() + ta.y();
			(*discCenterIter)[2] = tp.z() + ta.z();
			(*discNormalIter)[0] = normal.x();
			(*discNormalIter)[1] = normal.y();
			(*discNormalIter)[2] = normal.z();
			(*discRadiusIter) = element.width;
			(*discColorIter)[0] = element.color.r();
			(*discColorIter)[1] = element.color.g();
			(*discColorIter)[2] = element.color.b();
			(*discColorIter)[3] = element.color.a();
            ++discNormalIter;
            ++discCenterIter;
            ++discRadiusIter;
			++discColorIter;
		}
		else {
			FloatType arrowHeadRadius = element.width * FloatType(2.5);
			FloatType arrowHeadLength = arrowHeadRadius * FloatType(1.8);
			FloatType length = element.dir.length();
			if(length == 0)
				continue;

			if(length > arrowHeadLength) {
				tp = tm * element.pos;
				ta = tm * (element.dir * ((length - arrowHeadLength) / length));
				Vector3 tb = tm * (element.dir * (arrowHeadLength / length));
				Vector3 normal = ta;
				normal.normalizeSafely();
				(*discCenterIter)[0] = tp.x();
				(*discCenterIter)[1] = tp.y();
				(*discCenterIter)[2] = tp.z();
				(*discNormalIter)[0] = -normal.x();
				(*discNormalIter)[1] = -normal.y();
				(*discNormalIter)[2] = -normal.z();
				(*discRadiusIter) = element.width;
				(*discColorIter)[0] = element.color.r();
				(*discColorIter)[1] = element.color.g();
				(*discColorIter)[2] = element.color.b();
				(*discColorIter)[3] = element.color.a();
                ++discNormalIter;
                ++discCenterIter;
                ++discRadiusIter;
				++discColorIter;
				(*discCenterIter)[0] = tp.x() + ta.x();
				(*discCenterIter)[1] = tp.y() + ta.y();
				(*discCenterIter)[2] = tp.z() + ta.z();
				(*discNormalIter)[0] = -normal.x();
				(*discNormalIter)[1] = -normal.y();
				(*discNormalIter)[2] = -normal.z();
				(*discRadiusIter) = arrowHeadRadius;
				(*discColorIter)[0] = element.color.r();
				(*discColorIter)[1] = element.color.g();
				(*discColorIter)[2] = element.color.b();
				(*discColorIter)[3] = element.color.a();
                ++discNormalIter;
                ++discCenterIter;
                ++discRadiusIter;
				++discColorIter;
				(*coneCenterIter)[0] = tp.x() + ta.x() + tb.x();
				(*coneCenterIter)[1] = tp.y() + ta.y() + tb.y();
				(*coneCenterIter)[2] = tp.z() + ta.z() + tb.z();
				(*coneAxisIter)[0] = -tb.x();
				(*coneAxisIter)[1] = -tb.y();
				(*coneAxisIter)[2] = -tb.z();
				(*coneRadiusIter) = arrowHeadRadius;
				(*coneColorIter)[0] = element.color.r();
				(*coneColorIter)[1] = element.color.g();
				(*coneColorIter)[2] = element.color.b();
				(*coneColorIter)[3] = element.color.a();
				++coneCenterIter;
				++coneAxisIter;
				++coneRadiusIter;
				++coneColorIter;
			}
			else {
				FloatType r = arrowHeadRadius * length / arrowHeadLength;
				ta = tm * element.dir;
				Vector3 normal = ta;
				normal.normalizeSafely();
				(*discCenterIter)[0] = tp.x();
				(*discCenterIter)[1] = tp.y();
				(*discCenterIter)[2] = tp.z();
				(*discNormalIter)[0] = -normal.x();
				(*discNormalIter)[1] = -normal.y();
				(*discNormalIter)[2] = -normal.z();
				(*discRadiusIter) = r;
				(*discColorIter)[0] = element.color.r();
				(*discColorIter)[1] = element.color.g();
				(*discColorIter)[2] = element.color.b();
				(*discColorIter)[3] = element.color.a();
                ++discNormalIter;
                ++discCenterIter;
                ++discRadiusIter;
				++discColorIter;
				(*coneCenterIter)[0] = tp.x() + ta.x();
				(*coneCenterIter)[1] = tp.y() + ta.y();
				(*coneCenterIter)[2] = tp.z() + ta.z();
				(*coneAxisIter)[0] = -ta.x();
				(*coneAxisIter)[1] = -ta.y();
				(*coneAxisIter)[2] = -ta.z();
				(*coneRadiusIter) = r;
				(*coneColorIter)[0] = element.color.r();
				(*coneColorIter)[1] = element.color.g();
				(*coneColorIter)[2] = element.color.b();
				(*coneColorIter)[3] = element.color.a();
                ++coneCenterIter;
                ++coneAxisIter;
                ++coneRadiusIter;
				++coneColorIter;
				continue;
			}
		}
		(*cylVertex0Iter)[0] = tp.x();
		(*cylVertex0Iter)[1] = tp.y();
		(*cylVertex0Iter)[2] = tp.z();
		(*cylVertex1Iter)[0] = tp.x() + ta.x();
		(*cylVertex1Iter)[1] = tp.y() + ta.y();
		(*cylVertex1Iter)[2] = tp.z() + ta.z();
		(*cylRadiusIter) = element.width;
		(*colorIter)[0] = element.color.r();
		(*colorIter)[1] = element.color.g();
		(*colorIter)[2] = element.color.b();
		(*colorIter)[3] = element.color.a();
		++cylVertex0Iter;
		++cylVertex1Iter;
		++cylRadiusIter;
		++colorIter;
	}
	size_t ncylinders = cylRadiusIter - cylRadiusData.begin();
	if(ncylinders != 0) {
		//OSPReferenceWrapper<ospray::cpp::Geometry> cylinders("cylinders");
		auto cylinders = ospNewGeometry("cylinders");
		//cylinders.setParam("bytes_per_cylinder", (int)sizeof(float) * 7);
		//cylinders.setParam("offset_radius", (int)sizeof(float) * 6);

		//ospray::cpp::Data data(ncylinders * 7, OSP_FLOAT, (float*) cylData.data()); //TODO : TO CHECK : Casting pointer ???
        auto position0Data = ospNewSharedData1D(cylVertex0Data.data(), OSP_VEC3F, ncylinders);
        auto position1Data = ospNewSharedData1D(cylVertex1Data.data(), OSP_VEC3F, ncylinders);
        auto radiusData = ospNewSharedData1D(cylRadiusData.data(), OSP_FLOAT, ncylinders);
        ospCommit(position0Data);
        ospCommit(position1Data);
        ospCommit(radiusData);
//		cylinders.setParam("cylinders", data);
//
//        ospSetParam(cylinders,"cylinder.radius",OSP_FLOAT, cylRadiusData.data());
//        ospSetParam(cylinders,"cylinder.position0",OSP_VEC3F, cylVertex0Data.data());
//        ospSetParam(cylinders,"cylinder.position1",OSP_VEC3F, cylVertex1Data.data());
        ospSetObject(cylinders,"cylinder.radius",radiusData);
        ospSetObject(cylinders,"cylinder.position0",position0Data);
        ospSetObject(cylinders,"cylinder.position1",position1Data);
        ospCommit (cylinders);

		//data = ospray::cpp::Data(ncylinders, OSP_VEC4F, colorData.data());
        auto colorOSPData = ospNewSharedData1D(colorData.data(),OSP_VEC4F, ncylinders);
        ospCommit(colorOSPData);
//		cylinders.setParam("color", data);
//      cylinders.commit();

        //ospray::cpp::Geometry cylindersModel(cylinders);
        auto cylindersModel = ospNewGeometricModel(cylinders);
        //cylinders.setMaterial(*_ospMaterial);
        //ospSetObject(cylinders.handle(),"material",_ospMaterial->handle());
        //ospSetObject(cylinders.handle(),"material",_ospMaterial->handle());
        //cylinders.setParam("material",*_ospMaterial);
        ospSetParam(cylindersModel,"material",OSP_MATERIAL,_ospMaterial);
        //ospSetParam(cylindersModel,"color",OSP_VEC4F,colorData.data());
        ospSetObject(cylindersModel,"color",colorOSPData);
        ospCommit(cylindersModel);
        geometricModels.push_back(cylindersModel);
		//_ospGroup->addGeometry(cylinders);
	}

	size_t ndiscs = discRadiusIter - discRadiusData.begin();
	if(ndiscs != 0) {
		//OSPReferenceWrapper<ospray::cpp::Geometry> discs("discs");
		auto discs = ospNewGeometry("discs");
//		discs.setParam("bytes_per_disc", (int)sizeof(float) * 7);
//		discs.setParam("offset_center", (int)sizeof(float) * 0);
//		discs.setParam("offset_normal", (int)sizeof(float) * 3);
//		discs.setParam("offset_radius", (int)sizeof(float) * 6);
//		ospray::cpp::Data data(ndiscs * 7, OSP_FLOAT, (float*) discData.data()); //TODO : TO CHECK : Casting pointer ???
//		data.commit();
//		discs.setParam("discs", data);
        auto radiusData = ospNewSharedData1D(discRadiusData.data(), OSP_FLOAT, ndiscs);
        auto positionData = ospNewSharedData1D(discCenterData.data(), OSP_VEC3F, ndiscs);
        auto normalData = ospNewSharedData1D(discNormalData.data(), OSP_VEC3F, ndiscs);
        ospCommit(radiusData);
        ospCommit(positionData);
        ospCommit(normalData);

//      ospSetParam(discs,"disc.radius",OSP_FLOAT, discRadiusData.data());
//		ospSetParam(discs,"disc.position",OSP_VEC3F, discCenterData.data());
//		ospSetParam(discs,"disc.normal",OSP_VEC3F, discNormalData.data());
        ospSetObject(discs,"disc.radius",radiusData);
        ospSetObject(discs,"disc.position",positionData);
        ospSetObject(discs,"disc.normal",normalData);
        ospCommit (discs);


//		data = ospray::cpp::Data(ndiscs, OSP_VEC4F, discColorData.data());
//		data.commit();
//		discs.setParam("color", data);
//        discs.commit();

        auto colorOSPData = ospNewSharedData1D( discColorData.data(), OSP_VEC4F,ndiscs);
        ospCommit(colorOSPData);

        //ospray::cpp::GeometricModel discsModel(discs);
        auto discsModel = ospNewGeometricModel(discs);
        //discs.setMaterial(*_ospMaterial);
        //ospSetObject(discs.handle(),"material",_ospMaterial->handle());
        ospSetParam(discsModel,"material",OSP_MATERIAL,_ospMaterial);
        ospSetObject(discsModel,"color",colorOSPData);
        //discsModel.commit();
        ospCommit(discsModel);
        geometricModels.push_back(discsModel);
		//_ospGroup->addGeometry(discs);
	}

	size_t ncones = coneCenterIter - coneCenterData.begin();
	if(ncones != 0) {
//		OSPReferenceWrapper<ospray::cpp::Geometry> cones("cones");
//		cones.setParam("bytes_per_cone", (int)sizeof(float) * 7);
//		cones.setParam("offset_center", (int)sizeof(float) * 0);
//		cones.setParam("offset_axis", (int)sizeof(float) * 3);
//		cones.setParam("offset_radius", (int)sizeof(float) * 6);
		auto cones= ospNewGeometry("cones");

//		ospray::cpp::Data data(ncones * 7, OSP_FLOAT, (float *)coneData.data()); //TODO : TO CHECK : Casting pointer ???
//		data.commit();
//		cones.setParam("cones", data);
        ospSetParam(cones,"cones.center",OSP_VEC3F, coneCenterData.data());
        ospSetParam(cones,"cones.axis",OSP_VEC3F, coneAxisData.data());
        ospSetParam(cones,"cones.radius",OSP_FLOAT, coneRadiusData.data());
        ospCommit (cones);

		//data = ospray::cpp::Data(ncones, OSP_VEC4F, coneColorData.data());
		//data.commit();
		//cones.setParam("color", data);
		//cones.commit();

        //ospray::cpp::GeometricModel conesModel(cones);
        auto conesModel = ospNewGeometricModel(cones);
        //cones.setMaterial(*_ospMaterial);
        //cones.setParam("material",*_ospMaterial);
        ospSetParam(conesModel,"material", OSP_MATERIAL,_ospMaterial);
        ospSetParam(conesModel,"color", OSP_VEC4F,coneColorData.data());
        //ospSetObject(cones.handle(),"material",_ospMaterial->handle());
        //conesModel.commit();
        ospCommit (conesModel);
        geometricModels.push_back(conesModel);
		//_ospGroup->addGeometry(cones);
	}
}

/******************************************************************************
* Renders the text stored in the given buffer.
******************************************************************************/
void OSPRayRenderer::renderText(const DefaultTextPrimitive& textBuffer, const Point2& pos, int alignment)
{
	_textDrawCalls.push_back(std::make_tuple(textBuffer.text(), textBuffer.color(), textBuffer.font(), pos, alignment));
}

/******************************************************************************
* Renders the image stored in the given buffer.
******************************************************************************/
void OSPRayRenderer::renderImage(const DefaultImagePrimitive& imageBuffer, const Point2& pos, const Vector2& size)
{
	_imageDrawCalls.push_back(std::make_tuple(imageBuffer.image(), pos, size));
}

/******************************************************************************
* Renders the triangle mesh stored in the given buffer.
******************************************************************************/
void OSPRayRenderer::renderMesh(const DefaultMeshPrimitive& meshBuffer)
{
	const TriMesh& mesh = meshBuffer.mesh();

	// Allocate render vertex buffer.
	int renderVertexCount = mesh.faceCount() * 3;
	if(renderVertexCount == 0)
		return;

	std::vector<ospray::vec4f> colors(renderVertexCount);
	std::vector<ospray::vec3f> normals(renderVertexCount);
	//std::vector<Point_3<float>> positions(renderVertexCount);
	std::vector<ospray::vec3f> positions(renderVertexCount);
	std::vector<ospray::vec3i> indices(mesh.faceCount());

	// Repeat the following multiple times if instanced rendering is requested.
	size_t numInstances = meshBuffer.useInstancedRendering() ? meshBuffer.perInstanceTMs().size() : 1;
	for(size_t instanceIndex = 0; instanceIndex < numInstances; instanceIndex++) {

		AffineTransformationT<float> tm = (AffineTransformationT<float>)modelTM();
		if(meshBuffer.useInstancedRendering())
			tm = tm * (AffineTransformationT<float>)meshBuffer.perInstanceTMs()[instanceIndex];
		const Matrix_3<float> normalTM = tm.linear().inverse().transposed();
		quint32 allMask = 0;

		// Compute face normals.
		std::vector<Vector_3<float>> faceNormals(mesh.faceCount());
		auto faceNormal = faceNormals.begin();
		if(!mesh.hasNormals()) {
			for(auto face = mesh.faces().constBegin(); face != mesh.faces().constEnd(); ++face, ++faceNormal) {
				const Point3& p0 = mesh.vertex(face->vertex(0));
				Vector3 d1 = mesh.vertex(face->vertex(1)) - p0;
				Vector3 d2 = mesh.vertex(face->vertex(2)) - p0;
				*faceNormal = normalTM * (Vector_3<float>)d2.cross(d1);
				if(*faceNormal != Vector_3<float>::Zero()) {
					allMask |= face->smoothingGroups();
				}
			}
		}

		// Initialize render vertices.
		auto rv_pos = positions.begin();
		auto rv_normal = normals.begin();
		auto rv_color = colors.begin();
		auto rv_indices = indices.begin();
		faceNormal = faceNormals.begin();
		if(mesh.hasNormals()) {
			OVITO_ASSERT(mesh.normals().size() == normals.size());
			/*std::transform(mesh.normals().cbegin(), mesh.normals().cend(), rv_normal, [&](const Vector3& n) {
				return normalTM * Vector_3<float>(n); //TODO check ?
			});*/
		std::transform(mesh.normals().cbegin(), mesh.normals().cend(), rv_normal, [&](const Vector3& n) {
		    auto result = normalTM * Vector_3<float>(n);
		    ospray::vec3f vector (result.x(),result.y(),result.z());
				return vector; //TODO check ?
			});
		}
		ColorAT<float> defaultVertexColor = ColorAT<float>(meshBuffer.meshColor());
		if(meshBuffer.useInstancedRendering() && !meshBuffer.perInstanceColors().empty())
			defaultVertexColor = ColorAT<float>(meshBuffer.perInstanceColors()[instanceIndex]);
		int vindex = 0;
		for(auto face = mesh.faces().constBegin(); face != mesh.faces().constEnd(); ++face, ++faceNormal, ++rv_indices) {

			// Initialize render vertices for this face.
			for(size_t v = 0; v < 3; v++, ++rv_pos, ++rv_normal, ++rv_color) {
				(*rv_indices)[v] = vindex++;
				if(!mesh.hasNormals()) {
					if(face->smoothingGroups())
						*rv_normal = ospray::vec3f(0,0,0); //Vector_3<float>::Zero();
					else
						*rv_normal = ospray::vec3f (faceNormal->x(),faceNormal->y(),faceNormal->z());
				}
				//*rv_pos = tm * (Point_3<float>)mesh.vertex(face->vertex(v));
                auto rv_pos_as_point = tm * (Point_3<float>)mesh.vertex(face->vertex(v));
                *rv_pos =  ospray::vec3f(rv_pos_as_point.x(),rv_pos_as_point.y(),rv_pos_as_point.z());

				if(!meshBuffer.useInstancedRendering() || meshBuffer.perInstanceColors().empty()) {
					if(mesh.hasVertexColors()) {
                        //*rv_color = ColorAT<float>(mesh.vertexColor(face->vertex(v)));
                        auto color = mesh.vertexColor(face->vertex(v));
                        *rv_color = ospray::vec4f(color.r(),color.g(),color.b(),color.a());
                    }
					else if(mesh.hasFaceColors()) {
                        //*rv_color = ColorAT<float>(mesh.faceColor(face - mesh.faces().constBegin()));
                        auto color = mesh.faceColor(face - mesh.faces().constBegin());
                        *rv_color = ospray::vec4f(color.r(),color.g(),color.b(),color.a());
                    }
					else if(face->materialIndex() < meshBuffer.materialColors().size() && face->materialIndex() >= 0) {
                        auto color = meshBuffer.materialColors()[face->materialIndex()];
                        *rv_color = ospray::vec4f(color.r(),color.g(),color.b(),color.a());
                        //*rv_color = ColorAT<float>(meshBuffer.materialColors()[face->materialIndex()]);
                    }
					else
                        *rv_color = ospray::vec4f(defaultVertexColor.r(),defaultVertexColor.g(),defaultVertexColor.b(),defaultVertexColor.a());
                    //*rv_color = defaultVertexColor;
				}
				//else *rv_color = defaultVertexColor;
				else *rv_color = ospray::vec4f(defaultVertexColor.r(),defaultVertexColor.g(),defaultVertexColor.b(),defaultVertexColor.a());
			}
		}

		if(allMask) {
			std::vector<Vector_3<float>> groupVertexNormals(mesh.vertexCount());
			for(int group = 0; group < OVITO_MAX_NUM_SMOOTHING_GROUPS; group++) {
				quint32 groupMask = quint32(1) << group;
				if((allMask & groupMask) == 0) continue;

				// Reset work arrays.
				std::fill(groupVertexNormals.begin(), groupVertexNormals.end(), Vector_3<float>::Zero());

				// Compute vertex normals at original vertices for current smoothing group.
				faceNormal = faceNormals.begin();
				for(auto face = mesh.faces().constBegin(); face != mesh.faces().constEnd(); ++face, ++faceNormal) {
					// Skip faces which do not belong to the current smoothing group.
					if((face->smoothingGroups() & groupMask) == 0) continue;

					// Add face's normal to vertex normals.
					for(size_t fv = 0; fv < 3; fv++)
						groupVertexNormals[face->vertex(fv)] += *faceNormal;
				}

				// Transfer vertex normals from original vertices to render vertices.
				rv_normal = normals.begin();
				for(const auto& face : mesh.faces()) {
					if(face.smoothingGroups() & groupMask) {
						for(size_t fv = 0; fv < 3; fv++, ++rv_normal) {
                            auto normal = groupVertexNormals[face.vertex(fv)];
                            *rv_normal += ospray::vec3f (normal.x(),normal.y(),normal.z());
                        }
					}
					else rv_normal += 3;
				}
			}
		}

        auto triangles = ospNewGeometry("triangles");


        //ospray::cpp::Data data(chunkSize * 6 * 4, OSP_VEC3F, vertices.data() + (chunkOffset * 6 * 4));
        auto data = ospNewSharedData1D(positions.data(),OSP_VEC3F, positions.size());
        //ospray::cpp::Data data(chunkSize * 6 * 4, OSP_VEC3F, vertices.data());
        //ospray::cpp::Data data(vertices);
        //data.commit();
        ospCommit(data);
        //ospSetParam(triangles,"vertex",OSP_VEC3F, data);
        ospSetObject(triangles,"vertex", data);

        //data = ospray::cpp::Data(chunkSize * 6 * 4,OSP_VEC4F, colors.data() + (chunkOffset * 6 * 4));
        data = ospNewSharedData1D(colors.data() ,OSP_VEC4F, colors.size());
        ospCommit(data);
        ospSetObject(triangles,"vertex.color", data);

        //data = ospray::cpp::Data(chunkSize * 6 * 4, OSP_VEC3F, normals.data() + (chunkOffset * 6 * 4));
        data = ospNewSharedData1D(normals.data() ,OSP_VEC3F, normals.size());
        ospCommit(data);
        ospSetObject(triangles,"vertex.normal", data);

        //data = ospray::cpp::Data(chunkSize * 6 * 2, OSP_VEC3I, indices.data() + (chunkOffset * 6 * 3 * 2));
        data = ospNewSharedData1D(indices.data(),OSP_VEC3I, indices.size());
        ospCommit(data);
        //ospSetParam(triangles,"index", OSP_VEC3I, data);
        ospSetObject(triangles,"index",  data);

        ospCommit(triangles);

        OSPGeometricModel trianglesModel = ospNewGeometricModel(triangles);

        ospSetParam(trianglesModel,"material",OSP_MATERIAL, _ospMaterial);
        ospCommit(trianglesModel);
        geometricModels.push_back(trianglesModel);

		/*
		ospray::cpp::Geometry triangles("triangles");

		ospray::cpp::Data data(positions.size(), OSP_VEC3F, positions.data());
		data.commit();
		triangles.setParam("vertex", data);


        data = ospray::cpp::Data(colors.size(), OSP_VEC4F, colors.data());
		data.commit();
		triangles.setParam("vertex.color", data);

		data = ospray::cpp::Data(normals.size(), OSP_VEC3F, normals.data());
		data.commit();
		triangles.setParam("vertex.normal", data);

		data = ospray::cpp::Data(mesh.faceCount(), OSP_VEC3I, indices.data());
		data.commit();
		triangles.setParam("index", data);
		triangles.commit();

        ospray::cpp::GeometricModel trianglesModel(triangles);
		//triangles.setMaterial(*_ospMaterial);
        //ospSetObject(triangles.handle(),"material",_ospMaterial->handle());
        trianglesModel.setParam("material", *_ospMaterial);
        trianglesModel.commit();
        geometricModels.push_back(trianglesModel);
		//_ospGroup->addGeometry(triangles);*/
	}
}

}	// End of namespace
}	// End of namespace
