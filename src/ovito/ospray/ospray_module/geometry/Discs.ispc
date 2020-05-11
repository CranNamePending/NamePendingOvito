////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright 2018 Alexander Stukowski
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

// ospray
//#include "math/vec.ih"
//#include "math/box.ih"
#include <common/Ray.ih>
#include "geometry/Geometry.ih"
#include "common/Data.ih"
#include "math/vec.ih"
#include "math/box.ih"
#include "safe_gather.ih"
//#include "common/FilterIntersect.ih"
//#include "common/Intersect.ih"
//#include "common/World.ih"
//#include "common/Model.ih"
//#include "math/sampling.ih"
//#include "ospray/OSPDataType.h"

struct Discs {
  /*! inherit from "Geometry" class: */
  Geometry   super;

    /*! data array that contains the disc data (possibly but not
      necessarily including the color, which could be in color);
      most offset and stride values point into here.  */

    float global_radius;
    Data1D vertex;
    Data1D normal;
    Data1D radius;
    Data1D texcoord;

    /*uint8     *data;
    int32      stride;
    int        offset_center;
    int        offset_normal;
    int        offset_radius;
    int        offset_materialID;
    int        offset_colorID;

    float      radius;
    int        materialID;

    uint8     *color;
    int        color_stride;
    int        color_offset;
    int        color_format;

    vec2f     *texcoord;
    bool       huge_mesh;*/


};

/*
inline int32 Discs_getMaterialID(const uniform Discs *uniform self, int32 primID)
{
  int32 materialID = -1;
  if (self->offset_materialID >= 0) {
    const uniform int32 primsPerPage = (1024*1024*64);
    if (any(primID >= primsPerPage )) {
      const int primPageID  = primID / primsPerPage;
      const int localPrimID = primID % primsPerPage;
      foreach_unique (primPage in primPageID) {
        uniform uint8 *uniform pagePtr = self->data
          + (((int64)primPage)
             * primsPerPage
             * self->stride);
        uniform uint8 *varying discPtr = pagePtr + self->stride*localPrimID;
        materialID = *((uniform uint32 *varying)(discPtr+self->offset_materialID));
      }
    } else {
      uniform uint8 *varying discPtr = self->data + self->stride*primID;
      materialID = *((uniform uint32 *varying)(discPtr+self->offset_materialID));
    }
  } else {
    materialID = self->materialID;
  }
  return materialID;
}
*/
static void Discs_postIntersect(const Geometry *uniform geometry,
        varying DifferentialGeometry &dg,
        const varying Ray &ray,
        uniform int64 flags)
{
    Discs *uniform self = (Discs * uniform) geometry;

    dg.Ng = dg.Ns = ray.Ng;

//  uint8 * quadricPtr = self->data + self->stride*((int64)ray.primID);
//float radius = *((float *)(quadricPtr + self->offset_radius));
    float radius = self->global_radius;
    if (valid(self->radius))
        radius = get_float(self->radius, ray.primID);

// make epsilon large enough to not get lost when computing
// |CO| = |center-ray.org| ~ radius for 2ndary rays
    dg.epsilon = radius * ulpEpsilon;

    if (and(flags & DG_TEXCOORD, valid(self->texcoord))){
//dg.st = self->texcoord[ray.primID];
        dg.st = get_vec2f(self->texcoord,ray.primID);
    }
    else
    dg.st = make_vec2f(0.0f);

}

export void *uniform Discs_create(void *uniform cppEquivalent)
{
    uniform Discs *uniform self = uniform new uniform Discs;
    Geometry_Constructor(&self->super,cppEquivalent,Discs_postIntersect);
    Data1D_Constructor(&self->vertex);
    Data1D_Constructor(&self->normal);
    Data1D_Constructor(&self->radius);
    Data1D_Constructor(&self->texcoord);
    self->global_radius =0.01;
    return self;
}

unmasked void Discs_bounds(const RTCBoundsFunctionArguments *uniform args)
{
  uniform Discs *uniform self = (uniform Discs *uniform)args->geometryUserPtr;
  uniform int primID = args->primID;
  uniform float radius = valid(self->radius) ? get_float(self->radius, primID)
                                               : self->global_radius;
   uniform vec3f center = get_vec3f(self->vertex, primID);
  //uniform uint8 *uniform discPtr = self->data + self->stride*((uniform int64)primID);
  //uniform bool offr = self->offset_radius >= 0;
  /*uniform float radius =
    offr ? *((uniform float *uniform)(discPtr+self->offset_radius)) :
    self->radius;*/
  //uniform vec3f center = *((uniform vec3f*uniform)(discPtr+self->offset_center));

  box3fa *uniform out = (box3fa *uniform)args->bounds_o;
  *out = make_box3fa(center - radius, center + radius);
}

void Discs_intersect_kernel(const RTCIntersectFunctionNArguments *uniform args, const uniform bool isOcclusionTest)
{
  // make sure to set the mask
  if (!args->valid[programIndex]) return;

  uniform Discs *uniform self = (uniform Discs *uniform)args->geometryUserPtr;
  uniform unsigned int primID = args->primID;

  // this assumes that the args->rayhit is actually a pointer to a varying ray!
  varying Ray *uniform ray = (varying Ray *uniform)args->rayhit;

  //uniform uint8 *uniform discPtr = self->data + self->stride*((uniform int64)primID);
  //uniform float radius = self->radius;
    uniform float radius = valid(self->radius) ? get_float(self->radius, primID)
                                               : self->global_radius;
  //if(self->offset_radius >= 0) {
  //  radius = *((uniform float *uniform)(discPtr+self->offset_radius));
  //}
  //uniform vec3f center = *((uniform vec3f*uniform)(discPtr+self->offset_center));
  //uniform vec3f normal = *((uniform vec3f*uniform)(discPtr+self->offset_normal));
  uniform vec3f center = get_vec3f(self->vertex, primID);
  uniform vec3f normal = get_vec3f(self->normal, primID);

  const float d = -dot(center, normal);
  float t = -(d + dot(normal, ray->org));
  const float td = dot(normal, ray->dir);
  if(td == 0.f) return;

  t = t / td;
  if(t > ray->t0 && t < ray->t) {
    const vec3f hitpnt = ray->org + t * ray->dir - center;
    if(dot(hitpnt,hitpnt) < radius*radius) {
      if (isOcclusionTest) {
        ray->t = neg_inf;
      } else {
        ray->t = t;
        ray->primID = primID;
        //ray->geomID = self->super.geomID;
        ray->geomID = args->geomID;
        ray->instID = args->context->instID[0];
        ray->Ng = normal;
      }
    }
  }
}

unmasked void Discs_intersect(const struct RTCIntersectFunctionNArguments *uniform args)
{
  Discs_intersect_kernel(args,false);
}

unmasked void Discs_occluded(const struct RTCIntersectFunctionNArguments *uniform args)
{
  Discs_intersect_kernel(args,true);
}

/*int32 Discs_getMaterialID(const Geometry *uniform const _self, const int32 primID)
{
  const Discs *const uniform self = (const Discs *uniform)_self;
  return Discs_getMaterialID(self, primID);
}*/

export void Discs_finalize(void *uniform _self,
                              void *uniform _embreeGeometry,
                              const Data1D *uniform vertex,
                              const Data1D *uniform radius,
                              const Data1D *uniform texcoord,
                              const Data1D *uniform normal,
                              uniform float global_radius)
{
// first, typecast to our 'real' type. since ispc can't export real
// types to c we have to pass 'self' in as a void*, and typecast
// here.
    Discs *uniform self = (Discs * uniform) _self;


// set our internal data.
    self->vertex = *vertex;
    self->radius = *radius;
    self->texcoord = *texcoord;
    self->normal = *normal;
    self->super.numPrimitives = vertex->numItems;
    self->global_radius = global_radius;


    RTCGeometry geom = (RTCGeometry)_embreeGeometry;

// set 'us' as user data (this will be the first arg in intersect()
// and computebounds() callbacks
    rtcSetGeometryUserData(geom, self);
// set function pointers:
    rtcSetGeometryBoundsFunction(
        geom, (uniform RTCBoundsFunction)&Discs_bounds, self);
    rtcSetGeometryIntersectFunction(
        geom, (uniform RTCIntersectFunctionN)&Discs_intersect);
    rtcSetGeometryOccludedFunction(
        geom, (uniform RTCOccludedFunctionN)&Discs_occluded);
    rtcCommitGeometry(geom);

}

/*export void DiscsGeometry_set(void  *uniform _self
    , void *uniform _model
    , void *uniform data
    , void *uniform materialList
    , vec2f *uniform texcoord
    , void *uniform color
    , uniform int color_offset
    , uniform int color_stride
    , uniform int color_format
    , uniform int numDiscs
    , uniform int bytesPerDisc
    , uniform float radius
    , uniform int materialID
    , uniform int offset_center
    , uniform int offset_normal
    , uniform int offset_radius
    , uniform int offset_materialID
    , uniform int offset_colorID
    , uniform bool huge_mesh
    )
{
  uniform Discs *uniform self = (uniform Discs *uniform)_self;
  uniform Model *uniform model = (uniform Model *uniform)_model;

  RTCGeometry geom = rtcNewGeometry(ispc_embreeDevice(),RTC_GEOMETRY_TYPE_USER);
  uniform uint32 geomID = rtcAttachGeometry(model->embreeSceneHandle,geom);

  self->super.model = model;
  self->super.geomID = geomID;
  self->super.numPrimitives = numDiscs;
  self->super.materialList = (Material **)materialList;
  self->texcoord = texcoord;
  self->color = (uint8 *uniform)color;
  self->color_stride = color_stride;
  self->color_offset = color_offset;
  self->color_format = color_format;
  self->radius = radius;
  self->data = (uint8 *uniform)data;
  self->materialID = materialID;
  self->stride = bytesPerDisc;

  self->offset_center     = offset_center;
  self->offset_normal     = offset_normal;
  self->offset_radius     = offset_radius;
  self->offset_materialID = offset_materialID;
  self->offset_colorID    = offset_colorID;

  self->huge_mesh = huge_mesh;

  rtcSetGeometryUserData(geom, self);
  rtcSetGeometryUserPrimitiveCount(geom, numDiscs);
  rtcSetGeometryBoundsFunction(geom,(uniform RTCBoundsFunction)&Discs_bounds, self);
  rtcSetGeometryIntersectFunction(geom,(uniform RTCIntersectFunctionN)&Discs_intersect);
  rtcSetGeometryOccludedFunction(geom,(uniform RTCOccludedFunctionN)&Discs_occluded);
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);
}*/