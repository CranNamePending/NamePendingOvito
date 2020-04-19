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

#pragma once

// ospcomon: vec3f, box3f, etcpp - generic helper stuff
#include <ospcommon/vec.h>
#include <ospcommon/box.h>
// ospray: everything that's related to the ospray ray tracing core
#include <geometry/Geometry.h>
#include <common/Model.h>

/*! _everything_ in the ospray core universe should _always_ be in the
  'ospray' namespace. */
namespace ospray {
  namespace ovito {
    // import ospcommon component - vec3f etc
    using namespace ospcommon;

    /*! a geometry type that implements circular discs.
      This implements a new ospray geometry, and as such has
      to

      a) derive from ospray::Geometry
      b) implement a 'commit()' message that parses the
         parameters/data arrays that the app has specified as inputs
      c) create an actual ospray geometry instance with the
         proper intersect() and postIntersect() functions.

      Note that how this class is called does not particularly matter;
      all that matters is under which name it is registered in the cpp
      file (see comments on OSPRAY_REGISTER_GEOMETRY)
    */
    struct Discs : public ospray::Geometry
    {
      /*! constructor - will create the 'ispc equivalent' */
      Discs();

      /*! 'finalize' is what ospray calls when everything is set and
        done, and a actual user geometry has to be built */
      virtual void finalize(Model *model) override;

      /*! default radius, if no per-disc radius was specified. */
      float radius;

      size_t numDiscs;
      size_t bytesPerDisc; //!< num bytes per disc
      int32 materialID;
      int64 offset_center;
      int64 offset_radius;
      int64 offset_normal;
      int64 offset_materialID;
      int64 offset_colorID;

      /*! the input data array. the data array contains a list of
          discs, each of which consists of two vec3fs + optional radius. */
      Ref<Data> discData;

      Ref<Data> texcoordData;

      /*! data array from which we read the per-disc color data; if
        NULL we do not have per-disc data */
      Ref<Data> colorData;


      /*! The color format of the colorData array, one of:
          OSP_FLOAT3, OSP_FLOAT3A, OSP_FLOAT4 or OSP_UCHAR4 */
      OSPDataType colorFormat;

      /*! stride in colorData array for accessing i'th disc's
        color. color of disc i will be read as 3 floats from
        'colorOffset+i*colorStride */
      size_t    colorStride;

      /*! offset in colorData array for accessing i'th disc's
        color. color of disc i will be read as 3 floats from
        'colorOffset+i*colorStride */
      size_t    colorOffset;
    };

  } // ::ospray::ovito
} // ::ospray

