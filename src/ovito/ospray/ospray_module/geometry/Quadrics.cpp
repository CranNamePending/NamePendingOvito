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

#include <ospray/SDK/api/ISPCDevice.h>
#include "Quadrics.h"
// 'export'ed functions from the ispc file:
#include "Quadrics_ispc.h"
// ospray core:
//#include <common/Data.h>

/*! _everything_ in the ospray core universe should _always_ be in the
  'ospray' namespace. */
namespace ospray {
  namespace ovito {

    /*! constructor - will create the 'ispc equivalent' */
    Quadrics::Quadrics()
    {
      /*! create the 'ispc equivalent': ie, the ispc-side class that
        implements all the ispc-side code for intersection,
        postintersect, etc. */
      this->ispcEquivalent = ispc::Quadrics_create(this);
        embreeGeometry = rtcNewGeometry(ispc_embreeDevice(), RTC_GEOMETRY_TYPE_USER);

      // note we do _not_ yet do anything else here - the actual input
      // data isn't available to use until 'commit()' gets called
    }

      std::string Quadrics::toString() const
      {
          return "ospray::Quadrices";
      }

      void Quadrics::commit()
      {
          this->radius = getParam<float>("radius",0.01);
          this->centerData = getParamDataT<math::vec3f>("quadrics.center");
          this->radiusData = getParamDataT<float>("quadrics.radius");
          this->texcoordData = getParamDataT<math::vec2f>("quadrics.texcoord");
          this->coeffData = getParamDataT<quadmatrix>("quadrics.coeff");


          //radius = getParam<float>("radius", 0.01f);
          /*radius = getParam<float>("radius", 0.01f);
          vertexData = getParamDataT<vec3f>("discs.position", true);
          radiusData = getParamDataT<float>("disc.radius");
          texcoordData = getParamDataT<vec2f>("disc.texcoord");

          ispc::DiscsGeometry_set(getIE(),
                                    embreeGeometry,
                                    ispc(vertexData),
                                    ispc(radiusData),
                                    ispc(texcoordData),
                                    radius);*/
          Quadrics_finalize(getIE(),embreeGeometry,ispc(centerData),ispc(radiusData),
                         ispc(texcoordData),ispc(coeffData),radius);

          postCreationInfo();
      }

      size_t Quadrics::numPrimitives() const
      {
          return centerData ? centerData->size() : 0;
      }



      /*! 'finalize' is what ospray calls when everything is set and
          done, and a actual user geometry has to be built */
/*    void Quadrics::finalize(Model *model)
    {
      materialID        = getParam1i("materialID",0);
      bytesPerQuadric   = getParam1i("bytes_per_quadric",14*sizeof(float));
      offset_center     = getParam1i("offset_center",0);
      offset_coeff      = getParam1i("offset_coeff",4*sizeof(float));
      offset_radius     = getParam1i("offset_radius",3*sizeof(float));
      offset_materialID = getParam1i("offset_materialID",-1);
      offset_colorID    = getParam1i("offset_colorID",-1);
      quadricData       = getParamData("quadrics");
      colorData         = getParamData("color");
      colorOffset       = getParam1i("color_offset",0);
      texcoordData      = getParamData("texcoord");

      if (colorData) {
        if (hasParam("color_format")) {
          colorFormat = static_cast<OSPDataType>(getParam1i("color_format", OSP_UNKNOWN));
        } else {
          colorFormat = colorData->type;
        }
        if (colorFormat != OSP_FLOAT4 && colorFormat != OSP_FLOAT3
            && colorFormat != OSP_FLOAT3A && colorFormat != OSP_UCHAR4)
        {
          throw std::runtime_error("#ospray:geometry/quadrics: invalid "
                                  "colorFormat specified! Must be one of: "
                                  "OSP_FLOAT4, OSP_FLOAT3, OSP_FLOAT3A or "
                                  "OSP_UCHAR4.");
        }
      } else {
        colorFormat = OSP_UNKNOWN;
      }
      colorStride = getParam1i("color_stride",
                              colorFormat == OSP_UNKNOWN ?
                              0 : sizeOf(colorFormat));

      if(quadricData.ptr == nullptr) {
        throw std::runtime_error("#ospray:geometry/quadrics: no 'quadrics' data specified");
      }

      // look at the data we were provided with ....
      numQuadrics = quadricData->numBytes / bytesPerQuadric;

      if (numQuadrics >= (1ULL << 30)) {
        throw std::runtime_error("#ospray::Quadrics: too many qaudrics in this "
                                "qaudrics geometry. Consider splitting this "
                                "geometry in multiple geometries with fewer "
                                "quadrics (you can still put all those "
                                "geometries into a single model, but you can't "
                                "put that many quadrics into a single geometry "
                                "without causing address overflows)");
      }

      // check whether we need 64-bit addressing
      bool huge_mesh = false;
      if (colorData && colorData->numBytes > INT32_MAX)
        huge_mesh = true;
      if (texcoordData && texcoordData->numBytes > INT32_MAX)
        huge_mesh = true;

      ispc::QuadricsGeometry_set(getIE(), model->getIE(), quadricData->data,
        materialList ? ispcMaterialPtrs.data() : nullptr,
        texcoordData ? (ispc::vec2f *)texcoordData->data : nullptr,
        colorData ? colorData->data : nullptr,
        colorOffset, colorStride, colorFormat,
        numQuadrics, bytesPerQuadric,
        materialID,
        offset_center, offset_coeff, offset_radius,
        offset_materialID, offset_colorID,
        huge_mesh);
    }*/


    /*! This macro 'registers' the Quadrics class under the ospray
        geometry type name of 'quadrics'.

        It is _this_ name that one can now (assuming the module has
        been loaded with ospLoadModule(), of course) create geometries
        with; i.e.,

        OSPGeometry geom = ospNewGeometry("quadrics") ;
    */
//    OSP_REGISTER_GEOMETRY(Quadrics,quadrics);

  } // ::ospray::ovito
} // ::ospray
