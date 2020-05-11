// ======================================================================== //
// Copyright 2009-2019 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

// ospray
#include "Cylinders.h"
//#include "common/Data.h"
//#include "common/World.h"
// ispc-generated files
#include "Cylinders_ispc.h"
#include "ospcommon/utility/ParameterizedObject.h"


namespace ospray {
    namespace ovito {

    Cylinders::Cylinders(){
        this->ispcEquivalent = ispc::Cylinders_create(this);
        embreeGeometry = rtcNewGeometry(ispc_embreeDevice(), RTC_GEOMETRY_TYPE_USER);
    }

  std::string Cylinders::toString() const
  {
    return "ospray::Cylinders";
  }

  void Cylinders::commit()
  {

      this -> radius = getParam<float>("radius", 0.01f);
     this -> vertex0Data = getParamDataT<vec3f>("cylinder.position0", true);
    this -> vertex1Data = getParamDataT<vec3f>("cylinder.position1", true);
    if (vertex0Data->size() != vertex1Data->size())
      throw std::runtime_error(toString()
          + ": arrays 'cylinder.position0' and 'cylinder.position1' need to be of same size.");
    this -> radiusData = getParamDataT<float>("cylinder.radius");
    this -> texcoord0Data = getParamDataT<vec2f>("cylinder.texcoord0");
    this -> texcoord1Data = getParamDataT<vec2f>("cylinder.texcoord1");

    Cylinders_finalize(getIE(),embreeGeometry,ispc(vertex0Data),ispc(vertex1Data),ispc(radiusData),
                       ispc(texcoord0Data),ispc(texcoord1Data),radius);
    postCreationInfo();
  }

  size_t Cylinders::numPrimitives() const
  {
    return vertex0Data ? vertex0Data->size() : 0;
  }


  //OSP_REGISTER_GEOMETRY(Cylinders, cylinders);
    } // ::ospray::ovito

}  // namespace ospray
