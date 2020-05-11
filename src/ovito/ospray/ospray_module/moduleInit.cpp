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

#include <geometry/Quadrics.h>
#include <geometry/Cones.h>
#include <geometry/Cylinders.h>
#include "geometry/Discs.h"
#include "ospray/version.h"


/*! _everything_ in the ospray core universe should _always_ be in the 'ospray' namespace. */
namespace ospray {
  namespace ovito {

    /*! the actual module initialization function. This function gets
        called exactly once, when the module gets first loaded through
        'ospLoadModule'. Notes:
        a) this function does _not_ get called if the application directly
        links to libospray_module_<modulename> (which it
        shouldn't!). Modules should _always_ be loaded through
        ospLoadModule.
        b) it is _not_ valid for the module to do ospray _api_ calls
        inside such an intialization function. I.e., you can _not_ do a
        ospLoadModule("anotherModule") from within this function (but
        you could, of course, have this module dynamically link to the
        other one, and call its init function)
        c) in order for ospray to properly resolve that symbol, it
        _has_ to have extern C linkage, and it _has_ to correspond to
        name of the module and shared library containing this module
        (see comments regarding library name in CMakeLists.txt)
    */
    extern "C" OSPError OSPRAY_DLLEXPORT ospray_module_init_ovito(
            int16_t versionMajor, int16_t versionMinor, int16_t /*versionPatch*/) {


        auto status = moduleVersionCheck(versionMajor, versionMinor);

        if (status == OSP_NO_ERROR) {
            /*! maybe one of the most important parts of this example: this
                function 'registers' the BilinearPatches class under the ospray
                geometry type name of 'bilinear_patches'.

                It is _this_ name that one can now (assuming the module has
                been loaded with ospLoadModule(), of course) create geometries
                with; i.e.,

                OSPGeometry geom = ospNewGeometry("bilinear_patches") ;
            */
            Geometry::registerType<Quadrics>("quadrics");
            Geometry::registerType<Discs>("discs");
            Geometry::registerType<Cones>("cones");
            Geometry::registerType<Cylinders>("cylinders");
        }

        return status;
    }
  } // ::ospray::ovito
} // ::ospray
