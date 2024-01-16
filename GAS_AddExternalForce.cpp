//
// Created by LiYifan on 2024/1/12.
//

#include "GAS_AddExternalForce.h"
#include <UT/UT_Interrupt.h>
#include <UT/UT_NetMessage.h>
#include <UT/UT_StringStream.h>

#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_GeometryCopy.h>

const SIM_DopDescription *GAS_AddExternalForce::getDopDescription() {

    static PRM_Name external_force_name("external_force_name", "External Force Name");

    static PRM_Name external_force("external_force", "External Force");
    static std::array<PRM_Default, 3> external_force_default = {0, 0, 0};

    static std::array<PRM_Template, 3> PRMS{
            PRM_Template(PRM_STRING, 1, &external_force_name),
            PRM_Template(PRM_XYZ_J, 3, &external_force, external_force_default.data()),
            PRM_Template()
    };
    static SIM_DopDescription DESC(true,
                                   "add_external_force",
                                   "Add External Force",
                                   "AddExternalForce",
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_AddExternalForce::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    SIM_GeometryCopy *geometry = SIM_DATA_GET(*obj, "Geometry", SIM_GeometryCopy);
    if (!geometry) {
        std::cout << "No Geometry Copy!" << std::endl;
        return false;
    }

    {
        UT_AutoInterrupt task("Add External Force");
        SIM_GeometryAutoWriteLock lock(geometry);
        GU_Detail &gdp = lock.getGdp();

        GA_RWHandleV3 external_force_handle(gdp.findPointAttribute("eF"));

        UT_Vector3 add_force = getExternalForce();

        long long num_particles = gdp.getNumPoints();
        for(long long i = 0; i < num_particles; ++i)
        {
            GA_Offset offset = gdp.pointOffset(i);
            UT_Vector3 external_force = external_force_handle.get(offset);
            external_force += add_force;
            external_force_handle.set(offset, external_force);
        }
    }
    return true;
}
