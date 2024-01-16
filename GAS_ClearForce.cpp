//
// Created by LiYifan on 2024/1/15.
//

#include "GAS_ClearForce.h"

#include <UT/UT_Interrupt.h>
#include <UT/UT_NetMessage.h>
#include <UT/UT_StringStream.h>

#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_GeometryCopy.h>

const SIM_DopDescription *GAS_ClearForce::getDopDescription() {
    static std::array<PRM_Template, 1> PRMS{
            PRM_Template()
    };
    static SIM_DopDescription DESC(true,
                                   "clear_force",
                                   "Clear Force",
                                   "ClearForce",
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_ClearForce::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    SIM_GeometryCopy *geometry = SIM_DATA_GET(*obj, "Geometry", SIM_GeometryCopy);
    if (!geometry) {
        std::cout << "No Geometry Copy!" << std::endl;
        return false;
    }

    {
        UT_AutoInterrupt task("Clear Force");
        SIM_GeometryAutoWriteLock lock(geometry);
        GU_Detail &gdp = lock.getGdp();

        GA_RWHandleV3 external_force_handle(gdp.findPointAttribute("eF"));
        GA_RWHandleV3 pressure_force_handle(gdp.findPointAttribute("pF"));

        long long num_particles = gdp.getNumPoints();
        for(long long i = 0; i < num_particles; ++i)
        {
            GA_Offset offset = gdp.pointOffset(i);
            UT_Vector3 external_force =  UT_Vector3(0, 0, 0);
            external_force_handle.set(offset, external_force);

            UT_Vector3 pressure_force = UT_Vector3(0, 0, 0);
            pressure_force_handle.set(offset, pressure_force);
        }
    }
    return true;
}


