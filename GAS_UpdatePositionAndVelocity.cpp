//
// Created by LiYifan on 2024/1/12.
//

#include "GAS_UpdatePositionAndVelocity.h"

#include <UT/UT_Interrupt.h>
#include <UT/UT_NetMessage.h>
#include <UT/UT_StringStream.h>

#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_GeometryCopy.h>

const SIM_DopDescription *GAS_UpdatePositionAndVelocity::getDopDescription() {
    static PRM_Name DataType[] =
            {
                    PRM_Name("0", "SPHData"),
                    PRM_Name("1", "PBFData"),
                    PRM_Name(0)
            };

    static PRM_ChoiceList updateDataType((PRM_ChoiceListType) (PRM_CHOICELIST_SINGLE), &(DataType[0]));
    static PRM_Name updateDataTypeName("update_data_type", "Update Data Type");

    static PRM_Name time_step_name("time_step", "Time Step");
    static PRM_Default time_step_default(0.005);

    static PRM_Name volume_min("volume_min", "Volume Min");
    static std::array<PRM_Default, 3> volume_min_default = {-0.5, 0, -0.5};

    static PRM_Name volume_max("volume_max", "Volume Max");
    static std::array<PRM_Default, 3> volume_max_default = {0.5, 1.5, 0.5};

    static std::array<PRM_Template, 5> PRMS{
            PRM_Template(PRM_INT, 1, &updateDataTypeName, 0, &updateDataType),
            PRM_Template(PRM_FLT_J, 1, &time_step_name, &time_step_default),
            PRM_Template(PRM_XYZ_J, 3, &volume_min, volume_min_default.data()),
            PRM_Template(PRM_XYZ_J, 3, &volume_max, volume_max_default.data()),
            PRM_Template()
    };
    static SIM_DopDescription DESC(true,
                                   "update_position_and_velocity",
                                   "Update Position And Velocity",
                                   "UpdatePositionAndVelocity",
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool
GAS_UpdatePositionAndVelocity::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
SIM_GeometryCopy *geometry = SIM_DATA_GET(*obj, "Geometry", SIM_GeometryCopy);
    if (!geometry) {
        std::cout << "No Geometry Copy!" << std::endl;
        return false;
    }

    {
        UT_AutoInterrupt task("Update Position And Velocity");
        SIM_GeometryAutoWriteLock lock(geometry);
        GU_Detail &gdp = lock.getGdp();

        /// Update Position And Velocity
        GA_RWHandleV3 pos_handle(gdp.findPointAttribute("P"));
        GA_RWHandleV3 vel_handle(gdp.findPointAttribute("v"));
        GA_RWHandleV3 predict_pos_handle(gdp.findPointAttribute("predict_pos"));
        GA_RWHandleF mass_handle(gdp.findPointAttribute("mass"));

        GA_RWHandleV3 external_force_handle(gdp.findPointAttribute("eF"));
        GA_RWHandleV3 pressure_force_handle(gdp.findPointAttribute("pF"));
        GA_RWHandleV3 viscosity_force_handle(gdp.findPointAttribute("vF"));


        long long num_particles = gdp.getNumPoints();

        float dt = getTimeStep();
        const UT_Vector3 _volumeMin = getVolumeMin();
        const UT_Vector3 _volumeMax = getVolumeMax();
        int temp_choose = getupdateDataType();

        for(long long i = 0; i < num_particles; i++)
        {
            GA_Offset offset = gdp.pointOffset(i);
            UT_Vector3 vel;
            UT_Vector3 pos;
            if(temp_choose == 1) /// pbf
            {
                vel = (predict_pos_handle.get(offset) - pos_handle.get(offset)) / dt;
                pos = predict_pos_handle.get(offset);
            }
            else if(temp_choose == 0)
            {
                vel = vel_handle.get(offset);
                pos = pos_handle.get(offset);
                UT_Vector3 force = external_force_handle.get(offset) + pressure_force_handle.get(offset) + viscosity_force_handle.get(offset);
                vel += force * dt / mass_handle.get(offset);
                pos += vel * dt;
            }
            // Apply boundary conditions
            if (pos.x() < _volumeMin.x())
            {
                pos.x() = _volumeMin.x();
                vel.x() = 0.0;
            }
            else if (pos.x() > _volumeMax.x())
            {
                pos.x() = _volumeMax.x();
                vel.x() = 0.0;
            }
            if (pos.y() < _volumeMin.y())
            {
                pos.y() = _volumeMin.y();
                vel.y() = 0.0;
            }
            else if (pos.y() > _volumeMax.y())
            {
                pos.y() = _volumeMax.y();
                vel.y() = 0.0;
            }
            if (pos.z() < _volumeMin.z())
            {
                pos.z() = _volumeMin.z();
                vel.z() = 0.0;
            }
            else if (pos.z() > _volumeMax.z())
            {
                pos.z() = _volumeMax.z();
                vel.z() = 0.0;
            }

            vel_handle.set(offset, vel);
            pos_handle.set(offset, pos);
        }
    }
    return true;
}


