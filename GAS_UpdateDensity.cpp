//
// Created by LiYifan on 2024/1/12.
//

#include "GAS_UpdateDensity.h"
#include "sph_kernel.h"

#include <UT/UT_Interrupt.h>
#include <UT/UT_NetMessage.h>
#include <UT/UT_StringStream.h>

#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_GeometryCopy.h>

const SIM_DopDescription *GAS_UpdateDensity::getDopDescription() {
    static PRM_Name DataType[] =
            {
                    PRM_Name("0", "By Position"),
                    PRM_Name("1", "By Predict Position"),
                    PRM_Name(0)
            };

    static PRM_ChoiceList densityDataType((PRM_ChoiceListType) (PRM_CHOICELIST_SINGLE), &(DataType[0]));
    static PRM_Name densityDataTypeName("density_data_type", "Density Data Type");

    static PRM_Name kernel_radius("kernel_radius", "Kernel Radius");
    static PRM_Default kernel_radius_default(0.0493);

    static std::array<PRM_Template, 3> PRMS{
            PRM_Template(PRM_INT, 1, &densityDataTypeName, 0, &densityDataType),
            PRM_Template(PRM_FLT_J, 1, &kernel_radius, &kernel_radius_default),
            PRM_Template()
    };
    static SIM_DopDescription DESC(true,
                                   "update_density",
                                   "Update Density",
                                   "UpdateDensity",
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_UpdateDensity::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    SIM_GeometryCopy *geometry = SIM_DATA_GET(*obj, "Geometry", SIM_GeometryCopy);
    if (!geometry)
    {
        std::cout << "No Geometry Copy!" << std::endl;
        return false;
    }

    {
        UT_AutoInterrupt task("Update Density");
        SIM_GeometryAutoWriteLock lock(geometry);
        GU_Detail &gdp = lock.getGdp();

        /// Update Density
        GA_RWHandleF density_handle(gdp.findPointAttribute("density"));
        GA_RWHandleIA neighbor_list_handle(gdp.findPointAttribute("nList"));
        GA_RWHandleF mass_handle(gdp.findPointAttribute("mass"));

        int temp_choose = getDensityDataType();

        UT_String temp_choose_str;
        if(temp_choose == 0)
            temp_choose_str = "P";
        else if(temp_choose == 1)
            temp_choose_str = "predict_pos";

        GA_ROHandleV3 pos_handle(gdp.findPointAttribute(temp_choose_str));

        if (!pos_handle.isValid())
        {
            std::cout << "No Position Attribute!" << std::endl;
            return false;
        }

        fpreal h = getkernelRadius(); // kernel radius
        long long particle_size = gdp.getNumPoints();

        StdKernel poly6(h);
        for(long long i = 0; i < particle_size; i++)
        {
            GA_Offset offset = gdp.pointOffset(i);

            fpreal particle_density = mass_handle.get(offset) * poly6(0.0);

            UT_Int32Array neighbours;
            neighbor_list_handle.get(offset, neighbours);
            for(int neighbour : neighbours)
            {
                GA_Offset neighbor_offset = gdp.pointOffset(neighbour);
                UT_Vector3 r = pos_handle.get(offset) - pos_handle.get(neighbor_offset);
                particle_density += mass_handle.get(neighbor_offset) * poly6(r.length());
            }
            density_handle.set(offset, particle_density);
        }
    }
    return true;
}


