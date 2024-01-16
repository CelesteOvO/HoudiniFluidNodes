//
// Created by LiYifan on 2024/1/12.
//

#include "GAS_InitializeMass.h"
#include "sph_kernel.h"

#include <UT/UT_Interrupt.h>
#include <UT/UT_NetMessage.h>
#include <UT/UT_StringStream.h>

#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_GeometryCopy.h>

const SIM_DopDescription *GAS_InitializeMass::getDopDescription() {
    static PRM_Name DataType[] =
            {
                    PRM_Name("0", "By Density"),
                    PRM_Name("1", "Set Mass"),
                    PRM_Name(0)
            };

    static PRM_ChoiceList massDataType((PRM_ChoiceListType) (PRM_CHOICELIST_SINGLE), &(DataType[0]));
    static PRM_Name massDataTypeName("mass_data_type", "Mass Data Type");

    static PRM_Name set_mass_name("set_mass", "Set Mass");
    static PRM_Default set_mass_default(1.0);

    static PRM_Name kernel_radius("kernel_radius", "Kernel Radius");
    static PRM_Default kernel_radius_default(0.0493);

    static PRM_Name rest_density("rest_density", "Rest Density");
    static PRM_Default rest_density_default(1000.0);

    static std::array<PRM_Template, 5> PRMS{
            PRM_Template(PRM_INT, 1, &massDataTypeName, 0, &massDataType),
            PRM_Template(PRM_FLT_J, 1, &set_mass_name, &set_mass_default),
            PRM_Template(PRM_FLT_J, 1, &kernel_radius, &kernel_radius_default),
            PRM_Template(PRM_FLT_J, 1, &rest_density, &rest_density_default),
            PRM_Template()
    };
    static SIM_DopDescription DESC(true,
                                   "initialize_mass",
                                   "Initialize Mass",
                                   "InitializeMass",
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_InitializeMass::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    SIM_GeometryCopy *geometry = SIM_DATA_GET(*obj, "Geometry", SIM_GeometryCopy);
    if (!geometry)
    {
        std::cout << "No Geometry Copy!" << std::endl;
        return false;
    }

    {
        UT_AutoInterrupt task("Initialize Mass");
        SIM_GeometryAutoWriteLock lock(geometry);
        GU_Detail &gdp = lock.getGdp();

        GA_RWHandleF mass_handle(gdp.findPointAttribute("mass"));
        GA_ROHandleV3 pos_handle(gdp.findPointAttribute("P"));
        GA_RWHandleIA neighbor_list_handle(gdp.findPointAttribute("nList"));

        int temp_choose = getMassDataType();

        //////////////////////
        fpreal h = getkernelRadius(); // kernel radius
        fpreal target_density = getrestDensity();
        //////////////////////

        long long particle_size = gdp.getNumPoints();
        fpreal max_number_density = 0;
        StdKernel poly6(h);
        for(long long i = 0; i < particle_size; i++)
        {
            GA_Offset offset = gdp.pointOffset(i);

            UT_Int32Array neighbours;
            neighbor_list_handle.get(offset, neighbours);
            fpreal sum = poly6(0);
            for(int neighbour : neighbours) {
                GA_Offset neighbor_offset = gdp.pointOffset(neighbour);
                sum += poly6((pos_handle.get(offset) - pos_handle.get(neighbor_offset)).length());
            }
            max_number_density = std::max(max_number_density, sum);
        }

        if(temp_choose == 0)
        {
            if(max_number_density > 0)
            {
                fpreal mass = target_density / max_number_density;
                for(long long i = 0; i < particle_size; i++) {
                    GA_Offset offset = gdp.pointOffset(i);
                    if(mass_handle.get(offset) == 0)
                        mass_handle.set(offset, mass);
                }
            }
        }else if(temp_choose == 1)
        {
            for(long long i = 0; i < particle_size; i++) {
                GA_Offset offset = gdp.pointOffset(i);
                mass_handle.set(offset, getSetMass());
            }
        }
    }
    return true;
}

