//
// Created by LiYifan on 2024/1/11.
//

#include "GAS_Initialize.h"

#include <UT/UT_Interrupt.h>
#include <UT/UT_NetMessage.h>
#include <UT/UT_StringStream.h>

#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_GeometryCopy.h>

const SIM_DopDescription *GAS_Initialize::getDopDescription() {
    static PRM_Name DataType[] =
            {
                    PRM_Name("0", "SPHData"),
                    PRM_Name("1", "PciSPHData"),
                    PRM_Name("2", "PBFData"),
                    PRM_Name(0)
            };

    static PRM_ChoiceList particleDataType((PRM_ChoiceListType) (PRM_CHOICELIST_SINGLE), &(DataType[0]));
    static PRM_Name particleDataTypeName("particle_data_type", "Particle Data Type");

    static std::array<PRM_Template, 2> PRMS{
            PRM_Template(PRM_INT, 1, &particleDataTypeName, 0, &particleDataType),
            PRM_Template()
    };
    static SIM_DopDescription DESC(true,
                                   "init_fluid_particles",
                                   "Init Fluid Particles",
                                   "InitFluidParticles",
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_Initialize::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    SIM_GeometryCopy *geometry = SIM_DATA_GET(*obj, "Geometry", SIM_GeometryCopy);
    if (!geometry)
    {
        std::cout << "No Geometry Copy!" << std::endl;
        return false;
    }

    {
        UT_AutoInterrupt task("Init Fluid Particles");
        SIM_GeometryAutoWriteLock lock(geometry);
        GU_Detail &gdp = lock.getGdp();

        GA_RWAttributeRef mass_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, gdp.getStdAttributeName(GEO_ATTRIBUTE_MASS), 1,
                                                       GA_Defaults(0));
        GA_RWAttributeRef vel_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, gdp.getStdAttributeName(GEO_ATTRIBUTE_VELOCITY),
                                                      3, GA_Defaults(0));

        GA_RWAttributeRef density_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "density", 1, GA_Defaults(0));
        GA_RWAttributeRef pressure_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "pressure", 1, GA_Defaults(0));

        GA_RWAttributeRef external_force_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "eF", 3, GA_Defaults(0));
        GA_RWAttributeRef pressure_force_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "pF", 3, GA_Defaults(0));
        GA_RWAttributeRef viscosity_force_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "vF", 3, GA_Defaults(0));

        GA_RWAttributeRef neighbor_num_ref = gdp.addIntTuple(GA_ATTRIB_POINT, "nNum", 1, GA_Defaults(0));
        GA_RWAttributeRef neighbor_list_ref = gdp.addIntArray(GA_ATTRIB_POINT,GA_SCOPE_PUBLIC,"nList",50);

        int particle_data_type = getparticleDataType();

        if(particle_data_type == 0)
        {

        }
        else if(particle_data_type == 1)
        {
            GA_RWAttributeRef predict_pos_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "predict_pos", 3, GA_Defaults(0));
            GA_RWAttributeRef predict_vel_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "predict_vel", 3, GA_Defaults(0));

            GA_RWAttributeRef predict_density_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "pDensity", 1, GA_Defaults(0));
            GA_RWAttributeRef density_error_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "Derr", 1, GA_Defaults(0));
        }
        else if(particle_data_type == 2)
        {
            GA_RWAttributeRef predict_pos_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "predict_pos", 3, GA_Defaults(0));
            GA_RWAttributeRef predict_vel_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "predict_vel", 3, GA_Defaults(0));

            GA_RWAttributeRef lambda_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "lambda", 1, GA_Defaults(0));
            GA_RWAttributeRef delta_p_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "delta_p", 3, GA_Defaults(0));
        }

        /*GA_RWAttributeRef external_accel_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "eA", 3, GA_Defaults(0));
        GA_RWAttributeRef pressure_accel_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, "pA", 3, GA_Defaults(0));*/
    }

    return true;
}


