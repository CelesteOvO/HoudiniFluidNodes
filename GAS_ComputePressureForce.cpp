//
// Created by LiYifan on 2024/1/15.
//

#include "GAS_ComputePressureForce.h"
#include "sph_kernel.h"

#include <UT/UT_Interrupt.h>
#include <UT/UT_NetMessage.h>
#include <UT/UT_StringStream.h>

#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_GeometryCopy.h>

///////////////////test/////////////////////
fpreal	GAS_ComputePressureForce::_halfH = 0.0;
fpreal	GAS_ComputePressureForce::_kernelValueCoeff = 0.0;
fpreal	GAS_ComputePressureForce::_kernelGradientCoeffA = 0.0;
fpreal	GAS_ComputePressureForce::_kernelGradientCoeffB = 0.0;
///////////////////test/////////////////////

const SIM_DopDescription *GAS_ComputePressureForce::getDopDescription() {
    static PRM_Name DataType[] =
            {
                    PRM_Name("0", "SPH"),
                    PRM_Name("1", "PCISPH"),
                    PRM_Name("2", "PBF"),
                    PRM_Name(0)
            };

    static PRM_ChoiceList pressureDataType((PRM_ChoiceListType) (PRM_CHOICELIST_SINGLE), &(DataType[0]));
    static PRM_Name pressureDataTypeName("pressure_data_type", "Pressure Data Type");

    static PRM_Name target_density_name("target_density", "Target Density");
    static PRM_Default target_density_default(1000.0);

    static PRM_Name kernel_radius_name("kernel_radius", "Kernel Radius");
    static PRM_Default kernel_radius_default(0.0348);

    static PRM_Name eos_exponent_name("eos_exponent", "Eos Exponent");
    static PRM_Default eos_exponent_default(7.0);

    static PRM_Name speed_of_sound_name("speed_of_sound", "Speed Of Sound");
    static PRM_Default speed_of_sound_default(100.0);

    static std::array<PRM_Template, 6> PRMS{
            PRM_Template(PRM_INT, 1, &pressureDataTypeName, 0, &pressureDataType),
            PRM_Template(PRM_FLT_J, 1, &target_density_name, &target_density_default),
            PRM_Template(PRM_FLT_J, 1, &kernel_radius_name, &kernel_radius_default),
            PRM_Template(PRM_FLT_J, 1, &eos_exponent_name, &eos_exponent_default),
            PRM_Template(PRM_FLT_J, 1, &speed_of_sound_name, &speed_of_sound_default),
            PRM_Template()
    };
    static SIM_DopDescription DESC(true,
                                   "compute_pressure_force",
                                   "Compute Pressure Force",
                                   "ComputePressureForce",
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_ComputePressureForce::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    SIM_GeometryCopy *geometry = SIM_DATA_GET(*obj, "Geometry", SIM_GeometryCopy);
    if (!geometry) {
        std::cout << "No Geometry Copy!" << std::endl;
        return false;
    }

    {
        UT_AutoInterrupt task("Compute Pressure");
        SIM_GeometryAutoWriteLock lock(geometry);
        GU_Detail &gdp = lock.getGdp();

        GA_RWHandleF density_handle(gdp.findPointAttribute("density"));
        GA_RWHandleF pressure_handle(gdp.findPointAttribute("pressure"));
        GA_RWHandleV3 pressure_force_handle(gdp.findPointAttribute("pF"));
        GA_RWHandleF mass_handle(gdp.findPointAttribute("mass"));
        GA_RWHandleV3 pos_handle(gdp.findPointAttribute("P"));
        GA_RWHandleIA neighbor_list_handle(gdp.findPointAttribute("nList"));

        GA_RWHandleF predict_density_handle(gdp.findPointAttribute("pDensity"));
        GA_RWHandleF density_error_handle(gdp.findPointAttribute("Derr"));

        float target_density = getTargetDensity();
        int pressure_data_type = getPressureDataType();
        float kernel_radius = getKernelRadius();
        long long num_particles = gdp.getNumPoints();

        SpikyKernel spiky(kernel_radius);
        precomputeKernelCoefficients();
        if(pressure_data_type == 0) // sph
        {
            for(long long i = 0; i < num_particles; ++i) {
                GA_Offset offset = gdp.pointOffset(i);
                float density = density_handle.get(offset);

                // compute pressure from eos
                float eos_exponent = getEosExponent();
                float speed_of_sound = getSpeedOfSound();
                float eos_scale = speed_of_sound * speed_of_sound * target_density;

                float pressure =
                        1.0 * eos_scale / eos_exponent * (std::pow((density / target_density), eos_exponent) - 1.0);
                if (pressure < 0.0) {
                    pressure *= 0.0;
                }
                pressure_handle.set(offset, pressure);
            }
            for(long long i = 0; i < num_particles; ++i) {
                GA_Offset offset = gdp.pointOffset(i);
                // compute pressure force
                UT_Int32Array neighbours;
                neighbor_list_handle.get(offset, neighbours);
                for (int neighbour: neighbours) {
                    GA_Offset neighbor_offset = gdp.pointOffset(neighbour);
                    UT_Vector3 neighbor_pos = pos_handle.get(neighbor_offset);
                    UT_Vector3 pos = pos_handle.get(offset);
                    UT_Vector3 r = pos - neighbor_pos;
                    float r_length = r.length();
                    if (r_length > 0.0) {
                        UT_Vector3 pressure_force = pressure_force_handle.get(offset);
                        pressure_force -= mass_handle.get(offset) * mass_handle.get(neighbor_offset) *
                                          (pressure_handle.get(offset) / (density_handle.get(offset) * density_handle.get(offset)) + pressure_handle.get(neighbor_offset) /
                                                                            (density_handle.get(neighbor_offset) *
                                                                             density_handle.get(neighbor_offset))) *
                                          getKernelGradient(r_length, r);
                        pressure_force_handle.set(offset, pressure_force);
                    }
                }
            }
        }else if(pressure_data_type == 1) // pcisph
        {


        }
    }
    return true;
}

void GAS_ComputePressureForce::precomputeKernelCoefficients() const {
    const fpreal PI = 3.14159265359;
    const fpreal h = getKernelRadius();

    _halfH = h/2.0;	// In Monaghan2005, h=half of smoothing radius

    // Precompute value coefficient (Identical for part A and B)
    _kernelValueCoeff = 1.0 / (4.0*PI*pow(_halfH,3));

    // Precompute gradient coefficients
    _kernelGradientCoeffA = 3.0 / (4.0*PI*pow(_halfH,4));
    _kernelGradientCoeffB = -3.0 / (4.0*PI*pow(_halfH,4));
}

namespace
{
    // This is much faster than calling pow(val, exponent)
    inline double pow2(double val) { return val*val; }
    inline double pow3(double val) { return val*val*val; }
    inline double pow7(double val) { return val*val*val*val*val*val*val; }
}

fpreal GAS_ComputePressureForce::getKernelValue(fpreal dist) const {
    fpreal q = dist/_halfH;
    if (q<1.0)
    {
        return _kernelValueCoeff * ( pow3(2.0-q)-4*pow3(1.0-q) );
    }
    else
    {
        return _kernelValueCoeff * pow3(2.0-q);
    }
}

UT_Vector3 GAS_ComputePressureForce::getKernelGradient(fpreal dist, const UT_Vector3 &xij) const {
    fpreal q = dist/_halfH;
    UT_Vector3 gradient = xij;
    if (q<= 0.0)
    {
        gradient = UT_Vector3(0,0,0);
    }
    else if (q<1.0)
    {
        gradient *= _kernelGradientCoeffA * (4.0 * pow2(1.0-q) - pow2(2.0-q)) / dist;
    }
    else
    {
        gradient *= (_kernelGradientCoeffB * pow2(2.0 - q)) / dist;
    }

    return gradient;
}




