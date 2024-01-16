//
// Created by LiYifan on 2024/1/15.
//

#include "GAS_ComputeViscosityForce.h"

#include <UT/UT_Interrupt.h>
#include <UT/UT_NetMessage.h>
#include <UT/UT_StringStream.h>

#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_GeometryCopy.h>

///////////////////test/////////////////////
fpreal	GAS_ComputeViscosityForce::_halfH = 0.0;
fpreal	GAS_ComputeViscosityForce::_kernelValueCoeff = 0.0;
fpreal	GAS_ComputeViscosityForce::_kernelGradientCoeffA = 0.0;
fpreal	GAS_ComputeViscosityForce::_kernelGradientCoeffB = 0.0;
///////////////////test/////////////////////

const SIM_DopDescription *GAS_ComputeViscosityForce::getDopDescription() {
    static PRM_Name speed_of_sound_name("speed_of_sound", "Speed Of Sound");
    static PRM_Default speed_of_sound_default(100.0);

    static PRM_Name kernelRadiusName("kernel_radius", "Kernel Radius");
    static PRM_Default kernelRadiusDefault(0.0348);

    static PRM_Name bulk_viscosity("bulk_viscosity", "Bulk Viscosity");
    static PRM_Default bulk_viscosity_default(0.0);

    static PRM_Name shear_viscosity("shear_viscosity", "Shear Viscosity");
    static PRM_Default shear_viscosity_default(0.0);

    static std::array<PRM_Template, 5> PRMS{
            PRM_Template(PRM_FLT_J, 1, &speed_of_sound_name, &speed_of_sound_default),
            PRM_Template(PRM_FLT_J, 1, &kernelRadiusName, &kernelRadiusDefault),
            PRM_Template(PRM_FLT_J, 1, &bulk_viscosity, &bulk_viscosity_default),
            PRM_Template(PRM_FLT_J, 1, &shear_viscosity, &shear_viscosity_default),
            PRM_Template()
    };
    static SIM_DopDescription DESC(true,
                                   "compute_viscosity_force",
                                   "Compute Viscosity Force",
                                   "ComputeViscosityForce",
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool
GAS_ComputeViscosityForce::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    SIM_GeometryCopy *geometry = SIM_DATA_GET(*obj, "Geometry", SIM_GeometryCopy);
    if (!geometry) {
        std::cout << "No Geometry Copy!" << std::endl;
        return false;
    }

    {
        UT_AutoInterrupt task("Computing Viscosity Force");
        SIM_GeometryAutoWriteLock lock(geometry);
        GU_Detail &gdp = lock.getGdp();

        GA_RWHandleF density_handle(gdp.findPointAttribute("density"));
        GA_RWHandleF mass_handle(gdp.findPointAttribute("mass"));
        GA_RWHandleV3 vel_handle(gdp.findPointAttribute("v"));
        GA_RWHandleV3 viscosity_force_handle(gdp.findPointAttribute("vF"));
        GA_RWHandleIA neighbor_list_handle(gdp.findPointAttribute("nList"));

        fpreal kernelRadius = getkernelRadius();
        fpreal speedOfSound = getSpeedOfSound();
        const double alpha = getBulkViscosity();	// Bulk viscosity
        const double beta = getShearViscosity();	// Shear viscosity

        precomputeKernelCoefficients();

        long particle_size = gdp.getNumPoints();
        for (long p = 0; p < particle_size; ++p)
        {
            GA_Offset offset = gdp.pointOffset(p);
            UT_Int32Array neighbours;
            neighbor_list_handle.get(offset, neighbours);
            for (int neighbour: neighbours) {
                GA_Offset neighborOffset = gdp.pointOffset(neighbour);
                UT_Vector3 vij = vel_handle.get(offset) - vel_handle.get(neighborOffset);
                UT_Vector3 xij = gdp.getPos3(offset) - gdp.getPos3(neighborOffset);
                fpreal dij = xij.length();
                double vijxij = vij.dot(xij);
                double uij = kernelRadius * vijxij / (dij*dij + 0.01 * kernelRadius * kernelRadius);
                if (uij < 0)
                {
                    // Compute contribution
                    double avgDensity = 0.5 * (density_handle.get(offset) + density_handle.get(neighborOffset));
                    double IIij = (alpha * uij * speedOfSound + beta*uij*uij) / avgDensity;
                    UT_Vector3 contribution = getKernelGradient(dij, xij);
                    contribution *= IIij;
                    contribution *= mass_handle.get(offset);

                    // Add contribution
                    UT_Vector3 viscosity_force = viscosity_force_handle.get(offset);
                    viscosity_force += contribution * mass_handle.get(neighborOffset);
                    viscosity_force_handle.set(offset, viscosity_force);
                }
            }
        }

    }
}

void GAS_ComputeViscosityForce::precomputeKernelCoefficients() const {
    const fpreal PI = 3.14159265359;
    const fpreal h = getkernelRadius();

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

fpreal GAS_ComputeViscosityForce::getKernelValue(fpreal dist) const {
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

UT_Vector3 GAS_ComputeViscosityForce::getKernelGradient(fpreal dist, const UT_Vector3 &xij) const {
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


