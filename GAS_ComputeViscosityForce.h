//
// Created by LiYifan on 2024/1/15.
//

#ifndef HINAPE_JET_GAS_COMPUTEVISCOSITYFORCE_H
#define HINAPE_JET_GAS_COMPUTEVISCOSITYFORCE_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

class GAS_ComputeViscosityForce : public GAS_SubSolver
{
public:
    GETSET_DATA_FUNCS_F("bulk_viscosity", BulkViscosity);
    GETSET_DATA_FUNCS_F("shear_viscosity", ShearViscosity);
    GETSET_DATA_FUNCS_F("speed_of_sound", SpeedOfSound);
    GETSET_DATA_FUNCS_F("kernel_radius", kernelRadius)
protected:
    explicit GAS_ComputeViscosityForce(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_ComputeViscosityForce() override = default;
    bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;

    ////////test////////
    void precomputeKernelCoefficients() const;

    static fpreal	_halfH;
    static fpreal	_kernelValueCoeff;
    static fpreal	_kernelGradientCoeffA;
    static fpreal	_kernelGradientCoeffB;

    fpreal getKernelValue(fpreal dist) const;
    UT_Vector3 getKernelGradient(fpreal dist, const UT_Vector3 &xij) const;
    ////////test////////

private:
    static const SIM_DopDescription	*getDopDescription();
    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(GAS_ComputeViscosityForce,
                        GAS_SubSolver,
                        "Compute Viscosity Force",
                        getDopDescription());
};
#endif //HINAPE_JET_GAS_COMPUTEVISCOSITYFORCE_H
