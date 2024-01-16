//
// Created by LiYifan on 2024/1/15.
//

#ifndef HINAPE_JET_GAS_COMPUTEPRESSUREFORCE_H
#define HINAPE_JET_GAS_COMPUTEPRESSUREFORCE_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

class GAS_ComputePressureForce : public GAS_SubSolver
{
public:
    GETSET_DATA_FUNCS_I("pressure_data_type", PressureDataType)
    GETSET_DATA_FUNCS_F("target_density", TargetDensity)
    GETSET_DATA_FUNCS_F("kernel_radius", KernelRadius)
    GETSET_DATA_FUNCS_F("eos_exponent", EosExponent)
    GETSET_DATA_FUNCS_F("speed_of_sound", SpeedOfSound)
protected:
    explicit GAS_ComputePressureForce(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_ComputePressureForce() override = default;
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
    DECLARE_DATAFACTORY(GAS_ComputePressureForce,
                        GAS_SubSolver,
                        "Compute Pressure",
                        getDopDescription());
};
#endif //HINAPE_JET_GAS_COMPUTEPRESSUREFORCE_H
