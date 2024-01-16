//
// Created by LiYifan on 2024/1/12.
//

#ifndef HINAPE_JET_GAS_INITIALIZEMASS_H
#define HINAPE_JET_GAS_INITIALIZEMASS_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

class GAS_InitializeMass : public GAS_SubSolver
{
public:
    GETSET_DATA_FUNCS_I("mass_data_type", MassDataType)
    GETSET_DATA_FUNCS_F("set_mass", SetMass)
    GETSET_DATA_FUNCS_F("kernel_radius", kernelRadius)
    GETSET_DATA_FUNCS_F("rest_density", restDensity)
protected:
    explicit GAS_InitializeMass(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_InitializeMass() override = default;
    bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;
private:
    static const SIM_DopDescription	*getDopDescription();
    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(GAS_InitializeMass,
                        GAS_SubSolver,
                        "Initialize Mass",
                        getDopDescription());
};
#endif //HINAPE_JET_GAS_INITIALIZEMASS_H
