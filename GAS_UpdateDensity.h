//
// Created by LiYifan on 2024/1/12.
//

#ifndef HINAPE_JET_GAS_UPDATEDENSITY_H
#define HINAPE_JET_GAS_UPDATEDENSITY_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

class GAS_UpdateDensity : public GAS_SubSolver
{
public:
    GETSET_DATA_FUNCS_I("density_data_type", DensityDataType)
    GETSET_DATA_FUNCS_F("kernel_radius", kernelRadius)
protected:
    explicit GAS_UpdateDensity(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_UpdateDensity() override = default;
    bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;
private:
    static const SIM_DopDescription	*getDopDescription();
    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(GAS_UpdateDensity,
                        GAS_SubSolver,
                        "Update Density",
                        getDopDescription());
};

#endif //HINAPE_JET_GAS_UPDATEDENSITY_H
