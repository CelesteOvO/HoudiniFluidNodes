//
// Created by LiYifan on 2024/1/12.
//

#ifndef HINAPE_JET_GAS_UPDATEPOSITIONANDVELOCITY_H
#define HINAPE_JET_GAS_UPDATEPOSITIONANDVELOCITY_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

class GAS_UpdatePositionAndVelocity : public GAS_SubSolver
{
public:
    GETSET_DATA_FUNCS_I("update_data_type", updateDataType)
    GETSET_DATA_FUNCS_F("time_step", TimeStep)
    GETSET_DATA_FUNCS_V3("volume_min", VolumeMin);
    GETSET_DATA_FUNCS_V3("volume_max", VolumeMax);
protected:
    explicit GAS_UpdatePositionAndVelocity(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_UpdatePositionAndVelocity() override = default;
    bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;
private:
    static const SIM_DopDescription	*getDopDescription();
    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(GAS_UpdatePositionAndVelocity,
                        GAS_SubSolver,
                        "Update Position And Velocity",
                        getDopDescription());
};

#endif //HINAPE_JET_GAS_UPDATEPOSITIONANDVELOCITY_H
