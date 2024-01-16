//
// Created by LiYifan on 2024/1/11.
//

#ifndef HINAPE_JET_GAS_INITIALIZE_H
#define HINAPE_JET_GAS_INITIALIZE_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

class GAS_Initialize : public GAS_SubSolver
{
public:
    GETSET_DATA_FUNCS_I("particle_data_type", particleDataType)
protected:
    explicit GAS_Initialize(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_Initialize() override = default;
    bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;

private:
    static const SIM_DopDescription	*getDopDescription();
    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(GAS_Initialize,
                        GAS_SubSolver,
                        "Initialize Fluid Particles",
                        getDopDescription());
};


#endif //HINAPE_JET_GAS_INITIALIZE_H
