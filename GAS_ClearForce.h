//
// Created by LiYifan on 2024/1/15.
//

#ifndef HINAPE_JET_GAS_CLEARFORCE_H
#define HINAPE_JET_GAS_CLEARFORCE_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

class GAS_ClearForce : public GAS_SubSolver
{
protected:
    explicit GAS_ClearForce(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_ClearForce() override = default;
    bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;
private:
    static const SIM_DopDescription	*getDopDescription();
    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(GAS_ClearForce,
                        GAS_SubSolver,
                        "Clear Force",
                        getDopDescription());
};

#endif //HINAPE_JET_GAS_CLEARFORCE_H
