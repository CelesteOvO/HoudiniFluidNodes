//
// Created by LiYifan on 2024/1/12.
//

#ifndef HINAPE_JET_GAS_ADDEXTERNALFORCE_H
#define HINAPE_JET_GAS_ADDEXTERNALFORCE_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

class GAS_AddExternalForce : public GAS_SubSolver
{
public:
    GETSET_DATA_FUNCS_S("external_force_name", ExternalForceName);
    GETSET_DATA_FUNCS_V3("external_force", ExternalForce);
protected:
    explicit GAS_AddExternalForce(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_AddExternalForce() override = default;
    bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;
private:
    static const SIM_DopDescription	*getDopDescription();
    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(GAS_AddExternalForce,
                        GAS_SubSolver,
                        "Add External Force",
                        getDopDescription());
};

#endif //HINAPE_JET_GAS_ADDEXTERNALFORCE_H
