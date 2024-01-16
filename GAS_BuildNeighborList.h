//
// Created by LiYifan on 2024/1/11.
//

#ifndef HINAPE_JET_GAS_BUILDNEIGHBORLIST_H
#define HINAPE_JET_GAS_BUILDNEIGHBORLIST_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

class GAS_BuildNeighborList : public GAS_SubSolver
{
public:
    /// TODO: choose the type of neighbor search
    GETSET_DATA_FUNCS_F("kernel_radius", kernelRadius)
    GETSET_DATA_FUNCS_I("neighbor_data_type", neighborDataType)
protected:
    explicit GAS_BuildNeighborList(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_BuildNeighborList() override = default;
    bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;
private:
    static const SIM_DopDescription	*getDopDescription();
    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(GAS_BuildNeighborList,
                        GAS_SubSolver,
                        "Build Neighbor List",
                        getDopDescription());
};

#endif //HINAPE_JET_GAS_BUILDNEIGHBORLIST_H
