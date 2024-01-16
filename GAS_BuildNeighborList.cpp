//
// Created by LiYifan on 2024/1/11.
//

#include "GAS_BuildNeighborList.h"
#include "point_neighbor_search.h"

#include <UT/UT_Interrupt.h>
#include <UT/UT_NetMessage.h>
#include <UT/UT_StringStream.h>

#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_GeometryCopy.h>

const SIM_DopDescription *GAS_BuildNeighborList::getDopDescription() {
    static PRM_Name DataType[] =
            {
                    PRM_Name("0", "By Position"),
                    PRM_Name("1", "By Predict Position"),
                    PRM_Name(0)
            };

    static PRM_ChoiceList neighborDataType((PRM_ChoiceListType) (PRM_CHOICELIST_SINGLE), &(DataType[0]));
    static PRM_Name neighborDataTypeName("neighbor_data_type", "Neighbor Data Type");

    static PRM_Name kernel_radius("kernel_radius", "Kernel Radius");
    static PRM_Default kernel_radius_default(0.0493);

    static std::array<PRM_Template, 3> PRMS{
            PRM_Template(PRM_INT, 1, &neighborDataTypeName, 0, &neighborDataType),
            PRM_Template(PRM_FLT_J, 1, &kernel_radius, &kernel_radius_default),
            PRM_Template()
    };
    static SIM_DopDescription DESC(true,
                                   "build_neighbor_list",
                                   "Build Neighbor List",
                                   "BuildNeighborList",
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_BuildNeighborList::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    SIM_GeometryCopy *geometry = SIM_DATA_GET(*obj, "Geometry", SIM_GeometryCopy);
    if (!geometry)
    {
        std::cout << "No Geometry Copy!" << std::endl;
        return false;
    }

    {
        UT_AutoInterrupt task("Build Neighbor List");
        SIM_GeometryAutoWriteLock lock(geometry);
        GU_Detail &gdp = lock.getGdp();

        /// Build Neighbor List
        GA_RWHandleI neighbor_num_handle(gdp.findPointAttribute("nNum"));
        GA_RWHandleIA neighbor_list_handle(gdp.findPointAttribute("nList"));

        int neighbor_data_type = getneighborDataType();
        UT_String temp_choose_str;
        if(neighbor_data_type == 0)
            temp_choose_str = "P";
        else if(neighbor_data_type == 1)
            temp_choose_str = "predict_pos";

        GA_ROHandleV3 pos_handle(gdp.findPointAttribute(temp_choose_str));

        if (!pos_handle.isValid())
        {
            std::cout << "No Position Attribute!" << std::endl;
            return false;
        }
        ////////////////////////////////////////////////////////
        fpreal h = getkernelRadius(); // kernel radius

        ////////////////////////////////////////////////////////

        long long particle_size = gdp.getNumPoints();

        std::vector<UT_Vector3> position;
        position.reserve(particle_size);
        for (GA_Iterator it(gdp.getPointRange()); !it.atEnd(); ++it)
        {
            GA_Offset ptOff = *it;
            position.emplace_back(pos_handle.get(ptOff));
        }

        PointHashGridSearch3 searcher(h);
        searcher.build(position);

        std::vector<std::vector<long long>> neighbor_list;
        neighbor_list.resize(particle_size);
        auto &nl = neighbor_list;
        for (long long i = 0; i < particle_size; ++i)
        {
            GA_Offset offset = gdp.pointOffset(i);
            auto origin = position[offset];
            nl[offset].clear();
            searcher.for_each_nearby_point(origin, [&](size_t j, const UT_Vector3 &pos)
            {
                if (offset != j)
                {
                    nl[offset].emplace_back(j);
                }
            });
        }

        for (long long i = 0; i < particle_size; ++i)
        {
            GA_Offset offset = gdp.pointOffset(i);
            neighbor_num_handle.set(offset, neighbor_list[offset].size());

            UT_Int32Array nArray;
            nArray.setSize(neighbor_list[offset].size());
            for (long long j = 0; j < neighbor_list[offset].size(); ++j)
            {
                nArray[j] = neighbor_list[offset][j];
            }
            neighbor_list_handle.set(offset, nArray);

            //// 无语了，测完发现写进去了只是不在表格里显示出来，不知道为什么
            /*UT_Int32Array nArray1;
            neighbor_list_handle.get(offset, nArray1);
            if(nArray1.size() == 0)
                std::cout << "nArray size is 0" << std::endl;
            else{
                if(offset == 0)
                {
                    std::cout << "nArray size is " << nArray1.size() << std::endl;
                    for (int j = 0; j < nArray1.size(); ++j)
                    {
                        std::cout << nArray1[j] << " ";
                    }
                    std::cout << std::endl;
                }
            }*/
        }
    }
    return true;
}



