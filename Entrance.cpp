#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include "GAS_Initialize.h"
#include "GAS_BuildNeighborList.h"
#include "GAS_InitializeMass.h"
#include "GAS_UpdateDensity.h"
#include "GAS_UpdatePositionAndVelocity.h"
#include "GAS_AddExternalForce.h"
#include "GAS_ClearForce.h"
#include "GAS_ComputePressureForce.h"
#include "GAS_ComputeViscosityForce.h"

void initializeSIM(void *)
{
    IMPLEMENT_DATAFACTORY(GAS_Initialize);
    IMPLEMENT_DATAFACTORY(GAS_BuildNeighborList);
    IMPLEMENT_DATAFACTORY(GAS_InitializeMass)
    IMPLEMENT_DATAFACTORY(GAS_UpdateDensity)
    IMPLEMENT_DATAFACTORY(GAS_UpdatePositionAndVelocity)
    IMPLEMENT_DATAFACTORY(GAS_AddExternalForce)
    IMPLEMENT_DATAFACTORY(GAS_ClearForce)
    IMPLEMENT_DATAFACTORY(GAS_ComputePressureForce)
    IMPLEMENT_DATAFACTORY(GAS_ComputeViscosityForce)
}
