#include "OSPApplication.h"
#include <Magnum/GL/Mesh.h>
#include "osp/Resource/blueprints.h"
#include "adera/Shaders/PlumeShader.h"

using namespace osp;

OSPApplication::OSPApplication()
{

}

void osp::OSPApplication::shutdown()
{
    m_universe.clear();

    /* TODO HACK:
       BlueprintVehicle resources themselves store DependRes<PrototypePart>,
       whose group is destructed before BlueprintVehicle. In order to prevent
       dereferencing of invalid pointers, the BlueprintVehicle group must be
       manually cleared before the packages are destroyed.
    */
    Package& pkg = debug_get_packges()[0];
    pkg.clear<BlueprintVehicle>();

    m_packages.clear();
}
