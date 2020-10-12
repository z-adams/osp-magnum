#include "OSPApplication.h"
#include <Magnum/GL/Mesh.h>
#include "osp/Resource/blueprints.h"
#include "adera/Shaders/PlumeShader.h"

using namespace osp;

OSPApplication::OSPApplication()
{

}

void osp::OSPApplication::debug_add_package(Package&& p)
{
    m_packages.emplace(p.get_prefix(), std::move(p));
}

Package& osp::OSPApplication::debug_get_package(std::string prefix)
{
    return m_packages.at(prefix);
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
    Package& pkg = debug_get_package("lzdb");
    pkg.clear<BlueprintVehicle>();

    m_packages.clear();
}
