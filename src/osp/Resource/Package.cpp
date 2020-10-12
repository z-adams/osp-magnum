#include "Package.h"



namespace osp
{

Package::Package(std::string const& prefix, std::string const& packageName) :
    // not sure about this but it compiles
    m_groups(),
    m_prefix(prefix),
    m_packageName(packageName)
{
}

path_t decompose_path(std::string const& path)
{
    size_t pos = path.find(':');
    return {
        path.substr(0, pos),
        path.substr(pos + 1, path.length())
    };
}

}
