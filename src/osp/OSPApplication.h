#pragma once

#include "Universe.h"
#include "Resource/Package.h"

namespace osp
{


class OSPApplication
{
public:

    // put more stuff into here eventually

    OSPApplication();

    /**
     * Add a resource package to the application
     *
     * The package should be populated externally, then passed via rvalue
     * reference so the contents can be moved into the application resources
     * @param p [in] The package to add
     */
    void debug_add_package(Package&& p);

    /**
     * Get a resource package by prefix name
     *
     * @param [in] The short prefix name of the package
     * @return The resource package
     */
    Package& debug_get_package(std::string prefix);

    size_t debug_num_packages() const { return m_packages.size(); }

    universe::Universe& get_universe() { return m_universe; }

    void shutdown();
private:
    std::map<std::string, Package> m_packages;
    universe::Universe m_universe;
};

}
