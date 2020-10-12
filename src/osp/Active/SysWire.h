#pragma once

#include "activetypes.h"
#include "../types.h"
#include "../Resource/blueprints.h"

#include <Corrade/Containers/LinkedList.h>

#include <variant>
#include <string>
#include <vector>

namespace osp::active
{

using Corrade::Containers::LinkedList;
using Corrade::Containers::LinkedListItem;

class WireInput;
class WireOutput;

namespace wiretype
{

    /**
     * A rotation in global space
     */
    struct Attitude
    {
        Quaternion m_global;
    };

    /**
     * A change in rotation. Separate pitch, yaw and roll
     */
    struct AttitudeControl
    {
        //  each (-1.0 .. 1.0)
        // pitch, yaw, roll
        Vector3 m_attitude;

        //Quaternion m_precise;
        //Quaternion m_rot;
        //Vector3 m_yawpitchroll
    };

    /**
     * A change in rotation in axis angle
     *
     */
    //struct AttitudeControlPrecise
    //{
    //    //Quaternion m_precise;
    //    Vector3 m_axis;
    //};

    /**
     * For something like throttle
     */
    struct Percent
    {
        float m_value;
    };

    enum class DeployOp {NONE, ON, OFF, TOGGLE};

    /**
     * Used to turn things on and off or ignite stuff
     */
    struct Deploy
    {
        //int m_stage;
        //bool m_on, m_off, m_toggle;
        DeployOp m_op;
    };

    /**
     * your typical logic gate boi
     */
    struct Logic
    {
        bool m_value;
    };

    /**
     * Pipe, generically exposes the output entity to the input one
     */
    struct Pipe
    {
        ActiveEnt m_source;
    };

    // Supported data types
    using WireData = std::variant<Attitude,
                                  AttitudeControl,
                                  Percent,
                                  Deploy,
                                  Pipe>;
}

using wiretype::WireData;


/**
 * Object that have WireInputs and WireOutputs. So far, just Machines inherit.
 * keep WireInputs and WireOutputs as members. move them explicitly
 */
class IWireElement
{

public:

    //WireElement() = default;
    //WireElement(WireElement&& move) = default;
    //virtual ~WireElement() = default;

    // TODO: maybe get rid of the raw pointers somehow

    /**
     * Request a Dependent WireOutput's value to update, by reading the
     * WireInputs it depends on
     * @param pOutput [in] The output that needs to be updated
     */
    virtual void propagate_output(WireOutput* pOutput) = 0;

    /**
     * Request a WireOutput by port. What happens is up to the implemenation,
     * this may access a map, array, or maybe even create a WireOutput on the
     * fly.
     *
     * @param port Port to identify the WireOutput
     * @return Pointer to found WireOutput, nullptr if not found
     */
    virtual WireOutput* request_output(WireOutPort port) = 0;

    /**
     * Request a WireInput by port. What happens is up to the implemenation,
     * this may access a map, array, or maybe even create a WireInput on the
     * fly.
     *
     * @param port Port to identify the WireInput
     * @return Pointer to found WireInput, nullptr if not found
     */
    virtual WireInput* request_input(WireInPort port) = 0;

    // TODO: maybe return ports too, as the pointers returned here can be
    //       invalidated when the SysMachine reallocates

    /**
     * @return Vector of existing WireInputs
     */
    virtual std::vector<WireInput*> existing_inputs() = 0;

    /**
     * @return Vector of existing WireInputs
     */
    virtual std::vector<WireOutput*> existing_outputs() = 0;

};


class WireInput : private LinkedListItem<WireInput, WireOutput>
{
    friend LinkedListItem<WireInput, WireOutput>;
    friend LinkedList<WireInput>;

public:

    explicit WireInput(IWireElement *element, std::string const& name);

    /**
     * Move with new m_element. Use when this is a member of the WireElement
     * where m_element becomes invalid on move
     * @param element
     * @param move
     */
    explicit WireInput(IWireElement *element, WireInput&& move);

    WireInput(WireInput const& copy) = delete;
    WireInput(WireInput&& move) = default;

    // :
    //        m_name(name)
    //{
    //    static_cast<LinkedList<WireInput>*>
    //            (element)->insert((this));
    //}
    void doErase() override;

    //bool is_connected() { return list(); }
    constexpr std::string const& get_name() { return m_name; }

    WireOutput* connected() { return list(); }

    WireData* connected_value();

    /**
     * Get value from connected WireOutput
     */
    template<typename T>
    T* get_if();

    /**
     * Get const value from connected WireOutput
     */
    template<typename T>
    const T* get_if() const;

private:
    IWireElement* m_element;
    std::string m_name;
};

//template <void ()()>
//struct WireOutput
//{
//};

class WireOutput : private LinkedList<WireInput>
{
    friend LinkedList<WireInput>;
    friend LinkedListItem<WireInput, WireOutput>;

public:

    /**
     * Construct a WireOutput
     * @param element Associated WireElement, usually a Machine
     * @param name
     */
    explicit WireOutput(IWireElement* element, std::string const& name);
    explicit WireOutput(IWireElement* element, std::string const& name,
                        WireInput& propagateDepend ...);
    /**
     * Move with new m_element. Use when this is a member of the WireElement
     * where m_element becomes invalid on move
     * @param element
     * @param move
     */
    explicit WireOutput(IWireElement *element, WireOutput&& move);

    WireOutput(WireOutput const& copy) = delete;
    WireOutput(WireOutput&& move) = default;

    //WireOutput(WireElement* element, std::string const& name,
    //           void (WireElement::*propagate_output)()) :
    //        WireOutput(element, name)
    //{
    //    m_propagate_output = propagate_output;
    //}

    std::string const& get_name() { return m_name; }


    //using LinkedListItem<WireOutput, WireElement>::erase;
    //using LinkedList<WireInput>::erase;

    using LinkedList<WireInput>::insert;
    using LinkedList<WireInput>::cut;

    void propagate()
    {
        //(list()->*m_propagate_output)();
        //list()->propagate_output(this);
        //m_element->propagate_output(this);
    }

    WireData& value() { return m_value; }
    const WireData& value() const { return m_value; }

private:
    //unsigned m_port;
    WireData m_value;
    IWireElement* m_element;
    std::string m_name;

    //void (WireElement::*m_propagate_output)();

    //std::vector<>
    //virtual void propagate() {}
};



class SysWire
{
public:

    struct DependentOutput
    {
        IWireElement *m_element;
        WireOutput *m_output;
        unsigned depth;
    };

    SysWire(ActiveScene &scene);
    SysWire(SysWire const& copy) = delete;
    SysWire(SysWire&& move) = delete;

    void update_propigate();
    void connect(WireOutput &wireFrom, WireInput &wireTo);

private:
    std::vector<DependentOutput> m_dependentOutputs;
    UpdateOrderHandle m_updateWire;
};





// TODO: move this somewhere else
template<typename T>
T* WireInput::get_if()
{
    if (list() == nullptr)
    {
        // Not connected to any WireOutput!
        return nullptr;
    }
    else
    {
        // Attempt to get value of variant
        return std::get_if<T> (&(list()->value()));
    }
}

template<typename T>
const T* WireInput::get_if() const
{
    if (list() == nullptr)
    {
        // Not connected to any WireOutput!
        return nullptr;
    }
    else
    {
        // Attempt to get value of variant
        return std::get_if<T>(&(list()->value()));
    }
}


}
