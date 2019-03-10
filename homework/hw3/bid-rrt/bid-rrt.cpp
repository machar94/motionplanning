#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace OpenRAVE;

class bidirect-rrtconnect : public ModuleBase
{
public:
    bidirect-rrtconnect(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&bidirect-rrtconnect::MyCommand,this,_1,_2),
                        "This is an example command");
    }
    virtual ~bidirect-rrtconnect() {}
    
    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        sout << "output";
        return true;
    }
};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "bidirect-rrtconnect" ) {
        return InterfaceBasePtr(new bidirect-rrtconnect(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("bidirect-rrtconnect");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

