#include <openrave-core.h>
#include <openrave/openrave.h>
#include <vector>
#include <cstring>
#include <sstream>

using namespace OpenRAVE;
using namespace std;

int main(int argc, char ** argv)
{
    RaveInitialize(true); // start openrave core
    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment
    vector<dReal> vsetvalues;

    // load the scene
    char* env_cstr = "hw3.env.xml";
    std::string envName(env_cstr);
    if( !penv->Load(envName))
    {
        return 2;
    }

    RaveDestroy();
    return 0;
}

