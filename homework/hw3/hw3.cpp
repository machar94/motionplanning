#include <openrave-core.h>
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
    if( !penv->load('hw3.env.xml'))
    {
        return 2;
    }

    RaveDestroy();
    return 0;
}

