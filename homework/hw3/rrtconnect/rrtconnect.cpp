#include <openrave/openrave.h>
#include <openrave-core.h>

#include <openrave/plugin.h>
#include <boost/bind.hpp>

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <math.h>
#include <random>


using namespace OpenRAVE;


class RRTNode
{
private:
    const std::vector<float> q;     // Configuration of robot represented by node
    RRTNode* parent;                // Parent node

public:
    RRTNode(const std::vector<float>& q) : q(q), parent(NULL) {};

    RRTNode(const std::vector<float>& q, RRTNode* const p) : q(q), parent(p) {};
    
    // Apprently defining a member function inside the class definition means
    // that the compiler will inline this. The other way is to explicitly use
    // inline when defining the method outside of the class.
    void setParent(RRTNode* const node) { parent = node; }

    RRTNode* getParent() const { return parent; }

    const std::vector<float>& getConfig() const { return q; }
    
    bool isRoot() const { return parent == NULL; }
};


class NodeTree
{
private:
    std::vector<RRTNode*> nodes;

public:
    NodeTree() {} ;

    NodeTree(RRTNode* node) { nodes.push_back(node); }

    // Remember that applying the const before the type specifies that the data
    // underlying the pointer is constant where as applying the const after the
    // asterisk specifies that the pointer itself should not be modified by the
    // function
    void add(RRTNode* const node) { nodes.push_back(node); }
    
    // Makes use of the reverse - erase idiom from algorithm library. This
    // method allows for removing multiple elements in a vector with the same
    // value without needing to shift elements to the left multiple times after
    // removing an element. The function takes care of releasing the memory
    // pointed at by the node as well.
    void del(RRTNode* const node);
    
    // The vector returned represents the path from the root of the tree to the
    // node in a reverse view. Originally thought about adding an additional
    // paramter to the function to specify direction (i.e. root -> node or node
    // -> root) but after looking at it further, I don't really need a reverse
    // container - just a reversed view of the container which can be achieved
    // with rbegin() and rend() reverse iterators
    std::vector<RRTNode*> getPath(RRTNode* const node) const;

    // Implements a nearest neighbor via linear search. The decision was made to
    // use a linear search as opposed to using a kd-tree as apparently the tree
    // only scales better once the number of nodes in the space is more than
    // 2000.
    //
    // One assumption made is that because the constructor for the class
    // requires that a node initialize the root of the tree, there will always
    // be at least one node in the list of nodes. So given a configuration q, a
    // valid node will always be returned i.e. you don't need to worry about the
    // tree being empty.
    RRTNode* nearestNeighbor(const std::vector<float>& q) const;

    RRTNode* nearestNeighbor(RRTNode* const node) const;

private:

    // Calculates the euclidian distance between two configurations
    float calcEuclidianDist(const std::vector<float>& q1,
                            const std::vector<float>& q2) const;

    // Calculates the euclidian distance between configurations where each
    // dimension can be weighted differently
    float calcEuclidianDist(const std::vector<float>& q1,
                            const std::vector<float>& q2,
                            const std::vector<float>& w) const;
};

void NodeTree::del(RRTNode* const node) 
{
    delete node;
    nodes.erase(std::remove(nodes.begin(), nodes.end(), node), nodes.end());
}

std::vector<RRTNode*> NodeTree::getPath(RRTNode* const node) const
{    
    RRTNode* curr = node;
    std::vector<RRTNode*> path;
    
    while (curr->getParent() != NULL)
    {
        path.push_back(curr);
        curr = curr->getParent();
    }

    return path;
}

RRTNode* NodeTree::nearestNeighbor(RRTNode* const node) const
{
    return nearestNeighbor(node->getConfig());
}

RRTNode* NodeTree::nearestNeighbor(const std::vector<float>& q) const
{
    RRTNode* nearestNeighbor = nodes[0];
    float minDist = calcEuclidianDist(q, nodes[0]->getConfig());
    

    for ( auto node : nodes )
    {
        float dist = calcEuclidianDist(q, node->getConfig());
        if (dist < minDist)
        {
            minDist = dist;
            nearestNeighbor = node;
        }
    }
    return nearestNeighbor;
}

float NodeTree::calcEuclidianDist(
    const std::vector<float>& q1,
    const std::vector<float>& q2) const
{
    std::vector<float> weights (q1.size(), 1.0);
    return calcEuclidianDist(q1, q2, weights);
}

float NodeTree::calcEuclidianDist(
    const std::vector<float>& q1,
    const std::vector<float>& q2,
    const std::vector<float>& w) const
{
    float sum = 0.0;
    for (unsigned i = 0; i < q1.size(); i++)
    {
        sum += w[i]*(q1[i] - q2[i])*(q1[i] - q2[i]);
    }
    return sqrtf(sum);
}


class RRTConnect : public ModuleBase
{
private:
    std::vector<float> goalQ;       // Goal configuration to reach
    std::vector<float> startQ;      // Start (root) configuration
    NodeTree tree;
    RRTNode* goal;

    int numSamples;     // Number of samples to try before stopping
    float stepSize;     // Length of step towards sampled config
    int goalBias;       // Select goalQ every goalBias samples
    int biasCounter;    // Tracks how long it's been since last sampling goalQ

    std::vector<dReal> llim;        // lower limits of active joints
    std::vector<dReal> ulim;        // upper limits of active joints

    std::mt19937 gen;               // random number generator
    std::vector<std::uniform_real_distribution<float> > dists;

public:

    RRTConnect(EnvironmentBasePtr penv, std::istream& ss);

    virtual ~RRTConnect() {}

    bool Init(std::ostream& sout, std::istream& sinput);
    
    bool SetGoal(std::ostream& sout, std::istream& sinput);

    bool SetStart(std::ostream& sout, std::istream& sinput);

    bool SetStepSize(std::ostream& sout, std::istream& sinput);

    bool SetNumSamples(std::ostream& sout, std::istream& sinput);

    bool SetGoalBias(std::ostream& sout, std::istream& sinput);

    bool printClass(std::ostream& sout, std::istream& sinput);

    bool run(std::ostream& sout, std::istream& sinput);

private:

    template <class T>
    void printVector(std::string s, const std::vector<T>& v) const;

    void sampleConfiguration(std::vector<float>& q);

};
 
RRTConnect::RRTConnect(EnvironmentBasePtr penv, std::istream& ss) 
    : ModuleBase(penv)
    , tree(NodeTree())
    , goal(NULL)
    , numSamples(0)
    , stepSize(0.0)
    , goalBias(10)
    , biasCounter(1)
{
    RegisterCommand("SetGoal",boost::bind(&RRTConnect::SetGoal,this,_1,_2),
                    "Sets the goal config for motion planner");
    RegisterCommand("SetStart",boost::bind(&RRTConnect::SetStart,this,_1,_2),
                    "Sets the start config for motion planner");
    RegisterCommand("SetStepSize",boost::bind(&RRTConnect::SetStepSize,this,_1,_2),
                    "Sets the size of the step to take towards a sample config ");
    RegisterCommand("SetNumSamples",boost::bind(&RRTConnect::SetNumSamples,this,_1,_2),
                    "Sets the number of samples to sample in cspace before timing out");
    RegisterCommand("SetGoalBias",boost::bind(&RRTConnect::SetGoalBias,this,_1,_2),
                    "Sets how often the goal config should be sampled");
    RegisterCommand("printClass",boost::bind(&RRTConnect::printClass,this,_1,_2),
                    "Debug function for printing internal members of rrt class");
    RegisterCommand("Init",boost::bind(&RRTConnect::Init,this,_1,_2),
                    "All prep work needed done here before calling run");
    RegisterCommand("run",boost::bind(&RRTConnect::run,this,_1,_2),
                    "Run RRT Connect algorithm");
    std::random_device rd;
    gen.seed(rd());
}

template <class T>
void RRTConnect::printVector(std::string s, const std::vector<T>& v) const
{
    std::cout << s << " ";
    for (const auto& val : v)
    {
        std::cout << val << " ";
    }
    std::cout << std::endl;
}

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrtconnect" ) {
        return InterfaceBasePtr(new RRTConnect(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back("RRTConnect");
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

bool RRTConnect::SetStart(std::ostream& sout, std::istream& sinput)
{
    std::string sval;
    while (sinput >> sval)
    {
        startQ.push_back(atof(sval.c_str()));
    }
    RRTNode* root = new RRTNode(startQ);
    tree.add(root);

    return true;
}

bool RRTConnect::SetGoal(std::ostream& sout, std::istream& sinput)
{
    std::string sval;
    while (sinput >> sval)
    {
        goalQ.push_back(atof(sval.c_str()));
    }
    return true;
}

bool RRTConnect::Init(std::ostream& sout, std::istream& sinput)
{
    std::cout << "Initializing RRT-Connect Algorithm..." << std::endl;

    std::vector<RobotBasePtr> vrobots;
    GetEnv()->GetRobots(vrobots);
    RobotBasePtr probot = vrobots.at(0);

    // Retrieve the lower and upper limits for all active joints
    probot->GetActiveDOFLimits(llim,ulim);

    for (unsigned i = 0; i < llim.size(); ++i)
    {
        std::uniform_real_distribution<float> dis(llim[i], ulim[i]);
        dists.push_back(dis);
    }

    return true;
}

bool RRTConnect::SetStepSize(std::ostream& sout, std::istream& sinput)
{
    std::string sval;
    sinput >> sval;
    stepSize = atof(sval.c_str());
    return true;
}

bool RRTConnect::SetNumSamples(std::ostream& sout, std::istream& sinput)
{
    std::string sval;
    sinput >> sval;
    numSamples = atoi(sval.c_str());
    return true;
}

bool RRTConnect::SetGoalBias(std::ostream& sout, std::istream& sinput)
{
    std::string sval;
    sinput >> sval;
    goalBias = atoi(sval.c_str());
    return true;
}

bool RRTConnect::printClass(std::ostream& sout, std::istream& sinput)
{
    printVector("Start Config:", startQ);
    printVector("Goal Config :", goalQ);
    std::cout << "# Samples   : " << numSamples << std::endl;
    std::cout << "Step Size   : " << stepSize << std::endl;
    std::cout << "Goal Bias   : " << goalBias << std::endl;
    std::cout << "Bias Counter: " << biasCounter << std::endl;
    std::cout << "Goal Node*  : " << goal << std::endl;
    std::cout << "Tree*       : " << &tree << std::endl;
    printVector("Lower Limits:", llim);
    printVector("Upper Limits:", ulim);
    return true;
}

void RRTConnect::sampleConfiguration(std::vector<float>& q)
{
    if (biasCounter == goalBias)
    {
        q = goalQ;
        biasCounter = 1;
    }
    else
    {
        for (auto & udist : dists)
        {
            q.push_back(udist(gen));
        }
        biasCounter++;
    }
    // printVector("Sample:", q);
}

bool RRTConnect::run(std::ostream& sout, std::istream& sinput)
{
    
    for (int i = 0; i < 20; i++)
    {
        std::vector<float> config;
        sampleConfiguration(config);
    }
    return true;
}