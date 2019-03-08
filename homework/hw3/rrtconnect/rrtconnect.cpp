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
#include <time.h>
#include <chrono>

#define DEFAULT_GOAL_BIAS       10
#define L_END_EFFECTOR_LINK_IDX 49
#define NUM_SMOOTHING_ITER      200

using namespace OpenRAVE;

const std::vector<double> noWristRollW = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0};

const std::vector<double> operator+(const std::vector<double>& v1, 
                                   const std::vector<double>& v2)
{
    std::vector<double> v3(v1.size(), 0.0);
    for (unsigned i = 0; i < v1.size(); ++i)
    {
        v3[i] = v1[i] + v2[i];
    }
    return v3;
}

class RRTNode
{
private:
    const std::vector<double> q;     // Configuration of robot represented by node
    RRTNode* parent;                // Parent node

public:
    RRTNode(const std::vector<double>& q) : q(q), parent(NULL) {};

    RRTNode(const std::vector<double>& q, RRTNode* const p) : q(q), parent(p) {};
    
    // Apprently defining a member function inside the class definition means
    // that the compiler will inline this. The other way is to explicitly use
    // inline when defining the method outside of the class.
    void setParent(RRTNode* const node) { parent = node; }

    RRTNode* getParent() const { return parent; }

    const std::vector<double>& getConfig() const { return q; }
    
    bool isRoot() const { return parent == NULL; }
};

class NodeTree
{
private:
    std::vector<RRTNode*> nodes;

public:
    NodeTree() {} ;

    void clear();

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
    void getPath(RRTNode* const node) const;

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
    RRTNode* nearestNeighbor(const std::vector<double>& q) const;

    RRTNode* nearestNeighbor(RRTNode* const node) const;

    size_t getSize() const { return nodes.size(); }

    // Calculates the euclidian distance between two configurations
    double calcEuclidianDist(const std::vector<double>& q1,
                            const std::vector<double>& q2) const;

    // Calculates the euclidian distance between configurations where each
    // dimension can be weighted differently
    double calcEuclidianDist(const std::vector<double>& q1,
                            const std::vector<double>& q2,
                            const std::vector<double>& w) const;
};

void NodeTree::clear()
{
    for (unsigned i = 0; i < nodes.size(); ++i)
    {
        delete nodes[i];
    }
    nodes.clear();
}

void NodeTree::del(RRTNode* const node) 
{
    delete node;
    nodes.erase(std::remove(nodes.begin(), nodes.end(), node), nodes.end());
}

void NodeTree::getPath(RRTNode* const node) const
{    
    RRTNode* curr = node;
    path.clear();
    
    while (curr->getParent() != NULL)
    {
        path.push_back(curr->getConfig());
        curr = curr->getParent();
    }
}

RRTNode* NodeTree::nearestNeighbor(RRTNode* const node) const
{
    return nearestNeighbor(node->getConfig());
}

RRTNode* NodeTree::nearestNeighbor(const std::vector<double>& q) const
{
    RRTNode* nearestNeighbor = nodes[0];

    double minDist = calcEuclidianDist(q, nodes[0]->getConfig(), noWristRollW);
    

    for ( auto node : nodes )
    {
        double dist = calcEuclidianDist(q, node->getConfig(), noWristRollW);
        if (dist < minDist)
        {
            minDist = dist;
            nearestNeighbor = node;
        }
    }
    return nearestNeighbor;
}

double NodeTree::calcEuclidianDist(
    const std::vector<double>& q1,
    const std::vector<double>& q2) const
{
    std::vector<double> weights (q1.size(), 1.0);
    return calcEuclidianDist(q1, q2, weights);
}

double NodeTree::calcEuclidianDist(
    const std::vector<double>& q1,
    const std::vector<double>& q2,
    const std::vector<double>& w) const
{
    double sum = 0.0;
    for (unsigned i = 0; i < q1.size(); i++)
    {
        sum += w[i]*(q1[i] - q2[i])*(q1[i] - q2[i]);
    }
    return sqrtf(sum);
}


class RRTConnect : public ModuleBase
{
private:
    std::vector<double> goalQ;       // Goal configuration to reach
    std::vector<double> startQ;      // Start (root) configuration
    NodeTree tree;
    RRTNode* goal;

    int numSamples;     // Number of samples to try before stopping
    double stepSize;     // Length of step towards sampled config
    int goalBias;       // Select goalQ every goalBias percent of time
    int gSamples;       // Number of time the goal has been sampled

    std::vector<dReal> llim;        // lower limits of active joints
    std::vector<dReal> ulim;        // upper limits of active joints

    std::mt19937 gen;               // random number generator
    std::vector<std::uniform_real_distribution<double> > dists;

    RobotBasePtr probot;            // Holds pointer to the PR2 robot in scene
    std::vector<GraphHandlePtr> ghandle;         // Handle to all of the points being plotted

    std::vector<RRTNode*> path;
    std::vector<std::vector<double> > smoothedPath;

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

    bool resetTree(std::ostream& sout, std::istream& sinput);

private:

    template <class T>
    void printVector(std::string s, const std::vector<T>& v) const;

    void sampleConfiguration(std::vector<double>& q);

    bool takeStepsToSample(RRTNode* node, const std::vector<double>& q);

    void plotTrajectory();

    void executeTrajectory();

    void smoothPath();

};
 
RRTConnect::RRTConnect(EnvironmentBasePtr penv, std::istream& ss) 
    : ModuleBase(penv)
    , tree(NodeTree())
    , goal(NULL)
    , numSamples(0)
    , stepSize(0.0)
    , goalBias(DEFAULT_GOAL_BIAS)
    , gSamples(0)
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
    RegisterCommand("resetTree",boost::bind(&RRTConnect::resetTree,this,_1,_2),
                    "Clears all the nodes in the NodeTree except start");

    std::random_device rd;
    gen.seed(rd());
}

void smoothPath()
{
    smoothedPath.clear();

    for (unsigned i = 0; i < NUM_SMOOTHING_ITER; ++i)
    {
        // 1. Choose two nodes without repeating
        int idx1 = 0, idx2 = 0;
        
        srand (time(NULL))
        while (idx1 == idx2)
        {
            idx1 = rand() % path.size();
            idx2 = rand() % path.size();

            if (idx1 > idx2)
            {
                std::swap(idx1,idx2);
            }
        }
        std::vector<dReal> n1 = path[idx1];
        std::vector<dReal> n2 = path[idx2];

        // 2. Walk from node 1 towards node 2 with stepSize
        bool smoothing = true;
        std::vector<dReal> shorterPath;

        while (smoothing)
        {
            
        }
    }
}

void RRTConnect::executeTrajectory()
{
    TrajectoryBasePtr traj = RaveCreateTrajectory(GetEnv(), "");
    traj->Init(probot->GetActiveConfigurationSpecification());

    int i = 0;
    std::vector<double> point;
    for (auto rit = path.rbegin(); rit != path.rend(); ++rit)
    {
        point = (*rit)->getConfig();
        traj->Insert(i, point);
        i++;
    }
    probot->GetController()->SetPath(traj);
}

bool RRTConnect::resetTree(std::ostream& sout, std::istream& sinput)
{
    tree.clear();
    goal = NULL;

    RRTNode* root = new RRTNode(startQ);
    tree.add(root);

    gSamples = 0;
    return true;
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
    probot = vrobots.at(0);

    // Retrieve the lower and upper limits for all active joints
    probot->GetActiveDOFLimits(llim,ulim);
    llim[4] = -PI;
    llim[6] = -PI;
    ulim[4] = PI;
    ulim[6] = PI;

    for (unsigned i = 0; i < llim.size(); ++i)
    {
        std::uniform_real_distribution<double> dis(llim[i], ulim[i]);
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
    std::cout << "Goal Node*  : " << goal << std::endl;
    std::cout << "Tree*       : " << &tree << std::endl;
    printVector("Lower Limits:", llim);
    printVector("Upper Limits:", ulim);
    return true;
}

void RRTConnect::sampleConfiguration(std::vector<double>& q)
{
    if ((rand()%100)+1 <= goalBias)
    {
        q = goalQ;
        gSamples++;
        // printVector("Processing goal:", q);
    }
    else
    {
        while (true)
        {
            // Generate random sample config
            for (unsigned i = 0; i < q.size(); ++i)
            {
                q[i] = dists[i](gen);
            }

            // Check for collision
            probot->SetActiveDOFValues(q);
            if (!(GetEnv()->CheckCollision(RobotBaseConstPtr(probot)) || 
                probot->CheckSelfCollision()))
            {
                break;
            }
        }
        // printVector("Processing sample:", q);
    }
}

bool RRTConnect::run(std::ostream& sout, std::istream& sinput)
{
    time_t startTime(time(NULL)), endTime;
    unsigned i = 1;

    while (true)
    {
        if (i % 50 == 0)
        {
            std::cout << "Processed " << i << " samples | " 
                      << "Nodes: " << tree.getSize() <<  std::endl;
        }

        std::vector<double> sampleQ(startQ.size(), 0.0);
        sampleConfiguration(sampleQ);
        RRTNode* nearestNode = tree.nearestNeighbor(sampleQ);
        
        // sampleQ has been reached successfully if it returns true
        int nbefore = tree.getSize();
        if(takeStepsToSample(nearestNode, sampleQ))
        {
            int nafter = tree.getSize();
            // std::cout << "Reached Sample | Added " << nafter - nbefore << std::endl;
            if (sampleQ == goalQ)
            {
                endTime = time(NULL);
                std::cout << "Goal configuration reached!\n";
                std::cout << "Time        : " << endTime - startTime << std::endl;
                std::cout << "Total Nodes : " << tree.getSize() << std::endl;
                std::cout << "Samples     : " << i << std::endl;
                std::cout << "Goal Samples: " << gSamples << std::endl;

                sout << endTime - startTime << " ";
                sout << tree.getSize() << " ";
                sout << i << " ";
                
                break;
            }
        }

        endTime = time(NULL);
        if ((endTime - startTime) > 600) // Take no longer than 5 minutes
        {
            std::cout << "==== Sorry Timed Out ===" << std::endl;
            std::cout << "Start Time  : " << startTime << std::endl;
            std::cout << "End Time    : " << endTime << std::endl;
            std::cout << "Total Nodes : " << tree.getSize() << std::endl;
            std::cout << "Samples     : " << i << std::endl;
            break;
        }

        i++;
    }

    tree.getPath(goal);
    plotTrajectory();
    executeTrajectory();

    return true;
}

bool RRTConnect::takeStepsToSample(RRTNode* nn, const std::vector<double>& sample)
{
    std::vector<double> new_q(sample.size(), 0.0);

    // Calculate unit step direction vector
    std::vector<double> dir (sample.size(), 0.0);
    double dist = tree.calcEuclidianDist(
        nn->getConfig(), sample, noWristRollW);
    
    for (unsigned i = 0; i < sample.size(); ++i)
    {
        dir[i] = sample[i]-(nn->getConfig()[i]);
        dir[i] = dir[i] / dist * stepSize;
    }

    RRTNode* currNode = nn;
    while (true)
    {
        // Check if distance between sample and current node is small
        dist = tree.calcEuclidianDist(
            currNode->getConfig(), sample, noWristRollW);
        if (dist <= stepSize)
        {
            new_q = sample;
        }
        else
        {
            new_q = currNode->getConfig() + dir; 
        }

        // Check potential node
        probot->SetActiveDOFValues(new_q);
        if (!(GetEnv()->CheckCollision(RobotBaseConstPtr(probot))) && 
            !(probot->CheckSelfCollision()))
        {
            RRTNode* newNode = new RRTNode(new_q, currNode);
            tree.add(newNode);
            currNode = newNode;

            if (sample == new_q)
            {
                if (new_q == goalQ)
                {   
                    goal = newNode;
                }
                return true;
            }
        }
        else
        {
            return false;
        }   
    }
}

void RRTConnect::plotTrajectory() 
{
    // Clear any existing points in environment
    ghandle.clear();

    // Get path of nodes
    std::vector<RRTNode*> path = tree.getPath(goal);

    // Set robot to that configuration
    std::vector<float> p(3,0.0);
    float red[4] = {1,0,0,1};
    for (auto rit = path.rbegin(); rit != path.rend(); ++rit)
    {
        probot->SetActiveDOFValues((*rit)->getConfig());
        // Get the position of the end effector
        Transform endEffector = probot->GetLinks()[L_END_EFFECTOR_LINK_IDX]->GetTransform();
        p[0] = endEffector.trans.x;
        p[1] = endEffector.trans.y;
        p[2] = endEffector.trans.z;
        ghandle.push_back(GetEnv()->plot3(&p[0],1,12,5,red,0));
    }
}