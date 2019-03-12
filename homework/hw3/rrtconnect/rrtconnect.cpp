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
#define RESERVE_CAPACITY        10000

using namespace OpenRAVE;

static std::vector<double> noWristRollW = {5.0, 4.0, 3.0, 2.0, 1.0, 0.5, 0.0};
static const float red[4] = {1,0,0,1};
static const float blue[4] = {0,0,1,1};

void normalizeWeights()
{
    double sum = 0.0;
    for (const auto & val : noWristRollW)
    {
        sum += val;
    }
    for (auto & val : noWristRollW)
    {
        val /= sum;
    }
}


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
    std::vector<std::vector<dReal> > getPath(RRTNode* const node);

    // Implements a nearest neighbor via linear search. The decision was made to
    // use a linear search as opposed to using a kd-tree as apparently the tree
    // only scales better once the number of nodes in the space is more than
    // 2000.
    RRTNode* nearestNeighbor(const std::vector<double>& q) const;

    RRTNode* nearestNeighbor(RRTNode* const node) const;

    size_t getSize() const { return nodes.size(); }

    void reserve(const int capacity) { nodes.reserve(capacity); }

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

std::vector<std::vector<dReal>> NodeTree::getPath(RRTNode* const node)
{    
    RRTNode* curr = node;
    std::vector<std::vector<dReal> > path;
    
    while (curr->getParent() != NULL)
    {
        path.push_back(curr->getConfig());
        curr = curr->getParent();
    }

    // Finall add start config to path
    path.push_back(curr->getConfig());
    return path;
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
    unsigned smoothIter;     // Number of iterations of smoothing that should be applied

    std::vector<dReal> llim;        // lower limits of active joints
    std::vector<dReal> ulim;        // upper limits of active joints

    std::mt19937 gen;               // random number generator
    std::vector<std::uniform_real_distribution<double> > dists;

    RobotBasePtr probot;            // Holds pointer to the PR2 robot in scene
    std::vector<GraphHandlePtr> ghandle;         // Handle to all of the points being plotted

    std::vector<std::vector<double> > path;
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

    bool SetSmoothIteration(std::ostream& sout, std::istream& sinput);

    bool printClass(std::ostream& sout, std::istream& sinput);

    bool run(std::ostream& sout, std::istream& sinput);

    bool resetTree(std::ostream& sout, std::istream& sinput);

private:

    template <class T>
    void printVector(std::string s, const std::vector<T>& v) const;

    void sampleConfiguration(std::vector<double>& q);

    bool takeStepsToSample(RRTNode* node, const std::vector<double>& q);

    void plotTrajectory(const float color[4], std::vector<std::vector<double> >& path);

    void executeTrajectory(std::vector<std::vector<double> >& p);

    void smoothPath();

    std::vector<dReal> calcDirVector(const std::vector<dReal>& v1, 
                                     const std::vector<dReal>& v2,
                                     const std::vector<double>& w);

    double pathLength(std::vector<std::vector<double> > & path);

};
 
RRTConnect::RRTConnect(EnvironmentBasePtr penv, std::istream& ss) 
    : ModuleBase(penv)
    , tree(NodeTree())
    , goal(NULL)
    , numSamples(0)
    , stepSize(0.0)
    , goalBias(DEFAULT_GOAL_BIAS)
    , gSamples(0)
    , smoothIter(NUM_SMOOTHING_ITER)
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
    RegisterCommand("SetSmoothIteration",boost::bind(&RRTConnect::SetSmoothIteration,this,_1,_2),
                    "Sets how many iterations of smoothing should be applied");
    RegisterCommand("printClass",boost::bind(&RRTConnect::printClass,this,_1,_2),
                    "Debug function for printing internal members of rrt class");
    RegisterCommand("Init",boost::bind(&RRTConnect::Init,this,_1,_2),
                    "All prep work needed done here before calling run");
    RegisterCommand("run",boost::bind(&RRTConnect::run,this,_1,_2),
                    "Run RRT Connect algorithm");
    RegisterCommand("resetTree",boost::bind(&RRTConnect::resetTree,this,_1,_2),
                    "Clears all the nodes in the NodeTree except start");

    normalizeWeights();
    printVector("Normalized W:", noWristRollW);

    std::random_device rd;
    tree.reserve(RESERVE_CAPACITY);

    // For testing so seed generated is the same
    // srand(313);
    // gen.seed(313);

    // srand(time(NULL));
    // gen.seed(rd());
}

std::vector<dReal> RRTConnect::calcDirVector(
    const std::vector<dReal>& v1, 
    const std::vector<dReal>& v2,
    const std::vector<double>& w)
{
    std::vector<dReal> dir (v2.size(), 0.0);
    double dist = tree.calcEuclidianDist(v1, v2, w);
    
    for (unsigned i = 0; i < v2.size(); ++i)
    {
        dir[i] = (v2[i]-v1[i]) / dist * stepSize;
    }
    return dir;
}

void RRTConnect::smoothPath()
{
    smoothedPath.clear();
    smoothedPath.insert(smoothedPath.begin(), path.begin(), path.end());

    for (unsigned i = 0; i < smoothIter; ++i)
    {
        // 1. Choose two nodes without repeating
        int idx1 = 0, idx2 = 0;

        // Desire:
        // 1. Nodes selected should not be the same
        // 2. Nodes selected should not be right next to each other
        while ((idx1 == idx2) || std::abs(idx2-idx1) == 1)
        {
            idx1 = rand() % smoothedPath.size();
            idx2 = rand() % smoothedPath.size();

            if (idx1 > idx2)
            {
                std::swap(idx1,idx2);
            }
        }
        std::vector<dReal> n1 = smoothedPath[idx1];
        std::vector<dReal> n2 = smoothedPath[idx2];

        // 2. Walk from node 1 towards node 2 with stepSize
        bool smoothing = true;
        std::vector<std::vector<dReal> > shorterPath;

        std::vector<double> w(n1.size(), 1.0);
        std::vector<dReal> vec = calcDirVector(n1, n2, w);
        
        while (smoothing)
        {
            // Take a step forward to node 2
            n1 = n1 + vec;

            // Check that position is valid
            probot->SetActiveDOFValues(n1);
            if (GetEnv()->CheckCollision(probot) || 
                probot->CheckSelfCollision())
            {
                shorterPath.clear();
                break;
            }
            else
            {
                shorterPath.push_back(n1);
            }

            // If sufficiently close to node 2 stop
            double dist = tree.calcEuclidianDist(n1,n2);
            if (dist <= stepSize)
            {
                break;
            }
        }

        if (shorterPath.size() > 0)
        {
            smoothedPath.erase(smoothedPath.begin()+idx1+1, smoothedPath.begin()+idx2);
            smoothedPath.insert(smoothedPath.begin()+idx1+1, shorterPath.begin(), shorterPath.end());
        }
    }
}

bool RRTConnect::SetSmoothIteration(std::ostream& sout, std::istream& sinput)
{
    std::string sval;
    sinput >> sval;
    smoothIter = atof(sval.c_str());
    return true;
}

void RRTConnect::executeTrajectory(std::vector<std::vector<double> >& p)
{
    probot->SetActiveDOFValues(startQ);

    TrajectoryBasePtr traj = RaveCreateTrajectory(GetEnv(), "");
    ConfigurationSpecification conspec = probot->GetActiveConfigurationSpecification("linear");
    conspec.AddDeltaTimeGroup();
    traj->Init(conspec);

    int i = 0;
    std::vector<dReal> point;
    for (auto rit = p.rbegin(); rit != p.rend(); ++rit)
    {
        point = *rit;
        point.push_back(i*0.01);
        traj->Insert(i, point, conspec, true);
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
    
    // Clear any existing points in environment
    ghandle.clear();

    // srand(313);
    // gen.seed(313);

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
    startQ.clear();
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
    goalQ.clear();
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
        // int nbefore = tree.getSize();
        if(takeStepsToSample(nearestNode, sampleQ))
        {
            // int nafter = tree.getSize();
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
        if ((endTime - startTime) > 3600)
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

    path = tree.getPath(goal);
    plotTrajectory(red, path);

    double pathDist = pathLength(path);
    
    // Time path smoothing
    startTime = time(NULL);
    smoothPath();
    endTime = time(NULL);
    std::cout << "\nSmooth Time : " << endTime - startTime << std::endl;
    
    plotTrajectory(blue, smoothedPath);
    executeTrajectory(smoothedPath);
    
    pathDist = pathLength(path);
    double smoothedPathDist = pathLength(smoothedPath);
    sout << smoothedPathDist << " ";    // Send out smoothed path dist
    sout << endTime - startTime << " ";
    sout << pathDist << " ";

    std::cout << "\nPath Length : " << pathDist << std::endl;
    std::cout << "Path Nodes  : " << path.size() << std::endl;
    std::cout << "Smoothed Len: " << smoothedPathDist << std::endl;
    std::cout << "Smoothed Nod: " << smoothedPath.size() << std::endl;

    return true;
}

bool RRTConnect::takeStepsToSample(RRTNode* nn, const std::vector<double>& sample)
{
    std::vector<double> new_q(sample.size(), 0.0);

    // Calculate unit step direction vector
    std::vector<double> vec = calcDirVector(nn->getConfig(), sample, noWristRollW);

    RRTNode* currNode = nn;
    while (true)
    {
        // Check if distance between sample and current node is small
        double dist = tree.calcEuclidianDist(
            currNode->getConfig(), sample, noWristRollW);
        if (dist <= stepSize)
        {
            new_q = sample;
        }
        else
        {
            new_q = currNode->getConfig() + vec; 
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

void RRTConnect::plotTrajectory(const float color[4], std::vector<std::vector<double> >& path) 
{
    // Set robot to that configuration
    std::vector<float> p(3,0.0);

    for (auto rit = path.rbegin(); rit != path.rend(); ++rit)
    {
        probot->SetActiveDOFValues(*rit);
        // Get the position of the end effector
        Transform endEffector = probot->GetLinks()[L_END_EFFECTOR_LINK_IDX]->GetTransform();
        p[0] = endEffector.trans.x;
        p[1] = endEffector.trans.y;
        p[2] = endEffector.trans.z;
        ghandle.push_back(GetEnv()->plot3(&p[0],1,12,5,color,0));
    }
}

double RRTConnect::pathLength(std::vector<std::vector<double> >& path)
{
    // There should be at least two elements in the path
    assert(path.size() > 1);

    double dist = 0.0;
    for (unsigned i = 0; i < path.size()-1; ++i)
    {
        dist += tree.calcEuclidianDist(path[i],path[i+1]);
    }
    return dist;    
}