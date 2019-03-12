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

using namespace OpenRAVE;

#define DEFAULT_GOAL_BIAS       10
#define L_END_EFFECTOR_LINK_IDX 49
#define NUM_SMOOTHING_ITER      200
#define RESERVE_CAPACITY        10000

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

std::vector<double> operator+(const std::vector<double>& v1, 
                                   const std::vector<double>& v2)
{
    std::vector<double> v3(v1.size(), 0.0);
    for (unsigned i = 0; i < v1.size(); ++i)
    {
        v3[i] = v1[i] + v2[i];
    }
    return v3;
}

// Calculates the euclidian distance between configurations where each
// dimension can be weighted differently
double calcEuclidianDist(
    const std::vector<double>& q1,
    const std::vector<double>& q2,
    const std::vector<double>& w)
{
    double sum = 0.0;
    for (unsigned i = 0; i < q1.size(); i++)
    {
        sum += w[i]*(q1[i] - q2[i])*(q1[i] - q2[i]);
    }
    return sqrtf(sum);
}

// Calculates the euclidian distance between two configurations
double calcEuclidianDist(
    const std::vector<double>& q1,
    const std::vector<double>& q2)
{
    std::vector<double> weights (q1.size(), 1.0);
    return calcEuclidianDist(q1, q2, weights);
}


class RRTNode
{
private:
    const std::vector<double> q;     // Configuration of robot represented by node
    RRTNode* parent;                 // Parent node

public:
    RRTNode(const std::vector<double>& q) : q(q), parent(NULL) {};

    RRTNode(const std::vector<double>& q, RRTNode* const p) : q(q), parent(p) {};
    
    // Apprently defining a member function inside the class definition means
    // that the compiler will inline this. The other way is to explicitly use
    // inline when defining the method outside of the class.
    void setParent(RRTNode* const node) { parent = node; }

    RRTNode* getParent() const { return parent; }

    const std::vector<double>& getConfig() const { return q; }
};


class NodeTree
{
private:
    std::vector<RRTNode*> nodes;

public:
    NodeTree();

    NodeTree(RRTNode* node);

    void clear();

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

    RRTNode* getRoot() const { return nodes[0]; }

    size_t getSize() const { return nodes.size(); }
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

NodeTree::NodeTree()
{
    nodes.reserve(RESERVE_CAPACITY);
}

NodeTree::NodeTree(RRTNode* node)
{
    nodes.reserve(RESERVE_CAPACITY);
    nodes.push_back(node);
}

class BiRRT : public ModuleBase
{
private:
    std::vector<double> goalQ;      // Goal configuration to reach
    std::vector<double> startQ;     // Start (root) configuration
    NodeTree stree;                 // Start tree 
    NodeTree gtree;                 // Goal tree

    int numSamples;                 // Number of samples to try before stopping
    double stepSize;                // Length of step towards sampled config
    unsigned smoothIter;            // Number of iterations of smoothing that should be applied

    std::vector<dReal> llim;        // Lower limits of active joints
    std::vector<dReal> ulim;        // Upper limits of active joints

    int randseed;                   // Seed to generate random sequence (used for testing)
    std::mt19937 gen;               // Random number generator
    std::vector<std::uniform_real_distribution<double> > dists;

    RobotBasePtr probot;            // Holds pointer to the PR2 robot in scene

    // Handle to all of the points being plotted
    std::vector<GraphHandlePtr> ghandle;

    std::vector<std::vector<double> > path;
    std::vector<std::vector<double> > smoothedPath;

    enum Status { TIME_OUT, SUCCESS, INVALID };

public:

    BiRRT(EnvironmentBasePtr penv, std::istream& ss);

    virtual ~BiRRT() {}

    bool Init(std::ostream& sout, std::istream& sinput);
    
    bool SetGoal(std::ostream& sout, std::istream& sinput);

    bool SetStart(std::ostream& sout, std::istream& sinput);

    bool SetStepSize(std::ostream& sout, std::istream& sinput);

    bool SetSmoothIteration(std::ostream& sout, std::istream& sinput);

    bool SetRandomSeed(std::ostream& sout, std::istream& sinput);
    
    bool printClass(std::ostream& sout, std::istream& sinput);

    bool resetTrees(std::ostream& sout, std::istream& sinput);

    bool run(std::ostream& sout, std::istream& sinput);

private:

    template <class T>
    void printVector(std::string s, const std::vector<T>& v) const;

    void printReport(Status status, 
                     time_t start, 
                     time_t end, 
                     int samples) const;

    void sampleConfiguration(std::vector<double>& q);

    RRTNode* takeStepsToSample(
        NodeTree* tree, 
        RRTNode* node, 
        const std::vector<double>& q);

    std::vector<dReal> calcDirVector(
        const std::vector<dReal>& v1, 
        const std::vector<dReal>& v2,
        const std::vector<double>& w) const;

    bool checkCollision(const std::vector<dReal>& config) const;

    void plotTrajectory(
        const float color[4], 
        std::vector<std::vector<double> >& path);

    void smoothPath();

    double pathLength(std::vector<std::vector<double> >& path);

    void executeTrajectory(std::vector<std::vector<double> >& p);

};

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "birrt" ) {
        return InterfaceBasePtr(new BiRRT(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("BiRRT");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

BiRRT::BiRRT(EnvironmentBasePtr penv, std::istream& ss)
    : ModuleBase(penv)
    , stree(NodeTree())
    , gtree(NodeTree())
    , stepSize(0.0)
    , smoothIter(NUM_SMOOTHING_ITER)
    , randseed(time(NULL))
{
    RegisterCommand("SetGoal",boost::bind(&BiRRT::SetGoal,this,_1,_2),
                    "Sets the goal config for motion planner");
    RegisterCommand("SetStart",boost::bind(&BiRRT::SetStart,this,_1,_2),
                    "Sets the start config for motion planner");
    RegisterCommand("SetStepSize",boost::bind(&BiRRT::SetStepSize,this,_1,_2),
                    "Sets the size of the step to take towards a sample config ");
    RegisterCommand("SetSmoothIteration",boost::bind(&BiRRT::SetSmoothIteration,this,_1,_2),
                    "Sets how many iterations of smoothing should be applied");
    RegisterCommand("SetRandomSeed",boost::bind(&BiRRT::SetRandomSeed,this,_1,_2),
                    "Sets the random generators with the seed value passed");
    RegisterCommand("printClass",boost::bind(&BiRRT::printClass,this,_1,_2),
                    "Debug function for printing internal members of rrt class");
    RegisterCommand("Init",boost::bind(&BiRRT::Init,this,_1,_2),
                    "All prep work needed done here before calling run");
    RegisterCommand("resetTrees",boost::bind(&BiRRT::resetTrees,this,_1,_2),
                    "Clears all the nodes in the NodeTree except start");
    RegisterCommand("run",boost::bind(&BiRRT::run,this,_1,_2),
                    "Run RRT Connect algorithm");

    normalizeWeights();

    std::random_device rd;
    srand(randseed);
    gen.seed(rd());
}

bool BiRRT::SetGoal(std::ostream& sout, std::istream& sinput)
{
    std::string sval;
    goalQ.clear();
    while (sinput >> sval)
    {
        goalQ.push_back(atof(sval.c_str()));
    }
    RRTNode* root = new RRTNode(goalQ);
    gtree.add(root);
    return true;
}

bool BiRRT::SetStart(std::ostream& sout, std::istream& sinput)
{
    std::string sval;
    startQ.clear();
    while (sinput >> sval)
    {
        startQ.push_back(atof(sval.c_str()));
    }
    RRTNode* root = new RRTNode(startQ);
    stree.add(root);

    return true;
}

bool BiRRT::SetStepSize(std::ostream& sout, std::istream& sinput)
{
    std::string sval;
    sinput >> sval;
    stepSize = atof(sval.c_str());
    return true;
}

bool BiRRT::SetSmoothIteration(std::ostream& sout, std::istream& sinput)
{
    std::string sval;
    sinput >> sval;
    smoothIter = atof(sval.c_str());
    return true;
}

bool BiRRT::SetRandomSeed(std::ostream& sout, std::istream& sinput)
{
    std::string sval;
    sinput >> sval;
    randseed = atoi(sval.c_str());

    srand(randseed);
    gen.seed(randseed);

    return true;
} 

template <class T>
void BiRRT::printVector(std::string s, const std::vector<T>& v) const
{
    std::cout << s << " ";
    for (const auto& val : v)
    {
        std::cout << val << " ";
    }
    std::cout << std::endl;
}

bool BiRRT::Init(std::ostream& sout, std::istream& sinput)
{
    std::cout << "Initializing Bi-Directional RRT-Connect Algorithm..." << std::endl;

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

bool BiRRT::printClass(std::ostream& sout, std::istream& sinput)
{
    printVector("Start Config:", startQ);
    printVector("Goal Config :", goalQ);
    std::cout << "Step Size   : " << stepSize << std::endl;
    std::cout << "# Smooths   : " << smoothIter << std::endl;
    std::cout << "Random Seed : " << randseed << std::endl;
    printVector("Lower Limits:", llim);
    printVector("Upper Limits:", ulim);
    return true;
}

bool BiRRT::resetTrees(std::ostream& sout, std::istream& sinput)
{
    stree.clear();
    gtree.clear();
    RRTNode* sroot = new RRTNode(startQ);
    RRTNode* groot = new RRTNode(goalQ);
    stree.add(sroot);
    gtree.add(groot);
    
    // Clear any existing points in environment
    ghandle.clear();

    return true;
}

bool BiRRT::run(std::ostream& sout, std::istream& sinput)
{
    time_t startTime(time(NULL)), endTime;
    NodeTree* growT = &stree;
    NodeTree* reachT = &gtree;
    Status code = INVALID;
    unsigned i = 1;

    while (code == INVALID)
    {
        if (i % 50 == 0)
        {
            std::cout << "Processed " << i << " samples | " 
                      << "Nodes: " << stree.getSize() + gtree.getSize() <<  std::endl;
        }

        // Select tree to grow based on which tree has less nodes
        growT  = (stree.getSize() <= gtree.getSize()) ? &stree : &gtree;
        reachT = (stree.getSize() <= gtree.getSize()) ? &gtree : &stree;

        // Sample configuration and grow from the grow tree
        std::vector<double> sample(startQ.size(), 0.0);
        sampleConfiguration(sample);
        RRTNode* nearestNode = growT->nearestNeighbor(sample);
        RRTNode* target = takeStepsToSample(growT, nearestNode, sample);

        if (target)
        {
            // Calculate nearest node from other tree and grow
            nearestNode = reachT->nearestNeighbor(target->getConfig());
            RRTNode* nodeAdded = takeStepsToSample(reachT, nearestNode, target->getConfig());

            // If the node reached is equal to the target, trees have met
            if (nodeAdded != NULL && (nodeAdded->getConfig() == target->getConfig()))
            {
                endTime = time(NULL);
                code = SUCCESS;

                // Extract path from both trees and merge
                std::vector<std::vector<double> > otherhalf;
                if (growT->getRoot()->getConfig() == startQ)
                {
                    path = growT->getPath(target);
                    std::reverse(path.begin(), path.end());
                    otherhalf = reachT->getPath(nodeAdded);
                }
                else
                {
                    path = reachT->getPath(nodeAdded);
                    std::reverse(path.begin(), path.end());
                    otherhalf = growT->getPath(target);
                }

                // The node at which both trees meet is added twice; hence +1
                path.insert(path.end(), otherhalf.begin()+1, otherhalf.end());

                sout << endTime - startTime << " ";
                sout << stree.getSize() + gtree.getSize() - 2 <<  " ";
                sout << i << " ";
                break;
            }
        }

        endTime = time(NULL);
        if ((endTime - startTime) > 3600)
        {
            code = TIME_OUT;
            break;
        }

        i++;
    }

    printReport(code, startTime, endTime, i);

    if (code == SUCCESS)
    {
        plotTrajectory(red, path);
        double pathDist = pathLength(path);
    
        // Time path smoothing
        // startTime = time(NULL);
        // smoothPath();
        // endTime = time(NULL);
        // std::cout << "\nSmooth Time : " << endTime - startTime << std::endl;
    
        // plotTrajectory(blue, smoothedPath);
        // double smoothedPathDist = pathLength(smoothedPath);

        // executeTrajectory(smoothedPath);
    
        // sout << smoothedPathDist << " ";
        // sout << endTime - startTime << " ";
        sout << pathDist << " ";
    }

    return true;
}

void BiRRT::printReport(
    Status status, 
    time_t startTime, 
    time_t endTime, 
    int samples) const
{
    switch(status)
    {
        case SUCCESS :
            std::cout << "Goal configuration reached!\n";
            break;
        case TIME_OUT :
            std::cout << "==== Sorry Timed Out ====" << std::endl;
            break;
        default :
            std::cout << "=== Unrecognized Exit ===" << std::endl;
    }

    std::cout << "Time        : " << endTime - startTime << std::endl;
    std::cout << "STree Nodes : " << stree.getSize() << std::endl;
    std::cout << "GTree Nodes : " << gtree.getSize() << std::endl;
    std::cout << "Samples     : " << samples << std::endl;
}

void BiRRT::sampleConfiguration(std::vector<double>& q)
{
    do 
    {
        for (unsigned i = 0; i < q.size(); ++i)
        {
            q[i] = dists[i](gen);
        }
    } while (checkCollision(q));
}

RRTNode* BiRRT::takeStepsToSample(
    NodeTree* tree, 
    RRTNode* nn, 
    const std::vector<double>& sample)
{
    std::vector<double> new_q(sample.size(), 0.0);
    std::vector<double> vec = calcDirVector(nn->getConfig(), sample, noWristRollW);
    RRTNode* target = NULL;

    RRTNode* currNode = nn;
    while (true)
    {
        double dist = calcEuclidianDist(
            currNode->getConfig(), sample, noWristRollW);
        if (dist <= stepSize)
        {
            new_q = sample;
        }
        else
        {
            new_q = currNode->getConfig() + vec; 
        }

        if (!checkCollision(new_q))
        {
            RRTNode* newNode = new RRTNode(new_q, currNode);
            tree->add(newNode);
            currNode = newNode;
            target = newNode;

            if (sample == new_q)
                break;
        }
        else
            break;
    }

    return target;
}

std::vector<dReal> BiRRT::calcDirVector(
    const std::vector<dReal>& v1, 
    const std::vector<dReal>& v2,
    const std::vector<double>& w) const
{
    std::vector<dReal> dir (v2.size(), 0.0);
    double dist = calcEuclidianDist(v1, v2, w);
    
    for (unsigned i = 0; i < v2.size(); ++i)
    {
        dir[i] = (v2[i]-v1[i]) / dist * stepSize;
    }
    return dir;
}

bool BiRRT::checkCollision(const std::vector<dReal>& config) const
{
    probot->SetActiveDOFValues(config);

    bool ret;
    ret = GetEnv()->CheckCollision(RobotBaseConstPtr(probot));
    ret = probot->CheckSelfCollision() || ret;
    return ret;
}

void BiRRT::plotTrajectory(const float color[4], std::vector<std::vector<double> >& path) 
{
    // Set robot to that configuration
    std::vector<float> p(3,0.0);

    for (auto it = path.begin(); it != path.end(); ++it)
    {
        probot->SetActiveDOFValues(*it);
        // Get the position of the end effector
        Transform endEffector = probot->GetLinks()[L_END_EFFECTOR_LINK_IDX]->GetTransform();
        p[0] = endEffector.trans.x;
        p[1] = endEffector.trans.y;
        p[2] = endEffector.trans.z;
        ghandle.push_back(GetEnv()->plot3(&p[0],1,12,5,color,0));
    }
}

void BiRRT::smoothPath()
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
        std::vector<std::vector<dReal> > shorterPath;

        std::vector<double> w(n1.size(), 1.0);
        std::vector<dReal> vec = calcDirVector(n1, n2, w);
        
        while (true)
        {
            // Take a step forward to node 2
            n1 = n1 + vec;

            // Check that position is valid
            if (checkCollision(n1))
            {
                shorterPath.clear();
                break;
            }
            else
            {
                shorterPath.push_back(n1);
            }

            // If sufficiently close to node 2 stop
            double dist = calcEuclidianDist(n1,n2);
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

double BiRRT::pathLength(std::vector<std::vector<double> >& path)
{
    // There should be at least two elements in the path
    assert(path.size() > 1);

    double dist = 0.0;
    for (unsigned i = 0; i < path.size()-1; ++i)
    {
        dist += calcEuclidianDist(path[i],path[i+1]);
    }
    return dist;    
}

void BiRRT::executeTrajectory(std::vector<std::vector<double> >& p)
{
    probot->SetActiveDOFValues(startQ);

    TrajectoryBasePtr traj = RaveCreateTrajectory(GetEnv(), "");
    ConfigurationSpecification conspec = probot->GetActiveConfigurationSpecification("linear");
    conspec.AddDeltaTimeGroup();
    traj->Init(conspec);

    int i = 0;
    std::vector<dReal> point;
    for (auto it = p.begin(); it != p.end(); ++it)
    {
        point = *it;
        point.push_back(i*0.01);
        traj->Insert(i, point, conspec, true);
        i++;
    }
    probot->GetController()->SetPath(traj);
}