#ifndef RRT_HPP
#define RRT_HPP

#include <rrt-nodetree.hpp>
#include <iostream>

class RRT
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

public:
    RRT(const std::vector<float>& startq, std::vector<float>& goalq);

    void setNumSamples(const int n) { numSamples = n; }

    void setStepSize(const float s) { stepSize = s; }

    // The goal bias integer p is used to determine how often the goal should be
    // sampled. Every pth sample, the goal will be sampled.
    void setGoalBiasing(const int p) { goalBias = p; }

    // Returns true if RRT was able to successfully find the goal configuration
    // and false otherwise.
    bool run();

    std::vector<RRTNode* const> getPath() const;

private:
    std::vector<float> sampleConfiguration() const;
    
    // Attempts to take a step towards the sample configuration sampleQ from the
    // nearest node in the tree. If the configuration at the step is valid, the
    // function will return true. Otherwise if the configuration is in
    // CObstacle, the function will return false and newQ will be set to NULL.
    // Upon stepping successfully towards the sample config the value of the new
    // configuration will be returned in newQ. If the sampleQ given is the goal
    // configuration and it is reachable, a step size shorter than stepSize will
    // be taken and the goal configuration will be returned in newQ.
    bool stepToSample(const std::vector<float>& sampleQ,
                      std::vector<float>& newQ, 
                      RRTNode* const node) const;

};

RRT::RRT(const std::vector<float>& startq, std::vector<float>& goalq)
 : startQ(startq)
 , goalQ(goalq)
 , tree(NodeTree())
 , goal(NULL)
 , numSamples(0)
 , stepSize(0.0)
 , goalBias(10)
 , biasCounter(0)
{
    RRTNode* root = new RRTNode(startQ);
    tree.add(root);
}

bool RRT::run()
{
    for (int i = 0; i < numSamples; i++)
    {
        std::vector<float> sampleQ = sampleConfiguration();
        RRTNode* nearestNode = tree.nearestNeighbor(sampleQ);
        
        std::vector<float> newQ;
        if (stepToSample(sampleQ, newQ, nearestNode))
        {
            RRTNode* node = new RRTNode(newQ, nearestNode);
            tree.add(node);

            if (sampleQ == newQ)
            {
                std::cout << "Goal configuration reached!\n";
                goal = node;
                break;
            }
        }

    }
}

std::vector<float> RRT::sampleConfiguration() const
{
    return ;
}

std::vector<RRTNode* const> RRT::getPath() const
{
    std::vector<RRTNode* const> path;
    try
    {
        path = tree.getPath(goal);
    }
    catch(const std::invalid_argument& ia)
    {
        std::cerr << "Invalid argument: " << ia.what() << '\n';
    }
    return path;
}





#endif