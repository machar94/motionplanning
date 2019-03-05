#include <rrt-nodetree.hpp>


void NodeTree::add(RRTNode* const node)
{
    nodes.push_back(node);
}

void NodeTree::del(RRTNode* const node) 
{
    delete node;
    nodes.erase(std::remove(nodes.begin(), nodes.end(), node), nodes.end());
}

std::vector<RRTNode* const> NodeTree::getPath(RRTNode* const node) const
{
    if (node == NULL)
    {
        throw std::invalid_argument("Cannot get path from NULL");
    }
    
    auto curr = node;
    std::vector<RRTNode* const> path;
    
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
    
    if (nodes.empty())
    {
        throw std::out_of_range("Attempting to index into empty tree!");
    }

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
    for (int i = 0; i < q1.size(); i++)
    {
        sum += w[i]*(q1[i] - q2[i])*(q1[i] - q2[i]);
    }
    return sqrtf(sum);
}