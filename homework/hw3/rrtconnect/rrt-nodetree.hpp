#ifndef RRT_NODETREE_HPP
#define RRT_NODETREE_HPP

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <math.h>
#include <assert.h>

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
    std::vector<RRTNode* const> getPath(RRTNode* const node) const;

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

#endif