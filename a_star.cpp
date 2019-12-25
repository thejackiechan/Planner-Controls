#include<iostream>
#include<limits>
#include<cmath>
#include<vector>
#include<queue>
#include "mainwindow.h" // fix
#include "ui_mainwindow.h"
#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>

using namespace std;

// Define a Node
class Node
{
    public:
       int x;
       int y;
       float cumCost;
       Node* prevNode;

       Node(int inX, int inY, float inCumCost, Node* inPrevNode = 0);
};

Node::Node(int inX, int inY, float inCumCost, Node* inPrevNode)
{
    x = inX;
    y = inY;
    cumCost = inCumCost;
    prevNode = inPrevNode;
}

std::vector<Node> getMotionModel()
{
    return { Node(1, 0, 1),
             Node(0, 1, 1),
             Node(-1, 0, 1),
             Node(0, -1, 1),
             Node(-1, -1, std::sqrt(2)),
             Node(-1, 1, std::sqrt(2)),
             Node(1, -1, std::sqrt(2)),
             Node(1, 1, std::sqrt(2)) };
}

std::vector<std::vector<int>> buildObsMap(std::vector<int> &obsX, std::vector<int> &obsY,
                                const int minX, const int maxX,
                                const int minY, const int maxY,
                                float resolution, float vehicleSize)
{
    int width = maxX - minX;
    int height = maxY - minY;

    std::vector<std::vector<int>> obsMap(height, std::vector<int> (width, 0));

    for(int i = 0; i < width; i++)
    {
        int x = minX + i;
        for(int j = 0; j < height; j++)
        {
            int y = minY + i;
            for(size_t k = 0; k < obsX.size(); k++)
            {
                float dist = std::sqrt(std::pow((obsX[k] - x), 2) + std::pow((obsY[k] - y), 2));
                if(dist <= vehicleSize / resolution)
                {
                    // there is an obstacle here
                    obsMap[i][j] = 1;
                    // draw rectangle?
                    break;
                }
            }
        }
    }
    return obsMap;
}

// Returns the Euclidean distance between two nodes
float computeHeuristic(Node* node1, Node* node2)
{
    return std::sqrt(std::pow(node1->x - node2->x, 2) + std::pow(node1->y - node2->y, 2));
}

bool isValidNode(Node* node, std::vector<std::vector<int>> &obsMap,
                 int minX, int maxX, int minY, int maxY)
{
    // check bounds
    if(node->x < minX || node->x > maxX || node->y < minY || node->y > maxY)
    {
        return false;
    }
    // check for obstacles
    else if(obsMap[node->x - minX][node->y - minY])
    {
        return false;
    }
    else
    {
        return true;
    }
}

void getFinalPath(Node* goalNode, std::vector<float> &xCoords, std::vector<float> &yCoords, float resolution)
{
    Node* node = goalNode;

    while(node->prevNode != 0)
    {
        node = node->prevNode;
        xCoords.push_back(resolution * node->x);
        yCoords.push_back(resolution * node->y);
        // draw?
    }
}

void runAStar(float startX, float startY, float goalX, float goalY,
              std::vector<float> &inObsX, std::vector<float> inObsY,
              float resolution, float vehicleSize)
{
    // Define start and goal nodes (integers)
    Node* startNode = new Node((int) std::round(startX / resolution), (int) std::round(startY / resolution), 0.0);
    Node* goalNode = new Node((int) std::round(goalX / resolution), (int) std::round(goalY / resolution), 0.0);

    // Initialize bounds
    int minX = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::min();
    int minY = std::numeric_limits<int>::max();
    int maxY = std::numeric_limits<int>::min();

    // Obtain new obstacles (integers)
    std::vector<int> obsX;
    std::vector<int> obsY;

    for(float ix:inObsX)
    {
        int x = (int) std::round(ix / resolution);
        obsX.push_back(x);

        minX = std::min(x, minX);
        maxX = std::max(x, maxX);
    }

    for(float iy:inObsY)
    {
        int y = (int) std::round(iy / resolution);
        obsY.push_back(y);

        minY = std::min(y, minY);
        maxY = std::max(y, maxY);
    }

    int width = maxX - minX;
    int height = maxY - minY;

    // visualization?

    // Create a visited map (1 = visited, 0 else), cost map, and build obstacle map
    std::vector<std::vector<int>> visitedMap(height, std::vector<int>(width, 0));
    std::vector<std::vector<float>> costMap(height, std::vector<float>(width, std::numeric_limits<float>::max()));
    std::vector<std::vector<int>> obsMap = buildObsMap(obsX, obsY, minX, maxX, minY, maxY, resolution, vehicleSize);
    std::vector<Node> motions = getMotionModel();

    costMap[startNode->x][startNode->y] = 0;

    struct compareNodes
    {
       bool operator()( const Node* node1, const Node* node2 ) const
       {
        return node1->cumCost > node2->cumCost;
       }
    };

    // Initialize A*
    std::priority_queue<Node*, std::vector<Node*>, compareNodes> pq;

    pq.push(startNode);

    while(true)
    {
        Node* node = pq.top();
        pq.pop();

        // check if node has been visited
        if(visitedMap[node->x - minX][node->y - minY]) // returns bool
        {
            delete node;
            continue;
        }
        else
        {
            visitedMap[node->x - minX][node->y - minY] = 1;
        }

        // check if reached goal
        if(node->x == goalNode->x && node->y == goalNode->y)
        {
            goalNode->cumCost = node->cumCost;
            goalNode->prevNode = node;
            break;
        }

        // explore neighboring nodes
        for(size_t i = 0; i < motions.size(); i++)
        {
            Node* newNode = new Node(node->x + motions[i].x,
                                     node->y + motions[i].y,
                                     costMap[node->x][node->y] +
                                     motions[i].cumCost +
                                     computeHeuristic(node, goalNode),
                                     node);

            if(!isValidNode(newNode, obsMap, minX, maxX, minY, maxY))
            {
                delete newNode;
                continue;
            }

            if(visitedMap[newNode->x - minX][newNode->y - minY])
            {
                delete newNode;
                continue;
            }

            // update costMap
            if(costMap[node->x][node->y] + motions[i].cumCost < costMap[newNode->x][newNode->y])
            {
                costMap[newNode->x][newNode->y] = costMap[node->x][node->y] + motions[i].cumCost;
                pq.push(newNode); // add only if the cost decreases
            }
        }
    }

    // get final path
    std::vector<float> xCoords;
    std::vector<float> yCoords;
    getFinalPath(goalNode, xCoords, yCoords, resolution);
    delete startNode;
    delete goalNode;
}

int main()
{
    float startX = 10.0;
    float startY = 10.0;
    float goalX = 50.0;
    float goalY = 50.0;

    float gridSize = 1.0;
    float vehicleSize = 1.0;

    std::vector<float> obsX;
    std::vector<float> obsY;

    // add edges
    for(float i = 0; i < 60; i++)
    {
        obsX.push_back(i);
        obsY.push_back(60.0);
    }

    for(float i = 0; i < 60; i++)
    {
        obsX.push_back(60.0);
        obsY.push_back(i);
    }

    for(float i = 0; i < 61; i++)
    {
        obsX.push_back(i);
        obsY.push_back(60.0);
    }

    for(float i = 0; i < 61; i++)
    {
        obsX.push_back(0.0);
        obsY.push_back(i);
    }

    for(float i = 0; i < 40; i++)
    {
        obsX.push_back(20.0);
        obsY.push_back(i);
    }

    for(float i = 0; i < 40; i++)
    {
        obsX.push_back(40.0);
        obsY.push_back(60.0 - i);
    }

    runAStar(startX, startY, goalX, goalY, obsX, obsY, gridSize, vehicleSize);
    return 0;
}






