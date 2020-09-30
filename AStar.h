#ifndef ASTAR_ASTAR_H
#define ASTAR_ASTAR_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

using Position = std::pair<int, int>;
#define x first
#define y second

class AStar {
public:
    Position boardSize;
    Position start;
    Position goal;
    std::vector<Position> directions = { { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
                                         { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 } };
    std::vector<Position> obstacles;

    struct Node {
        Position coordinates;
        unsigned int f,g,h;
        Node* parent;

        Node(Position coord_, unsigned int f_ = 0, Node* parent_ = nullptr);
    };

    AStar(Position start_, Position goal_, Position boardSize_, std::vector<Position> obstacles_);

    bool checkCollision(Position coord);

    bool isInvalid(Position coord);

    static Node* inSet(std::vector<Node*> set_, Position coord);

    unsigned int calcHeuristic(Position currNew_);

    void clearSet(std::vector<Node*> set_);

    std::vector<Position>findPath(bool diagonal);
};

#endif //ASTAR_ASTAR_H
