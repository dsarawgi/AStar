#include "AStar.h"

AStar::AStar(Position start_, Position goal_, Position boardSize_, std::vector<Position> obstacles_)
{
    start.x = start_.x; start.y = start_.y;
    goal.x = goal_.x; goal.y = goal_.y;
    boardSize.x = boardSize_.x; boardSize.y = boardSize_.y;
    obstacles = obstacles_;
}

AStar::Node::Node(Position coord_, unsigned int f_, Node *parent_)
{
    coordinates.x = coord_.x;
    coordinates.y = coord_.y;
    f = f_;
    parent = parent_;
}

bool AStar::checkCollision(Position coord)
{
    if(std::find(obstacles.begin(), obstacles.end(), coord) != obstacles.end())
        return true;
    else
        return false;
}

bool AStar::isInvalid(Position coord)
{
    if(coord.x < 0 || coord.x > (boardSize.x - 1) || coord.y < 0 || coord.y > (boardSize.y - 1))
        return true;
    else
        return false;

}

AStar::Node* AStar::inSet(std::vector<Node*> set_, Position coord)
{
    for (auto itr = set_.begin(); itr != set_.end(); itr++)
    {
        auto node_ = *itr;
        if( (node_->coordinates.x == coord.x) && (node_->coordinates.y == coord.y) )
            return node_;
    }
    return nullptr;
}

unsigned int AStar::calcHeuristic(Position currNew_)
{
    return (unsigned int)(10*sqrt((currNew_.x - goal.x)*(currNew_.x - goal.x) +
                                  (currNew_.y - goal.y)*(currNew_.y - goal.y)));
}

void AStar::clearSet(std::vector<Node*> set_)
{
    for(auto itr = set_.begin(); itr != set_.end();)
    {
        delete *itr;
        itr = set_.erase(itr);
    }
}

std::vector<Position> AStar::findPath(bool diagonal)
{
    int numDirs;
    if(diagonal)
        numDirs = directions.size();
    else
        numDirs = directions.size()/2;

    std::vector<Node*> closedSet, openSet;
    openSet.push_back(new Node(start, 0));
    Node* curr = nullptr;

    while(!openSet.empty()) {
        auto curr_it = openSet.begin();
        curr = *curr_it;

        for(auto itr = openSet.begin(); itr != openSet.end(); itr++)
        {
            auto node = *itr;
            curr->f = curr->g + curr->h;
            node->f = node->g + node->h;
            if(node->f < curr->f)
            {
                curr = node;
                curr_it = itr;
            }
        }

        closedSet.push_back(curr);
        openSet.erase(curr_it);

        if(curr->coordinates.x == goal.x && curr->coordinates.y == goal.y)
            break;

        for(int pos = 0; pos < numDirs; pos++)
        {
            int xNew = curr->coordinates.x + directions[pos].x;
            int yNew =  curr->coordinates.y + directions[pos].y;
            Position currNew = {xNew,yNew};
            Node *successor = new Node(currNew, 0, curr);
            auto node_ = inSet(openSet, currNew);
            unsigned int gNew = curr->g + ((pos < 4 ) ? 10 : 14);
            successor->h = calcHeuristic(currNew);
            successor->g = gNew;
            successor->f = successor->g + successor->h;

            if(checkCollision(currNew) || isInvalid(currNew) || inSet(closedSet, currNew)) continue;

            if(!node_)
            {
                successor->coordinates = currNew;
                successor->parent = curr;
                openSet.push_back(successor);
            }
            else
            {
                unsigned int f_ = node_->h + gNew;
                if(successor->f < (node_)->f)
                {
                    (node_)->g = successor->g;
                    (node_)->f = successor->f;
                    (node_)->parent = curr;
                }
            }
        }
    }

    std::vector<Position> path;

    if(curr->coordinates.x == goal.x && curr->coordinates.y == goal.y)
    {
        while(curr != nullptr)
        {
            path.push_back(curr->coordinates);
            curr = curr->parent;
        }
    }

    clearSet(openSet);
    clearSet(closedSet);

    return path;
}