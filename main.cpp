#include <iostream>
#include <GL/glut.h>
#include "AStar.h"

#define WINDOW_SIZE 500 // for opengl representation

std::vector<std::pair<int,int>> obstacles_ = {{1,1},{1,0}, {3,3}, {3,4}, {7,7},{7,9},{8,8}}; // specify obstacles
std::vector<std::pair<int,int>> path;
std::pair<int,int> start_ = {0,0}; // start node
std::pair<int,int> goal_= {8,7}; // goal node
std::pair<int,int> boardSize_ = {10,10}; // grid size - (numRows, numCols)
bool diagonal = 0; // diagonal = 0 to turn off diagonal movement, = 1 to turn on diagonal movement

void drawGrid()
{
    glClear(GL_COLOR_BUFFER_BIT);
    double width = WINDOW_SIZE/boardSize_.first;

    // draw grid lines
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3f(1.0, 0.0, 0.0); // red
    for(int i=0;i<boardSize_.first;i++)
    {
        for(int j=0;j<boardSize_.second;j++)
        {
            glBegin(GL_POLYGON);
            glVertex3f(width*(i-boardSize_.first), width*(j-boardSize_.second), 0.0);
            glVertex3f(width*(i-boardSize_.first+1), width*(j-boardSize_.second), 0.0);
            glVertex3f(width*(i-boardSize_.first+1), width*(j-boardSize_.second+1), 0.0);
            glVertex3f(width*(i-boardSize_.first), width*(j-boardSize_.second+1), 0.0);
            glEnd();
        }
    }

    // draw obstacles
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor3f(1.0, 1.0, 1.0); // white
    for(int i=0;i<obstacles_.size();i++)
    {
        glBegin(GL_POLYGON);
        glVertex3f(width*(obstacles_[i].first-boardSize_.first), width*(obstacles_[i].second-boardSize_.second), 0.0);
        glVertex3f(width*(obstacles_[i].first-boardSize_.first+1), width*(obstacles_[i].second-boardSize_.second), 0.0);
        glVertex3f(width*(obstacles_[i].first-boardSize_.first+1), width*(obstacles_[i].second-boardSize_.second+1), 0.0);
        glVertex3f(width*(obstacles_[i].first-boardSize_.first), width*(obstacles_[i].second-boardSize_.second+1), 0.0);
        glEnd();
    }

    // draw start node
    glColor3f(0.0, 0.0, 1.0); // blue
    glPointSize(10.0);
    glBegin(GL_POINTS);
    glVertex3f (width*(start_.first-boardSize_.first) + (width/2), width*(start_.second-boardSize_.first) + (width/2), 0.0);
    glEnd();

    // draw goal node
    glColor3f(0.0, 1.0, 0.0); // green
    glBegin(GL_POINTS);
    glVertex3f (width*(goal_.first-boardSize_.first) + (width/2), width*(goal_.second-boardSize_.first) + (width/2), 0.0);
    glEnd();

    // draw the path
    glColor3f(1.0, 1.0, 0.0); // yellow
    glLineWidth(2.0);
    for(int i = path.size() - 1; i>0; i--)
    {
        glBegin(GL_LINE_STRIP);
        glVertex3f(width*(path[i].first-boardSize_.first) + (width/2), width*(path[i].second-boardSize_.first) + (width/2), 0.0);
        glVertex3f(width*(path[i-1].first-boardSize_.first) + (width/2), width*(path[i-1].second-boardSize_.first) + (width/2), 0.0);
        glEnd();

        if( i != path.size() - 1)
        {
            glBegin(GL_POINTS);
            glVertex3f(width*(path[i].first-boardSize_.first) + (width/2), width*(path[i].second-boardSize_.first) + (width/2), 0.0);
            glEnd();
        }
    }

    glFlush();
}

int main(int argc, char **argv) {

    AStar astar(start_, goal_, boardSize_, obstacles_); // start, goal, worldSize

    if(astar.checkCollision(goal_))
    {
        std::cout<<"Goal coordinates coincide with the obstacles. Pick another goal!";
        return 0;
    }
    if(astar.isInvalid(goal_))
    {
        std::cout<<"Goal coordinates are invalid. Pick a goal within the board size!";
        return 0;
    }
    if(astar.checkCollision(start_))
    {
        std::cout<<"Start coordinates coincide with the obstacles. Pick another start!";
        return 0;
    }
    if(astar.isInvalid(start_))
    {
        std::cout<<"Start coordinates are invalid. Pick a start within the board size!";
        return 0;
    }

    path = astar.findPath(diagonal);
    if (path.empty())
        std::cout<<"Could not find a path!!";
    else
    {
        std::cout << "Path found : ";
        for (int i=path.size()-1; i >= 0; i--)
        {
            std::cout << path[i].x << " " << path[i].y;

            if(i > 0)
                std::cout<<", ";
        }
    }

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE);
    glutInitWindowSize(WINDOW_SIZE, WINDOW_SIZE);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("A Star");
    glOrtho(-WINDOW_SIZE, 0, -WINDOW_SIZE, 0, 1.0, -1.0);
    glutDisplayFunc(drawGrid);
    glutMainLoop();

    return 0;
}

