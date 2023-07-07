/*
Pseudocode of A* Algorithm

Make an OPEN LIST containing starting node

If it reaches the destination node:
    Make a CLOSED EMPTY LIST 

If it does not reach the destination node, then consider a node with the lowest F-score in the OPEN LIST
We are finished

Else :
    Put the current node in the list and check its neighbors

For each neighbor of the current node :
    If the neighbor has a lower G value than the current node and is in the closed list:
        Replace neighbor with this new node as the neighbor’s parent

    Else If (current G is lower and neighbor is in the OPEN LIST):
        Replace neighbor with the lower g value and change the neighbor’s parent to the current node.

    Else If the neighbor is not in both lists:
        Add it to the OPEN LIST and set its G
*/

/** For OPEN LIST ===========================
 * priority_queue<int> variableName;  
Queue Functions:
    empty() – Checks whether the priority queue is empty 
    size() – Returns the number of elements
    top() – Returns a reference to the top most element from the priority queue container
    push(g) – Insert the element "g" inside the priority queue
    pop() – Deletes the element from the priority queue according to the highest priority 
    emplace() - insert a new element at the top of the priority queue
    swap() - swap the elements of the priority queue with another priority queue
=========================================== */

#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
using namespace std;

const int n = 60; // horizontal size
const int m = 60; // vertical size
static int map[n][m];
static int closed_nodes_map[n][m]; // tried-out nodes
static int open_nodes_map[n][m]; // not-yet-tried nodes
static int dir_map[n][m]; // map of directions

const int dir = 8; 
static int dx[dir] = {1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir] = {0, 1, 1, 1, 0, -1, -1, -1};

class node{  
    int xPos; int yPos; // current position
    int level; // total distance already traveled to reach node
    int priority; //=level + remaining distance estimate => smaller is higher priority
    
    public:
        node(int xp, int yp, int d, int p)
            {xPos = xp; yPos = yp; level = d; priority = p;}

        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        int getLevel() const {return level;}
        int getPriority() const {return priority;}
        //??????????
        void updatePriority(const int & xGoal, const int & yGoal){
            priority = level + estimate(xGoal, yGoal) * 10;
        }

        //give better priority to go strait instead of diagonally
        void nextLevel(const int & i){//i: direction
            level += (dir == 8? (i % 2 == 0? 10 : 14) : 10);
        }

        //estimation function for the remaining distance to goal
        const int & estimate(const int & xGoal, const int & yGoal) const{
            static int xd, yd, d;
            xd = xGoal - xPos;
            yd = yGoal - yPos;
            
            d = static_cast<int>(sqrt(xd * xd + yd * yd)); // Euclidian Distance
            //d=abs(xd)+abs(yd); // Manhattan distance
            //d=max(abs(xd), abs(yd)); // Chebyshev distance
            return (d);
        }
};

// Determine priority
bool operator < (const node & a, const node & b){
    return a.getPriority() > b.getPriority();
}

// ==================================
// A-Star algorithm
string pathFind(const int & xStart, const int & yStart, const int & xGoal, const int & yGoal)
{   
    // priority_queue<int> variableName;  
    static priority_queue<node> pq[2];// open list of (not-yet-tried) nodes
    static int pqi; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqi = 0; //index

    // reset the node maps
    for(y = 0; y < m; y++){
        for (x = 0; x < n; x++){
            closed_nodes_map[x][y] = 0;
            open_nodes_map [x][y] = 0;
        }
    } 

    // create the start node and push into the OPEN LIST
    n0 = new node(xStart, yStart, 0, 0);
    n0 -> updatePriority(xGoal, yGoal);
    pq[pqi].push(*n0); 
    open_nodes_map[x][y] = n0 -> getPriority(); // mark it on the open nodes map

    // A* search work ==================== 
    while(!pq[pqi].empty()){
        //get the current node w/ the highest priority from the OPEN LIST
        n0 = new node(pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), pq[pqi].top().getLevel(), pq[pqi].top().getPriority());
        x = n0 -> getxPos();
        y = n0 -> getyPos();
        pq[pqi].pop(); // remove the node from the OPEN LIST
        open_nodes_map[x][y] = 0;
        closed_nodes_map[x][y] = 1; //mark it on closed nodes map

        // quit searching when reaching the goal
        if(x == xGoal && y == yGoal){
            string path_dir = "";
            while(!(x == xStart && y == yStart)){
                j = dir_map[x][y];
                c = '0' + (j + dir/2) % dir;
                path_dir = c + path_dir;
                x += dx[j];
                y += dy[j];
            }
            
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();
            printf("temp route: %s", path_dir.c_str());
            return path_dir;
        }

        // generate moves (child nodes) in all possible directions
        for(i = 0; i < dir; i++){
            xdx = x + dx[i]; ydy = y + dy[i];

            if(!(xdx < 0 || xdx > n - 1 || ydy < 0 || ydy > m - 1 || map[xdx][ydy] == 1 || closed_nodes_map[xdx][ydy] == 1)){
                // generate a child node
                m0 = new node(xdx, ydy, n0 -> getLevel(), n0 -> getPriority());
                m0 -> nextLevel(i);
                m0 -> updatePriority(xGoal, yGoal);
                
                // if it is not in OPEN list then add into that
                if(open_nodes_map[xdx][ydy] == 0){
                    open_nodes_map[xdx][ydy] = m0 ->getPriority();
                    pq[pqi].push(*m0); // add nodes into OPEN list and arrange based on priority
                    dir_map[xdx][ydy] = (i + dir/2) % dir; // mark its parent node direction
                    
                }
                else if(open_nodes_map[xdx][ydy] > m0 -> getPriority()){
                    open_nodes_map[xdx][ydy] = m0 -> getPriority(); // update priority
                    dir_map[xdx][ydy] = (i + dir/2) % dir; // update direction

                    // replace node by empty one pq to the other one except the node to be replaced will be ignored and new node will bee pushed in instead
                    while(!(pq[pqi].top().getxPos() == xdx && pq[pqi].top().getyPos() == ydy)){
                        pq[1 - pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop();//remove the wanted node
                    //empty the larger size pq to the smaller node
                    if(pq[pqi].size() > pq[1 - pqi].size()) {pqi = 1 - pqi;}
                    while(!pq[pqi].empty()){
                        pq[1 - pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi = 1 - pqi;
                    pq[pqi].push(*m0); // add the better node instead
                    // cout<<pq<<endl;
                }
                else delete m0; //garbage collection
            }
        }
        delete n0; //garbage collection
    }
    return "";// no route found
}

    // ==================================
/*
map[x][y] 
0: empty    -> print "."
1: obstacle -> print "0"
2: start    -> print "S"
3: route    -> print "R"
4: goal     -> print "G"
*/

int main(){
    srand(time(NULL));
    // create empty map
    for(int y = 0; y < m; y++){
        for(int x = 0; x < n; x++){
            map[x][y] = 0;
        }
    }
    // create obstacle
    for(int x = n/8; x < n-5; x++){
        map[x][m/2] = 1;
    }

    for(int x = n- n/2 - 5; x < n; x++){
        map[x][m/5] = 1;
    }

    for(int x = n/2; x < n; x++){
        map[x][m/4*3] = 1;
    }

    for(int y = m/8; y < m-5; y++){
        map[n/3][y] = 1;
    }

    int xA, yA, xB, yB; // start and goal locations
    xA = 0; yA = 0; xB = n - 1; yB = m - 1;

    printf("Map size (X, Y): (%d, %d) cells\n", n, m);
    printf("Start: (%d, %d) \t -> \t", xA, yA);
    printf("Goal: (%d, %d) \n", xB, yB);
    // get the route
    clock_t start = clock();
    // search ==============================
    string route_dir = pathFind(xA, yA, xB, yB);
    if(route_dir == "") printf("An empty route generated!\n");
    clock_t end = clock();
    
    printf("\nTime elapsed (ms): %d \n", end - start);
    // printf("Route: %s", route_dir.c_str());
    printf("size: %d \n", route_dir.length());
    // follow the route on the map and display it
    if (route_dir.length() > 0){
        int j; char c;
        int x = xA; int y = yA;

        map[x][y] = 2;

        for(int i = 0; i < route_dir.length(); i++){
            c = route_dir.at(i); 
            j = atoi(&c); // convert string to int
            x = x + dx[j];
            y = y + dy[j];
            map[x][y] = 3;
            
        }
        map[x][y] = 4;

        // display the map with the route
        for(int y = 0; y < m; y++){
            for(int x = 0; x < n; x++){
                if (map[x][y] == 0)         printf(".");//empty
                else if (map[x][y] == 1)    printf("O");//obstacle
                else if (map[x][y] == 2)    printf("S");//start
                else if (map[x][y] == 3)    printf("R");//reach
                else if (map[x][y] == 4)    printf("G");//Goal
            }
            printf("\n");
        }
    }
    // getchar(); // wait for a (Enter) keypress  
    return(0);
}
