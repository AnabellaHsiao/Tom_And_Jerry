/*
	CSC D84 - Search Algorithms
	 
    - This covers Heuristic Search (A* based)
    - MiniMax search with alpha-beta pruning

	(c) F. Estrada, Updated Sep. 2025
*/

#ifndef __AI_SearchAlgorithms_headers

#define __AI_SearchAlgorithms_headers
#define size_X 32
#define size_Y 32
#define graph_size size_X*size_Y
#define loop_fraction .25

// Standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <math.h>
#include <string.h>

// Open GL headers - must be installed on the system's include dirs.
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#define IMAGE_OUTPUT 0		// Enable to output each frame to disk
#define DEBUG_DEPTH 0
#define __DEBUG

//************  USEFUL DATA STRUCTURES **********************
struct sprite{
	// Used to contain a single image - the image array does not
	// hold the image's size hence this little sprite structure.
	int sx;
	int sy;
	unsigned char *imgdata;
};
struct node{
	int index;//id basically
	int x; //x-axis
	int y; //y-axis
	int distance; //the current distance
	struct node *parent; // the parent of the current node
	struct node *child;
};


// ***********  GLOBAL DATA  ********************************
struct sprite framebuffer;							// Texture image for display
struct sprite maze;									// Buffer to hold empty maze
struct sprite catSprt,mouseSprt,cheeseSprt;			// Buffers for cat, cheese, and mouse images
double Graph[graph_size][4];						// Adjacency list for the graph representing the maze
int Path[graph_size][2];	        				// Array where we can store a path through the maze
double grid_value[size_X][size_Y];					// An array where to store values of interest for each grid cell (visiting order, or minimax cost)
int cats[10][2];									// Cat positions (x,y)
int cheese[10][2];									// Cheese positions (x,y)
int mouse[1][2];									// Mouse location
int n_cats;											// Number of cats
int n_cheese;										// Number of cheese chunks
int r_seed;											// Initial random seed
int s_mode; 										// Search mode  (0 - A*, 1 - A* no-kitty, 2 - MiniMax, 3 - MiniMax w. alpha/beta pruning
int md;												// maximum search depth for MiniMax
double cattitude;									// Cat smartness factor in [0,1]
int windowID;										// OpenGL Window ID
struct node *queue[graph_size];                     //our min queue for path
int found;

// ***********  FUNCTION HEADER DECLARATIONS ****************
// Search code functions
void findPath(int start_x, int start_y, int (*goalCheck)(int x, int y));
void Prim_MST(double AdjMat[graph_size][4]);
void randomMouse();
void moveAway(int pos[1][2], int tgt[1][2], int path[graph_size][2], int mode);
void catMoves(int idx, double smart);
int checkForMouse(int x, int y);
int checkForCheese(int x, int y);
int checkForCats(int x, int y);

// Open GL Initialization functions
void initGlut(const char* winName, int sizeX, int sizeY, int positionX, int positionY);

// Open GL callbacks for handling events in glut
void WindowReshape(int w, int h);
void WindowDisplay(void);
void kbHandler(unsigned char key, int x, int y);

// Image processing functions
void tweakSprite(struct sprite *buf, unsigned char xR, unsigned char xG, unsigned char xB);
void updateFrame(int path[graph_size][2]);
void overlaySprite(struct sprite *dest, struct sprite *src, int x, int y);
void renderMaze(struct sprite *buf, double AdjMat[graph_size][4],unsigned char R, unsigned char G, unsigned char B);
void line(struct sprite *buf, double x1, double y1, double x2, double y2, unsigned char R, unsigned char G, unsigned char B);
void block(struct sprite *buf, double x1, double y1, double x2, double y2, unsigned char R, unsigned char G, unsigned char B);
unsigned char *readPPMimage(const char *filename, int *sizx, int *sizy);
void imageOutput(unsigned char *im, int sx, int sy, const char *filename);

// Solution code functions
void Heuristic_Search(int (*heuristic)(int x, int y));
int H_cost(int x, int y);
int H_cost_nokitty(int x, int y);
double MiniMax(int cat_loc[10][2], int ncats, int cheese_loc[10][2], int ncheeses, int mouse_loc[1][2], int mode, double (*utility)(int cat_loc[10][2], int cheese_loc[10][2], int mouse_loc[1][2], int cats, int cheeses, int depth), int agentId, int depth, int maxDepth, double alpha, double beta);
double utility(int cat_loc[10][2], int cheese_loc[10][2], int mouse_loc[1][2], int ncats, int ncheeses, int depth);
struct node *initialization(int x,int y,struct node *parent,int distance);

#endif

