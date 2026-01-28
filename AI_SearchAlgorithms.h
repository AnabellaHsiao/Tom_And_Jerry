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
#define graph_size size_X *size_Y
#define loop_fraction .25

// Standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

// Open GL headers - must be installed on the system's include dirs.
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#define IMAGE_OUTPUT 0 // Enable to output each frame to disk
#define DEBUG_DEPTH 0
#define __DEBUG

//************  USEFUL DATA STRUCTURES **********************
struct sprite
{
	// Used to contain a single image - the image array does not
	// hold the image's size hence this little sprite structure.
	int sx;
	int sy;
	unsigned char *imgdata;
};
struct node
{
	int id;		  // id basically
	int x;		  // x-axis
	int y;		  // y-axis
	int distance; // the current distance
	int predecessor;
	struct node *child;
};

// Priority queue structure
typedef struct min_heap
{
	int size;		  // current number of elements in heap
	int capacity;	  // maximum number of elements heap can hold
	struct node *arr; // array of nodes in the heap
	int *indexMap;	  // maps index to its position in the heap array i.e. indexmap[id] = position of node with ID id in array arr
} MinHeap;

// ***********  GLOBAL DATA  ********************************
struct sprite framebuffer;					  // Texture image for display
struct sprite maze;							  // Buffer to hold empty maze
struct sprite catSprt, mouseSprt, cheeseSprt; // Buffers for cat, cheese, and mouse images
double Graph[graph_size][4];				  // Adjacency list for the graph representing the maze
int Path[graph_size][2];					  // Array where we can store a path through the maze
double grid_value[size_X][size_Y];			  // An array where to store values of interest for each grid cell (visiting order, or minimax cost)
int cats[10][2];							  // Cat positions (x,y)
int cheese[10][2];							  // Cheese positions (x,y)
int mouse[1][2];							  // Mouse location
int n_cats;									  // Number of cats
int n_cheese;								  // Number of cheese chunks
int r_seed;									  // Initial random seed
int s_mode;									  // Search mode  (0 - A*, 1 - A* no-kitty, 2 - MiniMax, 3 - MiniMax w. alpha/beta pruning
int md;										  // maximum search depth for MiniMax
double cattitude;							  // Cat smartness factor in [0,1]
int windowID;								  // OpenGL Window ID

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
int getIndexFromXY(int x, int y);

// Priority Queue functions
/* Returns the node with minimum priority in minheap 'heap'.
 * Precondition: heap is non-empty
 */
struct node getMin(MinHeap *heap);
/* Removes and returns the node with minimum priority in minheap 'heap'.
 * Precondition: heap is non-empty
 */
struct node extractMin(MinHeap *heap);
/* Inserts a new node with priority 'priority' and ID 'id' into minheap 'heap'.
 * Precondition: 'id' is unique within this minheap
 *               0 <= 'id' < heap->capacity
 *               heap->size < heap->capacity
 */
void insert(MinHeap *heap, int distance, int id, int x, int y, int predecessor, struct node *child);

/* Returns priority of the node with ID 'id' in 'heap'.
 * Precondition: 'id' is a valid node ID in 'heap'.
 */
int getPriority(MinHeap *heap, int id);

/* Sets priority of node with ID 'id' in minheap 'heap' to 'newPriority', if
 * such a node exists in 'heap' and its priority is larger than
 * 'newPriority', and returns True. Has no effect and returns False, otherwise.
 * Note: this function bubbles up the node until the heap property is restored.
 */
bool decreaseDistance(MinHeap *heap, int id, int newDistance);

/* Prints the contents of this heap, including size, capacity, full index
 * map, and, for each non-empty element of the heap array, that node's ID and
 * priority. */
void printHeap(MinHeap *heap);

/* Returns a newly created empty minheap with initial capacity 'capacity'.
 * Precondition: capacity >= 0
 */
MinHeap *newHeap(int capacity);
/* Frees all memory allocated for minheap 'heap'.
 */

void deleteHeap(MinHeap *heap);
bool isValidIndex(MinHeap *heap, int maybeIdx);
int rightIdx(MinHeap *heap, int nodeIndex);
int leftIdx(MinHeap *heap, int nodeIndex);
int parentIdx(MinHeap *heap, int nodeIndex);
int minChildIdx(MinHeap *heap, int nodeIndex);
int distanceAt(MinHeap *heap, int nodeIndex);
int idAt(MinHeap *heap, int nodeIndex);

// Open GL Initialization functions
void initGlut(const char *winName, int sizeX, int sizeY, int positionX, int positionY);

// Open GL callbacks for handling events in glut
void WindowReshape(int w, int h);
void WindowDisplay(void);
void kbHandler(unsigned char key, int x, int y);

// Image processing functions
void tweakSprite(struct sprite *buf, unsigned char xR, unsigned char xG, unsigned char xB);
void updateFrame(int path[graph_size][2]);
void overlaySprite(struct sprite *dest, struct sprite *src, int x, int y);
void renderMaze(struct sprite *buf, double AdjMat[graph_size][4], unsigned char R, unsigned char G, unsigned char B);
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

#endif
