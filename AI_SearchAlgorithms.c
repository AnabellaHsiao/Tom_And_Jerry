/*
  CSC D84 - Search Algorithms

  Search algorithms code - handles the command line, sets up the image
  rendering loop, and provides all the support functions required to
  implement heuristic search and mini-max for adversarial games.

  NOTE: You don't have to read most of the code in this file, you
  are responsible only for the functions ABOVE main(), which have
  sections clearly marked 'TO DO'.

  Anything else should NOT BE MODIFIED. You can read through it if
  you are curious about how the code implements what you're seeing,
  but there's nothing to modify and you are not responsible for
  learning or explaining any of the code below the clearly marked
  sections

  (c) F. Estrada, Updated Sep. 2025
*/

#include "AI_SearchAlgorithms.h"

// ***********  CONSTANTS  **********************************
#define ROOT_INDEX 0
#define NOTHING -1
#define INT_MAX 10000
#define CATVOIDANCE 1000
#define CHEESEATTRACTION 1
#define EDGEPENALTY 20
#define AHHHH 2
/*************************************************************************
 * Functions you have to complete for this assignment start below
 * this commend block
 * **********************************************************************/

void Heuristic_Search(int (*heuristic)(int x, int y))
{

	/*
	  This function implements A* search for path finding - the function will be used by both
	  - Regular A* with an admissible heuristic
	  - A* with a no-kitty heuristic intended to make the mouse as smart as possible

	  Board and game layout:

	 The game takes place on a grid of size 32x32.
	 The grid of locations is represented by a graph with one node per grid location, so given
	 the 32x32 cells, the graph has 1024 nodes.

	 To create a maze, we connect cell locations in the grid in such a way that a) there is a path from any
	 grid location to any other grid location (i.e. there are no disconnected subsets of nodes in the graph),
	 and b) there are loops.

	 Since each node represents a grid location, each node can be connected to up to 4 neighbours in the
	 top, right, bottom, and left directions respectively:

		 node at (i,j-1)
		   ^
		   |
	 (node at i-1, j) <- node at (i,j) -> node at (i+1, j)
		   |
		   v
		 node at (i,j+1)

	 The graph is therefore stored as an adjacency list with size 1024 x 4, with one row per node in the
	 graph, and 4 columns corresponding to the weight of an edge linking the node with each of its 4
	 possible neighbours in the order top, right, bottom, left (clockwise from top).

	 Since all we care is whether nodes are connected. Weights will be either 0 or 1, if the weight is
	 1, then the neighbouring nodes are connected, if the weight is 0, they are not. For example, if

	 Graph[i][0] = 0
	 Graph[i][1] = 1        // The graph is stored in a global array called 'Graph' (of course)
	 Graph[i][2] = 0
	 Graph[i][3] = 1

	 then node i is connected to the right and left neighbours, but not to top or bottom.

	 The index in the graph for the node corresponding to grid location (x,y) is

	 index = x + (y*size_X) 		or in this case		index = x + (y*32)

	 Conversely, if you have the index and want to figure out the grid location,

	 x = index % size_X		or in this case		x = index % 32
	 y = index / size_Y		or in this case		y = index / 32

	 (all of the above are *integer* operations!)

	 A path is a sequence of (x,y) grid locations. We store it using an array of size
	 1024 x 2 (since there are 1024 locations, this is the maximum length of any
	 path that visits locations only once).

	 The driver code expects paths to be stored in a global array called 'Path'
	 (of course)

	 Agent locations are coordinate pairs (x,y)

	  This function takes a single input argument:
	   (*heuristic)(int x, int y)
		   - This is a pointer to one of the two heuristic functions you have to complete.
			 Depending on which one is passed into the function, the behaviour will be
			 either regular A*, or A* no-kitty.

			 * How to call the heuristic function from within this function : *
			 - Like any other function:
			   h = heuristic( x, y);

	  Return values:
	   Your search code will directly the following GLOBAL arrays:

	   - Path[graph_size][2]	: Your search code will update this array to contain the path from
			   the mouse to one of the cheese chunks. The order matters, so
			   Path[0][:] must be the mouse's current location, Path[1][:]
			   is the next move for the mouse. Each successive row will contain
			   the next move toward the cheese, and the path ends at a location
			   whose coordinates correspond to one of the cheese chunks.
			   Any entries beyond that must remain set to -1

	   - grid_value[size_X][size_Y] 	:  Your search code will update this array to contain the
				  order in which each location in the grid was expanded
				  during search.

				  Recall that A* uses a priority-queue, and expands nodes in the order
				  they show up at the front of the priority queue. So, suppose that
				  the function's priority queue gives us the following node indexes
				  in order (from left to right, the current mouse location is at
				  node 25):

							   25, 78, 241, 11,  53

							  Then the grid_value array will be updated so that the location
							  corresponding to node 25 will have a value '0', the location
							  for node 78 will have value '1', the location for node 241
							  will have value '2', and so on...

							  Notice that this DOES NOT PRODUCE A PATH, it just shows the
							  order in which A* is expanding nodes while looking for a
							  solution, and should show that A* favour expansion in the
							  general direction of a solution.

							  Contrast this with what you would expect if using regular BFS

							  In the worst case, the A* function will explore the entire
							  grid, so the grid_values array will contain all values
							  in [0 - 1023].

	   * Your code MUST NOT modify the locations or numbers of cats and/or cheeses, the graph,
			 or the location of the mouse. This will be easily noticeable and will cause you
			 to lose points on the assignment

		   * You are free to implement the priority queue in any way you want - if you want to use
			 a standard library that is also ok BUT MAKE SURE TO DOCUMENT which library you used
			 for the PQ.

		   * The path found by this function MUST be valid - the mouse must not walk through walls
			 or teleport.

		   * If the game ends unexpectedly - that means an error occurred somewhere in your solution
			 which the driver code can't handle - in that stuation, it teleports the mouse to
			 (-1, -1) which is not a valid location, and the game ends. If this is happening to you,
			 check your path construction code, and test every step of your solution. There is a bug
			 for sure!
	*/

	/********************************************************************************************************
	 *
	 * TO DO:	Implament A* as discussed in lecture. The function must produce the correct search
	 *          behaviour - nodes have to be expanded in order of heuristic cost.
	 *
	 *          When adding the neighbours of the current node, if there is a tie in terms of
	 *          heuristic cost, expand them in the order: TOP, RIGHT, BOTTOM, and LEFT
	 *
	 *		How you design your solution is up to you. But:
	 *
	 *		- Document your implementation by adding concise and clear comments in this file
	 *		- Document your design (how you implemented the solution, and why) in the report
	 *
	 ********************************************************************************************************/

	int goal = -1;
	int grid_counter = 0;
	// construct priority queue via minheap, Q for minheap prio queue, d for shortest distances, p for predecessor in shortest path
	MinHeap *Q = newHeap(graph_size);
	// keep an array of expanded nodes to fill - initialize to avoid garbage values
	struct node expanded_nodes[graph_size];
	memset(expanded_nodes, 0, sizeof(expanded_nodes));
	for (int i = 0; i < graph_size; i++)
	{
		expanded_nodes[i].predecessor = NOTHING;
		expanded_nodes[i].child = NULL;
	}
	// insert starting node into Q starting at mouse start location
	int id = getIndexFromXY(mouse[0][0], mouse[0][1]);
	insert(Q, 0, id, mouse[0][0], mouse[0][1], NOTHING, NULL);
	// insert all other nodes into Q with infinite distance
	for (int i = 0; i < graph_size; i++)
	{
		if (i != id)
		{
			int x = i % size_X;
			int y = i / size_Y;
			insert(Q, INT_MAX, i, x, y, NOTHING, NULL);
		}
	}
	while (Q->size != 0)
	{
		struct node current = extractMin(Q);

		// add current to expanded nodes
		expanded_nodes[current.id] = current;
		grid_value[current.x][current.y] = grid_counter++;
		// if found goal (cheese), break
		if (checkForCheese(current.x, current.y))
		{
			goal = current.id;
			break;
		}
		// check connected nodes
		for (int direction = 0; direction < 4; direction++)
		{
			if (Graph[current.id][direction] == 1)
			{
				int neighbor_x = current.x;
				int neighbor_y = current.y;
				if (direction == 0)
				{
					neighbor_y -= 1;
				}
				else if (direction == 1)
				{
					neighbor_x += 1;
				}
				else if (direction == 2)
				{
					neighbor_y += 1;
				}
				else if (direction == 3)
				{
					neighbor_x -= 1;
				}
				int neighbor_id = getIndexFromXY(neighbor_x, neighbor_y);
				int tentative_distance = current.distance + 1 + heuristic(neighbor_x, neighbor_y);
				if (decreaseDistance(Q, neighbor_id, tentative_distance))
				{
					// update predecessor/parent
					int index = Q->indexMap[neighbor_id];
					Q->arr[index].predecessor = current.id;
				}
			}
		}
	}

	if (goal == -1)
	{
		// No path found - just stay at current position
		Path[0][0] = mouse[0][0];
		Path[0][1] = mouse[0][1];
		deleteHeap(Q);
		return;
	}

	// reconstruct path by building linked list from goal to start
	struct node *iter = &expanded_nodes[goal];
	while (iter->predecessor != NOTHING)
	{
		// have the predecessor child point to current
		expanded_nodes[iter->predecessor].child = iter;
		iter = &expanded_nodes[iter->predecessor];
	}

	// fill Path array
	iter = &expanded_nodes[getIndexFromXY(mouse[0][0], mouse[0][1])];
	int path_index = 0;
	while (iter != NULL)
	{
		Path[path_index][0] = iter->x;
		Path[path_index][1] = iter->y;
		path_index++;
		iter = iter->child;
	}

	deleteHeap(Q);
	return;
}

int H_cost(int x, int y)
{

	/*
	   This function computes and returns the heuristic cost for location x,y.
	   As discussed in lecture, this means estimating the cost of getting from x,y to the goal.
	   The goal is cheese. Which cheese is up to you.
	   Whatever you code here, your heuristic must be admissible.

	   Input arguments have the same meaning as in the search() function above.
	*/

	// manhatten distance from x,y to closest cheese
	int min_dist = graph_size; // set as largest val possible
	int curr = min_dist;
	for (int chs_idx = 0; chs_idx < 10; chs_idx++)
	{
		curr = abs(x - cheese[chs_idx][0]) + abs(y - cheese[chs_idx][1]);
		if (min_dist > curr)
		{
			min_dist = curr;
		}
	}

	return min_dist;
}

// manhatten distance helper function
double manhatten_distance(int x1, int y1, int x2, int y2)
{
	return abs(x1 - x2) + abs(y1 - y2);
}

// euclidean distance helper function
double euclidean_distance(int x1, int y1, int x2, int y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// closest cat to (x,y)
double closest_cat_distance(int x, int y)
{
	double min_dist = size_X + size_Y; // set as largest val possible
	double curr = min_dist;
	for (int cat_idx = 0; cat_idx < n_cats; cat_idx++)
	{
		curr = manhatten_distance(x, y, cats[cat_idx][0], cats[cat_idx][1]);
		if (min_dist > curr)
		{
			min_dist = curr;
		}
	}
	return min_dist;
}

// closest cheese to (x,y)
double closest_cheese_distance(int x, int y)
{
	double min_dist = size_X + size_Y; // set as largest val possible
	double curr = min_dist;
	for (int chs_idx = 0; chs_idx < n_cheese; chs_idx++)
	{
		curr = manhatten_distance(x, y, cheese[chs_idx][0], cheese[chs_idx][1]);
		if (min_dist > curr)
		{
			min_dist = curr;
		}
	}
	return min_dist;
}

// cheese score of (x,y)
double cheese_distance_sum(int x, int y)
{
	double sum = 0;
	for (int chs_idx = 0; chs_idx < n_cheese; chs_idx++)
	{
		sum += n_cheese / (manhatten_distance(x, y, cheese[chs_idx][0], cheese[chs_idx][1]) + 1);
	}
	return sum;
}

// cat score of (x,y)
double cat_distance_sum(int x, int y)
{
	double sum = 0;
	for (int cat_idx = 0; cat_idx < n_cats; cat_idx++)
	{
		sum += n_cats / (manhatten_distance(x, y, cats[cat_idx][0], cats[cat_idx][1]) + 1);
	}
	return sum;
}

// find the cheese index of the closest cheese to mouse
int closest_cheese_index(int x, int y)
{
	double min_dist = size_X + size_Y; // set as largest val possible
	double curr = min_dist;
	int index = -1;
	for (int chs_idx = 0; chs_idx < n_cheese; chs_idx++)
	{
		curr = manhatten_distance(x, y, cheese[chs_idx][0], cheese[chs_idx][1]);
		if (min_dist > curr)
		{
			min_dist = curr;
			index = chs_idx;
		}
	}
	return index;
}

// find the cat index of the closest cat to (x,y)
int closest_cat_index(int x, int y)
{
	double min_dist = size_X + size_Y; // set as largest val possible
	double curr = min_dist;
	int index = -1;
	for (int cat_idx = 0; cat_idx < n_cats; cat_idx++)
	{
		curr = manhatten_distance(x, y, cats[cat_idx][0], cats[cat_idx][1]);
		if (min_dist > curr)
		{
			min_dist = curr;
			index = cat_idx;
		}
	}
	return index;
}

//

// give x1,y1, x2,y2, x3,y3, create a line from x1,y1 to x2,y2 then get the distance of x3,y3 to the closest point on the line
double dist_from_line(int x1, int y1, int x2, int y2, int x3, int y3)
{
	// check that x3,y3 is between x1,y1 and x2,y2, otherwise assign same value as corner
	if ((x3 <= fmin(x1, x2)) || (x3 >= fmax(x1, x2)) || (y3 <= fmin(y1, y2)) || (y3 >= fmax(y1, y2)))
	{
		// handle vertical line case
		if (x2 - x1 == 0)
		{
			// calculate distance from (x1,y2) to the closest point on the line from (x1,y1) to (x2,y2) and return it
			return 1 + fabs(x3 - x1);
		}
		// calculate distance from (x2,y1) to the closest point on the line from (x1,y1) to (x2,y2) and return it
		//  get slope of line and intercept
		double m = (double)(y2 - y1) / (double)(x2 - x1);
		double b = y1 - m * x1;
		// get distance from point to line, so we have Ax + By + C = 0 that is, mx - y + b = 0
		double A = m;
		double B = -1.0;
		double C = b;
		// get the shortest distance from point x2,y1 to line
		double dist = fabs(A * x2 + B * y1 + C) / sqrt(A * A + B * B);

		return dist; // not between, return max (dist of corner)
	}

	// handle vertical line case
	if (x2 - x1 == 0)
	{
		return fabs(x3 - x1);
	}
	// get slope of line and intercept
	double m = (double)(y2 - y1) / (double)(x2 - x1);
	double b = y1 - m * x1;
	// get distance from point to line, so we have Ax + By + C = 0 that is, mx - y + b = 0
	double A = m;
	double B = -1.0;
	double C = b;
	// get the shortest distance from point x3,y3 to line
	double dist = fabs(A * x3 + B * y3 + C) / sqrt(A * A + B * B);
	return dist;
}

int H_cost_nokitty(int x, int y)
{
	/*
	 This function computes and returns the heuristic cost for location x,y.
	 As discussed in lecture, this means estimating the cost of getting from x,y to the goal.
	 The goal is cheese.

	 However - this time you want your heuristic function to help the mouse avoid being eaten.
	 Therefore: You have to somehow incorporate knowledge of the cats' locations into your
	 heuristic cost estimate. How well you do this will determine how well your mouse behaves
	 and how good it is at escaping kitties.

	 This heuristic *does not have to* be admissible.

	 Input arguments have the same meaning as in the search() function above.
	*/

	// double dist_to_closest_cheese = closest_cheese_distance(x, y);
	// double dist_to_closest_cat = closest_cat_distance(x, y);

	// double cheese_score = cheese_distance_sum(x, y);
	// double cat_score = cheese_score + cat_distance_sum(x, y);

	// // if dist to cat is closer than dist to cheese, add penalty to heuristic
	// int penalty = 0;
	// if (dist_to_closest_cat < dist_to_closest_cheese)
	// {
	// 	// closer cat = higher penalty, might need to increase from 80 for smarter cats
	// 	penalty = (int)(CATVOIDANCE / (dist_to_closest_cat + 1)); // +1 to avoid div by 0
	// }

	// // add penalty proportional to closeness to edge of map, to avoid getting cornered
	// penalty += (int)(EDGEPENALTY / (fmin(fmin(x, size_X - x - 1), fmin(y, size_Y - y - 1)) + 1));

	// printf("Cheese dist: %.2f, Cat dist: %.2f, Penalty: %d\n", dist_to_closest_cheese, dist_to_closest_cat, penalty);
	// // print cheese and cat score
	// printf("Cheese score: %.2f, Cat score: %.2f, Penalty: %d\n", cheese_score, cat_score, penalty);
	// return dist_to_closest_cheese + penalty;

	// new attempt

	// get distance to line from mouse to cheese, and how far x,y is from that line
	int closest_cheese = closest_cheese_index(mouse[0][0], mouse[0][1]);
	double dist_from_cheeseline = dist_from_line(mouse[0][0], mouse[0][1], cheese[closest_cheese][0], cheese[closest_cheese][1], x, y);

	// get distance to line from mouse to closest cat, and how far x,y is from that line
	int closest_cat = closest_cat_index(mouse[0][0], mouse[0][1]);
	double dist_from_catline = dist_from_line(mouse[0][0], mouse[0][1], cats[closest_cat][0], cats[closest_cat][1], x, y);
	double penalty = 0;
	// penalty = CATVOIDANCE / (dist_from_catline + 1); // +1 to avoid div by 0
	//   only add penalty if mouse is within AHHHH units of nearest cat
	if (closest_cat_distance(x, y) <= AHHHH)
	{
		penalty = CATVOIDANCE;
	}
	// heuristic 2, for each cat, get distance from x,y to cat(chebyshev) and add 1 / (dist + 1) to penalty
	// for (int cat_idx = 0; cat_idx < n_cats; cat_idx++)
	// {
	// 	penalty += 1 / (fmax(abs(x - cats[cat_idx][0]), abs(y - cats[cat_idx][1])) + 1);
	// }

	int heuristic1 = CHEESEATTRACTION * dist_from_cheeseline + penalty;
	int heuristic2 = CHEESEATTRACTION * dist_from_cheeseline + (100 - (closest_cat_distance(x, y))) + penalty;

	// only print if x,y is mouse location
	if (x == mouse[0][0] && y == mouse[0][1])
	{

		printf("Dist from cheese line: %.2f, Dist from cat line: %.2f\n, Dist from closest cat: %.2f, Dist from closest cheese: %.2f, heuristic1: %d, heuristic2: %d\n", dist_from_cheeseline, dist_from_catline, closest_cat_distance(x, y), closest_cheese_distance(x, y), heuristic1, heuristic2);
	}

	// calculate heuristic cost, we want to minimize cost if close to cheese line and maximize cost if close to cat line
	return heuristic2; // +1 to avoid div by 0
}

double MiniMax(int cat_loc[10][2], int ncats, int cheese_loc[10][2], int ncheeses, int mouse_loc[1][2], int mode, double (*utility)(int cat_loc[10][2], int cheese_loc[10][2], int mouse_loc[1][2], int cats, int cheeses, int depth), int agentId, int depth, int maxDepth, double alpha, double beta)
{
	/*
	  In this function you will implement MiniMax search as discussed in lecture.

	  IMPORTANT NOTE: Mini-max is a recursive procedure. This function will need to fill-in the mini-max values for
		  all game states down to the maximum search depth specified by the user. In order to do that,
		  the function needs to be called with the correct state at each specific node in the mini-max
		  search tree.

		  The game state is composed of:

		 * Mouse, cat, and cheese positions (and number of cats and cheeses)
		   NOTE THAT THESE WILL IN GENERAL NOT BE THE GLOBAL ARRAYS that contain the CURRENT game state,
				  but a MODIFIED version of these which is updated according to specific moves and
				  configurations, and passed into the MiniMax function so the game can be SIMULATED
				  up to a specified depth.

		  Therefore, you will need to define local variables to keep the game state at each node of the
		  mini-max search tree, and you will need to update this state when calling recursively so that
		  the search does the right thing.

		  This function *must check* whether:
		 * A candidate move results in a terminal configuration (cat eats mouse, mouse eats cheese)
		   at which point it calls the utility funceefvtion to get a value
		 * Maximum search depth has been reached (depth==maxDepth), at which point it will also call
		   the utility function to get a value
		 * Otherwise, evaluate possible moves at this level, and for each of these, figure out the
		   game configuration and call MiniMax() recursively to get a utility value. Then take
		   the MAX or MIN depending on whose move is happening at this level.

	  Uses:
		   Graph[1024][2]    -  The graph describing the maze, same as with Heuristic Search, GLOBAL
		   Path[1024][2]     -  A GLOBAL array to hold a mouse path. Since MiniMax specifies only one
								move per turn, the TOP-LEVEL call that chooses the move must place
								the NEXT move in Path[0][0], and Path[0][1]
		   grid_values[32][32]   -  This array must be updated with the UTILITY values for different
									grid locations - i.e. during MiniMax, the mouse is simulating
									moving in different directions, and it will simulate visiting
									different grid cells around its current location. Update the
									grid_values[][] array to contain the BEST utility the mouse
									could expect if it reaches that corresponding grid cell.
									This will be positive or negative, and will show you which
									cells the mouse thinks are better and which are worse. If you
									look at the displayed utilities, that should tell you whether
									your utlity function is doing a reasonable job or not.

	  Arguments:
	   cat_loc[10][2], cats   - Location of cats and number of cats (we can have at most 10,
			  but there can be fewer). Only valid cat locations are 0 to (cats-1)
	   cheese_loc[10][2], cheeses - Location and number of cheese chunks (again at most 10,
				  but possibly fewer). Valid locations are 0 to (cheeses-1)
	   mouse_loc[1][2] - Mouse location - there can be only one!
			 mode = 0 	- No alpha-beta pruning
			 mode = 1	- Alpha-beta pruning

	   (*utility)(int cat_loc[10][2], int cheese_loc[10][2], int mouse_loc[1][2], int cats, int cheeses, int depth, double gr[graph_size][4]);
		   - This is a pointer to the utility function which returns a value for a specific game configuration

			  NOTE: You are allowed to do any computation you want in your utility function. HOWEVER - there is a limited time budget,
							if your utility function takes too long to compute, we can only run your algorithm with lower max_depth. So
							think carefully about the tradeoff:

						 better (but slower) utility and lower search depth  -- may or may not beat --  worse (but faster) utility and greater search depth

			 * How to call the utility function from within this function : *
			 - Like any other function:
			   u = utility(cat_loc, cheese_loc, mouse_loc, ncats, ncheeses, depth);

	   agentId: Identifies which agent we are doing MiniMax for. agentId=0 for the mouse, agentId in [1, cats] for cats. Notice that recursive calls
					to this function should increase the agentId to reflect the fact that the next level down corresponds to the next agent! For a game
					with two cats and a mouse, the agentIds for the recursion should look like 0, 1, 2, 0, 1, 2, ...

	   depth: Current search depth - whether this is a MIN or a MAX node depends both on depth and agentId.

	   maxDepth: maximum desired search depth - once reached, your code should somehow return
		   a minimax utility value for this location.

	   alpha. beta: alpha and beta values passed from the parent node to constrain search at this
			  level.

	  Return values:
	   Your search code will directly update data passed-in as arguments:

	   - Mini-Max value  : Notice this function returns a double precision number. This is
					 the minimax value at this level of the tree. It will be used
					 as the recursion backtracks filling-in the mini-max values back
					 from the leaves to the root of the search tree.

	  UPDATES:

	   - grid_values[size_X][size_Y] 	:  Your search code will update this array to contain the
											  minimax value for locations that were expanded during
								  the search. This must be done *only* for the mouse
											  which means agent_id=0.

											  Values in this array will be in the range returned by
											  your utility function.

			AT THE TOP LEVEL ONLY (i.e. when the function knows the next optimal move)
			- Path[0][:]		: The next location for the mouse after MiniMax.
								 The move MUST be valid, the mouse should not walk through walls or
								 teleport.

	   * Your code MUST NOT modify the GLOBAL locations or numbers of cats and/or cheeses, the graph,
			 or the location of the mouse *

	   That's that, now, implement your solution!
	*/

	/********************************************************************************************************
	 *
	 * TO DO:	Implement code to perform a MiniMax search. This will involve a limited-depth BFS-like
	 *              expansion. Once nodes below return values, your function will propagate minimax utilities
	 *		as per the minimax algorithm.
	 *
	 *		Note that if alpha-beta pruning is specified, you must keep track of alphas and betas
	 *		along the path.
	 *
	 *		You can use helper functions if it seems reasonable. Add them to the MiniMax_search.h
	 *		file and explain in your code why they are needed and how they are used.
	 *
	 *		Recursion should appear somewhere.
	 *
	 *		MiniMax cost: If the agentId=0 (Mouse), then once you have a MiniMax value for a location
	 *		in the maze, you must update minmax_cost[][] for that location.
	 *
	 *		How you design your solution is up to you. But:
	 *
	 *		- Document your implementation by adding concise and clear comments in this file
	 *		- Document your design (how you implemented the solution, and why) in the report
	 *
	 ********************************************************************************************************/

	// Stub so that the code compiles/runs - This will be removed and replaced by your code!

	// mouse min, cat max

	// if terminal state or max depth reached, return utility score
	if (depth == maxDepth)
	{
		double util = utility(cat_loc, cheese_loc, mouse_loc, ncats, ncheeses, depth);
		printf("Reached max depth %d, utility: %.2f\n", depth, util);
		return util;
	}

	// check for terminal state: mouse eats a cheese, no more cheeses, cat eats mouse
	if (checkForTerminalState(cat_loc, cheese_loc, mouse_loc, ncats, ncheeses))
	{
		double util = utility(cat_loc, cheese_loc, mouse_loc, ncats, ncheeses, depth);
		printf("Reached terminal state at depth %d, utility: %.2f\n", depth, util);
		return util;
	}

	// if agentId is mouse, return min of minimax(depth+1, each possible mouse move)
	if (agentId == 0)
	{
		double min_util = 100000.0;
		// for each possible mouse move
		for (int direction = 0; direction < 4; direction++)
		{
			int mouse_x = mouse_loc[0][0];
			int mouse_y = mouse_loc[0][1];
			if (Graph[getIndexFromXY(mouse_x, mouse_y)][direction] == 1)
			{
				// valid move
				if (direction == 0)
				{
					mouse_y -= 1;
				}
				else if (direction == 1)
				{
					mouse_x += 1;
				}
				else if (direction == 2)
				{
					mouse_y += 1;
				}
				else if (direction == 3)
				{
					mouse_x -= 1;
				}
				int new_mouse_loc[1][2] = {{mouse_x, mouse_y}};
				double util = MiniMax(cat_loc, ncats, cheese_loc, ncheeses, new_mouse_loc, mode, utility, 1, depth + 1, maxDepth, alpha, beta);
				// update grid_values for mouse
				if (agentId == 0)
				{
					grid_value[mouse_loc[0][0]][mouse_loc[0][1]] = util;
				}
				if (util < min_util)
				{
					min_util = util;
					// at top level, update Path with next move
					if (depth == 0)
					{
						Path[depth][0] = mouse_x;
						Path[depth][1] = mouse_y;
					}
				}
				// alpha-beta pruning. mouse is a min node so update beta when util less than beta, stop when alpha greater than beta
				// only if in right mode
				if (mode == 1)
				{
					if (min_util < beta)
					{
						beta = min_util;
					}
					if (beta < alpha)
					{
						break; // beta cut-off
					}
				}
			}
		}
		// update grid_values for mouse
		// if (agentId == 0)
		// {
		// 	grid_value[mouse_loc[0][0]][mouse_loc[0][1]] = min_util;
		// }
		return min_util;
	}

	// if agentId is cat, return max utility of all possible cat moves (minimax(depth+1, each possible cat move))
	if (agentId >= 1 && agentId <= ncats)
	{
		double max_util = -100000.0;
		int cat_idx = agentId - 1;
		// for each possible cat move
		for (int direction = 0; direction < 4; direction++)
		{
			int cat_x = cat_loc[cat_idx][0];
			int cat_y = cat_loc[cat_idx][1];
			if (Graph[getIndexFromXY(cat_x, cat_y)][direction] == 1)
			{
				// valid move
				if (direction == 0)
				{
					cat_y -= 1;
				}
				else if (direction == 1)
				{
					cat_x += 1;
				}
				else if (direction == 2)
				{
					cat_y += 1;
				}
				else if (direction == 3)
				{
					cat_x -= 1;
				}
				int new_cat_loc[10][2];
				new_cat_loc[cat_idx][0] = cat_x;
				new_cat_loc[cat_idx][1] = cat_y;
				int next_agentId = (agentId % (ncats + 1)); // next agent
				double util = MiniMax(new_cat_loc, ncats, cheese_loc, ncheeses, mouse_loc, mode, utility, next_agentId, depth + 1, maxDepth, alpha, beta);
				if (util > max_util)
				{
					max_util = util;
				}
				// alpha-beta pruning
				if (mode == 1)
				{
					if (max_util > alpha)
					{
						alpha = max_util;
					}
					if (beta <= alpha)
					{
						break;
					}
				}
			}
		}
		return max_util;
	}

	printf("Error in MiniMax: invalid agentId %d\n", agentId);
	return 0;

	// Path[0][0] = mouse_loc[0][0];
	// Path[0][1] = mouse_loc[0][1];
	// return (0.0);
}

double utility(int cat_loc[10][2], int cheese_loc[10][2], int mouse_loc[1][2], int ncats, int ncheeses, int depth)
{
	/*
	 This function computes and returns the utility value for a given game configuration.
	 As discussed in lecture, this should return a positive value for configurations that are 'good'
	 for the mouse, and a negative value for locations that are 'bad' for the mouse.

	 How to define 'good' and 'bad' is up to you. Note that you can write a utility function
	 that favours your mouse or favours the cats, but that would be a bad idea... (why?)

	 Input arguments:

	   cat_loc - Cat locations
	   cheese_loc - Cheese locations
	   mouse_loc - Mouse location
	   cats - # of cats
	   cheeses - # of cheeses
	   depth - current search depth
	   gr - The graph's adjacency list for the maze

	   These arguments are as described in A1. Do have a look at your solution!
	*/

	// basic utility: if cat eats mouse, return positive large value, if mouse eats cheese, return negative large value
	for (int cat_idx = 0; cat_idx < ncats; cat_idx++)
	{
		if (mouse_loc[0][0] == cat_loc[cat_idx][0] && mouse_loc[0][1] == cat_loc[cat_idx][1])
		{
			return 1000; // cat eats mouse, bad for mouse(max for cat)
		}
	}
	for (int chs_idx = 0; chs_idx < ncheeses; chs_idx++)
	{
		if (mouse_loc[0][0] == cheese_loc[chs_idx][0] && mouse_loc[0][1] == cheese_loc[chs_idx][1])
		{
			return -1000; // mouse eats cheese, good for mouse(min)
		}
	}
	// return 0; // neutral state

	//  use heuristic function as utility: closer to cheese = lower utility, closer to cat = higher utility
	double dist_to_closest_cheese = closest_cheese_distance(mouse_loc[0][0], mouse_loc[0][1]);
	double dist_to_closest_cat = closest_cat_distance(mouse_loc[0][0], mouse_loc[0][1]);
	double utility_value = dist_to_closest_cheese - 1 / ((dist_to_closest_cat) * (dist_to_closest_cat)); // closer to cheese = lower utility, closer to cat = higher utility
	// if there is a wall between mouse and direction of cheese, add penalty to utility
	int closest_cheese = closest_cheese_index(mouse_loc[0][0], mouse_loc[0][1]);
	if (mouse_loc[0][0] < cheese_loc[closest_cheese][0])
	{
		// cheese is to the right
		if (Graph[getIndexFromXY(mouse_loc[0][0], mouse_loc[0][1])][1] == 0)
		{
			utility_value += 5; // add penalty
		}
	}
	else if (mouse_loc[0][0] > cheese_loc[closest_cheese][0])
	{
		// cheese is to the left
		if (Graph[getIndexFromXY(mouse_loc[0][0], mouse_loc[0][1])][3] == 0)
		{
			utility_value += 5; // add penalty
		}
	}
	if (mouse_loc[0][1] < cheese_loc[closest_cheese][1])
	{
		// cheese is below
		if (Graph[getIndexFromXY(mouse_loc[0][0], mouse_loc[0][1])][2] == 0)
		{
			utility_value += 5; // add penalty
		}
	}
	else if (mouse_loc[0][1] > cheese_loc[closest_cheese][1])
	{
		// cheese is above
		if (Graph[getIndexFromXY(mouse_loc[0][0], mouse_loc[0][1])][0] == 0)
		{
			utility_value += 5; // add penalty
		}
	}
	return utility_value;
}

// Helper functions

// Checks for terminal state
bool checkForTerminalState(int cat_loc[10][2], int cheese_loc[10][2], int mouse_loc[1][2], int ncats, int ncheeses)
{

	// check if cat eats mouse
	for (int cat_idx = 0; cat_idx < ncats; cat_idx++)
	{
		if (mouse_loc[0][0] == cat_loc[cat_idx][0] && mouse_loc[0][1] == cat_loc[cat_idx][1])
		{
			return true;
		}
	}
	// check if mouse eats cheese
	for (int chs_idx = 0; chs_idx < ncheeses; chs_idx++)
	{
		if (mouse_loc[0][0] == cheese_loc[chs_idx][0] && mouse_loc[0][1] == cheese_loc[chs_idx][1])
		{
			return true;
		}
	}
	return false;
};

// This gets index of graph from x,y coordinates
int getIndexFromXY(int x, int y)
{
	return x + (y * size_X);
};

// Minheap /Priority Queue functions

/* Swaps contents of heap->arr[index1] and heap->arr[index2] if both 'index1'
 * and 'index2' are valid indices for minheap 'heap'. Has no effect
 * otherwise.
 */
void swap(MinHeap *heap, int index1, int index2)
{
	if (isValidIndex(heap, index1) && isValidIndex(heap, index2))
	{
		struct node temp = heap->arr[index1];
		heap->arr[index1].distance = heap->arr[index2].distance;
		heap->arr[index1].id = heap->arr[index2].id;
		heap->arr[index1].x = heap->arr[index2].x;
		heap->arr[index1].y = heap->arr[index2].y;
		heap->arr[index1].predecessor = heap->arr[index2].predecessor;
		heap->arr[index1].child = heap->arr[index2].child;
		heap->indexMap[heap->arr[index2].id] = index1;
		heap->arr[index2].distance = temp.distance;
		heap->arr[index2].id = temp.id;
		heap->arr[index2].x = temp.x;
		heap->arr[index2].y = temp.y;
		heap->arr[index2].predecessor = temp.predecessor;
		heap->arr[index2].child = temp.child;
		heap->indexMap[temp.id] = index2;
	}
};

/* Bubbles up the element newly inserted into minheap 'heap' at index
 * 'nodeIndex', if 'nodeIndex' is a valid index for heap. Has no effect
 * otherwise.
 */
void bubbleUp(MinHeap *heap, int nodeIndex)
{

	int parentIndex = parentIdx(heap, nodeIndex);
	while (isValidIndex(heap, nodeIndex) && isValidIndex(heap, parentIndex) && (distanceAt(heap, nodeIndex) < distanceAt(heap, parentIndex)))
	{
		swap(heap, nodeIndex, parentIndex);
		nodeIndex = parentIndex;
		parentIndex = parentIdx(heap, nodeIndex);
	}
};

/* Bubbles down the element newly inserted into minheap 'heap' at the root,
 * if it exists. Has no effect otherwise.
 */
void bubbleDown(MinHeap *heap)
{
	int parentIndex = ROOT_INDEX;
	int childIndex = minChildIdx(heap, parentIndex);
	if (isValidIndex(heap, childIndex))
	{
		while (heap->arr[parentIndex].distance > heap->arr[childIndex].distance)
		{
			swap(heap, parentIndex, childIndex);
			parentIndex = childIndex;
			childIndex = minChildIdx(heap, parentIndex);
			if (!isValidIndex(heap, childIndex))
			{
				break;
			}
		}
	}
};

/* Returns the index of the left child of a node at index 'nodeIndex' in
 * minheap 'heap', if such exists.  Returns NOTHING if there is no such left
 * child.
 */
int leftIdx(MinHeap *heap, int nodeIndex)
{
	if (isValidIndex(heap, nodeIndex * 2 + 1))
	{
		return nodeIndex * 2 + 1;
	}
	return NOTHING;
};

/* Returns the index of the right child of a node at index 'nodeIndex' in
 * minheap 'heap', if such exists.  Returns NOTHING if there is no such right
 * child.
 */
int rightIdx(MinHeap *heap, int nodeIndex)
{
	if (isValidIndex(heap, nodeIndex * 2 + 2))
	{
		return nodeIndex * 2 + 2;
	}
	return NOTHING;
};

/* Returns the index of the min child of a node at index 'nodeIndex' in
 * minheap 'heap', if such exists.  Returns NOTHING if there is no such min
 * child.
 */
int minChildIdx(MinHeap *heap, int nodeIndex)
{
	int right = rightIdx(heap, nodeIndex);
	int left = leftIdx(heap, nodeIndex);
	if (right == NOTHING && left == NOTHING)
	{
		return NOTHING;
	}
	if (right == NOTHING)
	{
		return left;
	}
	if (left == NOTHING)
	{
		return right;
	}
	if (heap->arr[left].distance > heap->arr[right].distance)
	{
		return right;
	}
	return left;
};

/* Returns the index of the parent of a node at index 'nodeIndex' in minheap
 * 'heap', if such exists.  Returns NOTHING if there is no such parent.
 */
int parentIdx(MinHeap *heap, int nodeIndex)
{
	if (isValidIndex(heap, nodeIndex) && nodeIndex > ROOT_INDEX)
	{
		return (nodeIndex - 1) / 2;
	}
	return NOTHING;
};

/* Returns True if 'maybeIdx' is a valid index in minheap 'heap', and 'heap'
 * stores an element at that index. Returns False otherwise.
 */
bool isValidIndex(MinHeap *heap, int maybeIdx)
{
	if (maybeIdx < heap->size && maybeIdx >= 0)
	{
		return true;
	}
	return false;
};

/* Returns node at index 'nodeIndex' in minheap 'heap'.
 * Precondition: 'nodeIndex' is a valid index in 'heap'
 *               'heap' is non-empty
 */
struct node nodeAt(MinHeap *heap, int nodeIndex)
{
	return heap->arr[nodeIndex];
};

/* Returns priority of node at index 'nodeIndex' in minheap 'heap'.
 * Precondition: 'nodeIndex' is a valid index in 'heap'
 *               'heap' is non-empty
 */
int distanceAt(MinHeap *heap, int nodeIndex)
{
	return nodeAt(heap, nodeIndex).distance;
};

/* Returns ID of node at index 'nodeIndex' in minheap 'heap'.
 * Precondition: 'nodeIndex' is a valid index in 'heap'
 *               'heap' is non-empty
 */
int idAt(MinHeap *heap, int nodeIndex)
{
	return nodeAt(heap, nodeIndex).id;
};

/* Returns index of node with ID 'id' in minheap 'heap'.
 * Precondition: 'id' is a valid ID in 'heap'
 *               'heap' is non-empty
 */
int indexOf(MinHeap *heap, int id)
{
	for (int i = 0; i < heap->size; i++)
	{
		if ((heap->arr[i]).id == id)
		{
			return i;
		}
	}
	return NOTHING;
};

/*********************************************************************
 * Required functions
 ********************************************************************/

/* Returns the node with minimum priority in minheap 'heap'.
 * Precondition: heap is non-empty
 */
struct node getMin(MinHeap *heap)
{
	return heap->arr[ROOT_INDEX];
};

/* Removes and returns the node with minimum priority in minheap 'heap'.
 * Precondition: heap is non-empty
 */
struct node extractMin(MinHeap *heap)
{
	struct node min = getMin(heap);
	// Swap root with last element BEFORE decrementing size
	swap(heap, ROOT_INDEX, heap->size - 1);
	heap->size--;
	heap->indexMap[min.id] = NOTHING;
	bubbleDown(heap);
	return min;
};

/* Inserts a new node with priority 'priority' and ID 'id' into minheap 'heap'.
 * Precondition: 'id' is unique within this minheap
 *               0 <= 'id' < heap->capacity
 *               heap->size < heap->capacity
 */
void insert(MinHeap *heap, int distance, int id, int x, int y, int predecessor, struct node *child)
{
	struct node newNode;
	newNode.distance = distance;
	newNode.id = id;
	newNode.x = x;
	newNode.y = y;
	newNode.predecessor = predecessor;
	newNode.child = child;

	int insertIndex = heap->size;
	heap->arr[insertIndex] = newNode;
	heap->indexMap[id] = insertIndex;
	heap->size++;

	bubbleUp(heap, insertIndex);
};

/* Sets priority of node with ID 'id' in minheap 'heap' to 'newPriority', if
 * such a node exists in 'heap' and its priority is larger than
 * 'newPriority', and returns True. Has no effect and returns False, otherwise.
 * Note: this function bubbles up the node until the heap property is restored.
 */
bool decreaseDistance(MinHeap *heap, int id, int newDistance)
{
	int index = heap->indexMap[id];
	if (isValidIndex(heap, index) && heap->arr[index].distance > newDistance)
	{
		heap->arr[index].distance = newDistance;
		bubbleUp(heap, index);
		return true;
	}
	return false;
};

/* Returns a newly created empty minheap with initial capacity 'capacity'.
 * Precondition: capacity >= 0
 */
MinHeap *newHeap(int capacity)
{
	MinHeap *heap = (MinHeap *)calloc(1, sizeof(MinHeap));
	heap->size = 0;
	heap->capacity = capacity;
	heap->arr = (struct node *)calloc(capacity, sizeof(struct node));
	heap->indexMap = (int *)calloc(capacity, sizeof(int));
	for (int i = 0; i < capacity; i++)
	{
		heap->indexMap[i] = NOTHING;
	}
	return heap;
};

/* Frees all memory allocated for minheap 'heap'.
 */
void deleteHeap(MinHeap *heap)
{
	free(heap->arr);
	free(heap->indexMap);
	free(heap);
};

void printHeap(MinHeap *heap)
{
	printf("MinHeap with size: %d\n\tcapacity: %d\n\n", heap->size,
		   heap->capacity);
	printf("index: distance [ID]\t ID: index\n");
	for (int i = 0; i < heap->capacity; i++)
		printf("%d: %d [%d]\t\t%d: %d\n", i, distanceAt(heap, i), idAt(heap, i), i,
			   indexOf(heap, i));
	printf("%d: %d [%d]\t\t\n", heap->capacity, distanceAt(heap, heap->capacity),
		   idAt(heap, heap->capacity));
	printf("\n\n");
}

/*************************************************************************
 * you DO NOT need to read the code below this comment block, and
 * there's nothing for you to implement below this section
 * ***********************************************************************/

/**************************************************************************
 * Initialization - notice that the program's main loop is not in main()!
 *  this is because GLUT is handling everything after we pass control to
 *  whatever routine we specified to be the glut main loop.
 *  The exit statement at the end of main() is never reached.
 **************************************************************************/
int main(int argc, char *argv[])
{
	int px, py, done;

	// Command line parsing
	if (argc < 7)
	{
		fprintf(stderr, "Incorrect usage\n");
		fprintf(stderr, "  AI_SearchAlgorithms random_seed n_cats n_cheese search_mode max_depth\n");
		fprintf(stderr, "   random_seed : initial random seed for maze generation\n");
		fprintf(stderr, "   n_cats: Number of cats (1-4)\n");
		fprintf(stderr, "   n_cheese: Number of cheese (1-10)\n");
		fprintf(stderr, "   search_mode: Type of search algorithm in [0,4] (see handout)\n");
		fprintf(stderr, "     0 - A* with ZERO heuristic (gives you the search behaviour of BFS)\n");
		fprintf(stderr, "     1 - A* (basic heuristic) - must show correct search pattern\n");
		fprintf(stderr, "     2 - A* no-kitty - must be really good at avoiding cats!\n");
		fprintf(stderr, "     3 - MiniMax - must show smart behaviour, key is in the utility function\n");
		fprintf(stderr, "     4 - MiniMax w. alpha/beta pruning - must show correct behaviour and search speed-up\n");
		fprintf(stderr, "   cat_smarness: How smart kitties are in [0,1]\n");
		fprintf(stderr, "   max_depth: MiniMax maximum search depth in [1 30]\n");

		exit(0);
	}
	n_cats = atoi(argv[2]);
	n_cheese = atoi(argv[3]);
	r_seed = atoi(argv[1]);
	s_mode = atoi(argv[4]);
	cattitude = strtod(argv[5], NULL);
	md = atoi(argv[6]);
	if (cattitude < 0)
		cattitude = 0;
	if (cattitude > 1)
		cattitude = 1;

	if (n_cats < 1 || n_cats > 4 || n_cheese < 1 || n_cheese > 10 || s_mode < 0 || s_mode > 4 || cattitude < 0 || cattitude > 1 || md < 1 || md > 30)
	{
		fprintf(stderr, "Requested value out of range, please review the expected range for each parameter\n");
		exit(0);
	}

	// Init random number generator, reset cat, cheese, and mouse locations, and create maze
	srand48(r_seed);
	memset(&Graph[0][0], 0, graph_size * 4 * sizeof(double));
	memset(&cats[0][0], 0, 10 * 2 * sizeof(int));
	memset(&cheese[0][0], 0, 10 * 2 * sizeof(int));
	mouse[0][0] = mouse[0][1] = 0;
	Prim_MST(Graph);

	// Set initial (non-overlapping) positions for cats, mouse, and cheese
	for (int i = 0; i < n_cats; i++)
	{
		done = 0;
		while (!done)
		{
			done = 1;
			px = (int)round(drand48() * (size_X - 1));
			py = (int)round(drand48() * (size_Y - 1));
			for (int j = 0; j < i; j++)
				if (cats[j][0] == px && cats[j][1] == py)
					done = 0;
			if (done)
			{
				cats[i][0] = px;
				cats[i][1] = py;
			}
		}
	}
	for (int i = 0; i < n_cheese; i++)
	{
		done = 0;
		while (!done)
		{
			done = 1;
			px = (int)round(drand48() * (size_X - 1));
			py = (int)round(drand48() * (size_Y - 1));
			for (int j = 0; j < n_cats; j++)
				if (cats[j][0] == px && cats[j][1] == py)
					done = 0;
			for (int j = 0; j < i; j++)
				if (cheese[j][0] == px && cheese[j][1] == py)
					done = 0;
			if (done)
			{
				cheese[i][0] = px;
				cheese[i][1] = py;
			}
		}
	}
	done = 0;
	while (!done)
	{
		done = 1;
		px = (int)round(drand48() * (size_X - 1));
		py = (int)round(drand48() * (size_Y - 1));
		for (int j = 0; j < n_cats; j++)
			if (cats[j][0] == px && cats[j][1] == py)
				done = 0;
		for (int j = 0; j < n_cheese; j++)
			if (cheese[j][0] == px && cheese[j][1] == py)
				done = 0;
		if (done)
		{
			mouse[0][0] = px;
			mouse[0][1] = py;
		}
	}

	fprintf(stderr, "Cat locations:\n");
	for (int i = 0; i < n_cats; i++)
		fprintf(stderr, "%d, %d\n", cats[i][0], cats[i][1]);
	fprintf(stderr, "Cheese locations:\n");
	for (int i = 0; i < n_cheese; i++)
		fprintf(stderr, "%d, %d\n", cheese[i][0], cheese[i][1]);
	fprintf(stderr, "Mouse is at:\n");
	fprintf(stderr, "%d, %d\n", mouse[0][0], mouse[0][1]);

	fprintf(stderr, "Importing Sprites...\n");
	catSprt.imgdata = readPPMimage("kitty.ppm", &catSprt.sx, &catSprt.sy);
	mouseSprt.imgdata = readPPMimage("mousy.ppm", &mouseSprt.sx, &mouseSprt.sy);
	cheeseSprt.imgdata = readPPMimage("cheesy.ppm", &cheeseSprt.sx, &cheeseSprt.sy);

	if (catSprt.imgdata == NULL || mouseSprt.imgdata == NULL || cheeseSprt.imgdata == NULL)
	{
		fprintf(stderr, "Unable to open sprite image files. Please move them to this directory.\n");
		exit(0);
	}

	fprintf(stderr, "Init framebuffer and maze buffer\n");
	framebuffer.sx = 1024;
	framebuffer.sy = 1024;
	framebuffer.imgdata = (unsigned char *)calloc(1024 * 1024 * 3, sizeof(unsigned char));

	maze.sx = 1024;
	maze.sy = 1024;
	maze.imgdata = (unsigned char *)calloc(1024 * 1024 * 3, sizeof(unsigned char));

	renderMaze(&maze, Graph, 0, 0, 255);

	fprintf(stderr, "All done! let's play!\n");

	// Intialize GLUT and OpenGL, and launch the window display loop
	glutInit(&argc, argv);
	initGlut("Mouse vs. Cats v4.0 - D84 Search Algorithms", 1024, 1024, 10, 10);
	glutMainLoop(); // <--- from this point on, GLUT controls the program.
					//      the function called once for each frame is that which we
					//      registered with glutDisplayFunc(), namely, WindowDisplay()
					//      below. That function does everything else until the
					//	  program terminates.

	exit(0); // <--- this is never reached
}

/**************************************************************************
 *
 * Support functions
 *
 ***************************************************************************/
int zero(int x, int y) { return 0; }

int checkForMouse(int x, int y)
{
	// Returns 1 if the mouse is at (x,y)
	if (mouse[0][0] == x && mouse[0][1] == y)
		return (1);
	else
		return (0);
}

int checkForCheese(int x, int y)
{
	// Returns 1 if there is a cheese at (x,y)
	for (int i = 0; i < n_cheese; i++)
		if (cheese[i][0] == x && cheese[i][1] == y)
			return (1);
	return (0);
}

int checkForCats(int x, int y)
{
	// Returns 1 if there is a cat at (x,y)
	for (int i = 0; i < n_cats; i++)
		if (cats[i][0] == x && cats[i][1] == y)
			return (1);
	return (0);
}

void findPath(int start_x, int start_y, int (*goalCheck)(int x, int y))
{
	// Finds a path from location (x,y) to wherever the goalCheck() function
	// returns true. This can be used to find a path to a cheese, mouse, cat, or
	// anything else for which a goalCheck() function can be provided.

	int pred[graph_size];		 // Predecessor array
	int queue[graph_size];		 // Local queue for BFS
	int loc_path[graph_size][2]; // Scratch path location
	int node_loc[1][2];
	int head = 0;
	int idx = 0;
	int node_idx, next_idx;
	int visit_ord, visit_map[size_X][size_Y];

	for (int i = 0; i < graph_size; i++)
	{
		pred[i] = -1;
		queue[i] = -1;
	}
	memset(&visit_map[0][0], 0, size_X * size_Y * sizeof(int));

	// First node in the queue is the initial location
	queue[idx] = start_x + (start_y * size_X);
	idx++;
	visit_ord = 1;
	visit_map[start_x][start_y] = visit_ord++;

	while (head < idx)
	{
		node_idx = queue[head++];
		node_loc[0][0] = node_idx % size_X;
		node_loc[0][1] = node_idx / size_X;
		if (goalCheck(node_loc[0][0], node_loc[0][1]))
		{
			// Done! build path back-to-front in loc_path[][] and store front-to-back in Path[][]
			idx = 0;
			while (node_loc[0][0] != start_x || node_loc[0][1] != start_y)
			{
				loc_path[idx][0] = node_loc[0][0];
				loc_path[idx++][1] = node_loc[0][1];
				node_idx = pred[node_loc[0][0] + (node_loc[0][1] * size_X)];
				node_loc[0][0] = node_idx % size_X;
				node_loc[0][1] = node_idx / size_X;
			}
			loc_path[idx][0] = start_x;
			loc_path[idx][1] = start_y;
			for (int i = idx; i >= 0; i--)
			{
				Path[idx - i][0] = loc_path[i][0];
				Path[idx - i][1] = loc_path[i][1];
			}
			return;
		}
		else
		{
			if (Graph[node_loc[0][0] + (node_loc[0][1] * size_X)][0] == 1 && visit_map[node_loc[0][0]][node_loc[0][1] - 1] == 0)
			{
				next_idx = node_loc[0][0] + ((node_loc[0][1] - 1) * size_X);
				visit_map[node_loc[0][0]][node_loc[0][1] - 1] = visit_ord++;
				queue[idx] = next_idx;
				pred[next_idx] = node_idx;
				idx++;
			}
			if (Graph[node_loc[0][0] + (node_loc[0][1] * size_X)][1] == 1 && visit_map[node_loc[0][0] + 1][node_loc[0][1]] == 0)
			{
				next_idx = node_loc[0][0] + 1 + ((node_loc[0][1]) * size_X);
				visit_map[node_loc[0][0] + 1][node_loc[0][1]] = visit_ord++;
				queue[idx] = next_idx;
				pred[next_idx] = node_idx;
				idx++;
			}
			if (Graph[node_loc[0][0] + (node_loc[0][1] * size_X)][2] == 1 && visit_map[node_loc[0][0]][node_loc[0][1] + 1] == 0)
			{
				next_idx = node_loc[0][0] + ((node_loc[0][1] + 1) * size_X);
				visit_map[node_loc[0][0]][node_loc[0][1] + 1] = visit_ord++;
				queue[idx] = next_idx;
				pred[next_idx] = node_idx;
				idx++;
			}
			if (Graph[node_loc[0][0] + (node_loc[0][1] * size_X)][3] == 1 && visit_map[node_loc[0][0] - 1][node_loc[0][1]] == 0)
			{
				next_idx = node_loc[0][0] - 1 + ((node_loc[0][1]) * size_X);
				visit_map[node_loc[0][0] - 1][node_loc[0][1]] = visit_ord++;
				queue[idx] = next_idx;
				pred[next_idx] = node_idx;
				idx++;
			}
		}
	}
}

void catMoves(int idx, double smart)
{
	// Handles cat movement...

	double dice;
	int dir, clear, done;

	dice = drand48();
	if (dice <= smart)
	{
		for (int i = 0; i < graph_size; i++)
		{
			Path[i][0] = -1;
			Path[i][1] = -1;
		}
		findPath(cats[idx][0], cats[idx][1], checkForMouse);
		cats[idx][0] = Path[1][0];
		cats[idx][1] = Path[1][1];
		return;
	}
	else
	{
		done = 0;
		while (!done)
		{
			dir = (int)round(drand48() * 3);
			if (Graph[cats[idx][0] + (cats[idx][1] * size_X)][dir] == 1)
				done = 1;
		}
		switch (dir)
		{
		case 0:
			cats[idx][1] = cats[idx][1] - 1;
			break;
		case 1:
			cats[idx][0] = cats[idx][0] + 1;
			break;
		case 2:
			cats[idx][1] = cats[idx][1] + 1;
			break;
		case 3:
			cats[idx][0] = cats[idx][0] - 1;
			break;
		default:
			fprintf(stderr, "Something went wrong updating the cat position!\n");
		}

		if (cats[idx][0] >= size_X || cats[idx][1] >= size_Y || cats[idx][0] < 0 || cats[idx][1] < 0)
		{
			fprintf(stderr, "Ugh! Elvis-gato has left the building\n");
			cats[idx][0] = 15;
			cats[idx][1] = 15;
		}
	}
}

void randomMouse()
{
	// For search mode 0 (starter code) the mouse moves around randomly - poor dude
	static int dir = 0;
	int clear, done;
	double dice;

	// Check if current direction of motion is open
	clear = 0;
	if (Graph[mouse[0][0] + (mouse[0][1] * size_X)][dir] == 1)
		clear = 1;

	dice = drand48();
	if (clear == 0 || dice > .1)
	{
		// Choose a new random direction
		done = 0;
		while (!done)
		{
			dir = (int)round(drand48() * 3);
			if (Graph[mouse[0][0] + (mouse[0][1] * size_X)][dir] == 1)
				done = 1;
		}
	}

	Path[0][0] = mouse[0][0];
	Path[0][1] = mouse[0][1];

	switch (dir)
	{
	case 0:
		Path[1][0] = mouse[0][0];
		Path[1][1] = mouse[0][1] - 1;
		break;
	case 1:
		Path[1][0] = mouse[0][0] + 1;
		Path[1][1] = mouse[0][1];
		break;
	case 2:
		Path[1][0] = mouse[0][0];
		Path[1][1] = mouse[0][1] + 1;
		break;
	case 3:
		Path[1][0] = mouse[0][0] - 1;
		Path[1][1] = mouse[0][1];
		break;
	default:
		fprintf(stderr, "Something went wrong updating the mouse position!\n");
	}

	if (Path[1][0] < 0 || Path[1][0] >= size_X || Path[1][1] < 0 || Path[1][1] >= size_Y)
	{
		fprintf(stderr, "Ugh! we're out of the map!\n");
		Path[1][0] = mouse[0][0];
		Path[1][1] = mouse[0][1];
	}
}

void moveAway(int pos[1][2], int tgt[1][2], int path[graph_size][2], int mode)
{
	// Find a direction that moves an agent at pos[][] away or toward an agent
	// at tgt[][]. The destination position is returned in path[1][:].
	// If no such direction exists, path[1][:] will be equal to pos[][].
	// mode == 0 -> move away
	// mode == 1 -> move toward
	int best_dist;
	int cur_dist;

	best_dist = abs(tgt[0][0] - pos[0][0]) + abs(tgt[0][1] - pos[0][1]);
	path[0][0] = pos[0][0];
	path[0][1] = pos[0][1];
	path[1][0] = pos[0][0];
	path[1][1] = pos[0][1];

	if (Graph[pos[0][0] + (pos[0][1] * size_X)][0] == 1)
	{
		cur_dist = abs(tgt[0][0] - pos[0][0]) + abs(tgt[0][1] - (pos[0][1] - 1));
		if ((mode == 1 && cur_dist <= best_dist) || (mode == 0 & cur_dist >= best_dist))
		{
			best_dist = cur_dist;
			path[1][1] = pos[0][1] - 1;
		}
	}
	if (Graph[pos[0][0] + (pos[0][1] * size_X)][1] == 1)
	{
		cur_dist = abs(tgt[0][0] - (pos[0][0] + 1)) + abs(tgt[0][1] - pos[0][1]);
		if ((mode == 1 && cur_dist <= best_dist) || (mode == 0 & cur_dist >= best_dist))
		{
			best_dist = cur_dist;
			path[1][0] = pos[0][0] + 1;
		}
	}
	if (Graph[pos[0][0] + (pos[0][1] * size_X)][2] == 1)
	{
		cur_dist = abs(tgt[0][0] - pos[0][0]) + abs(tgt[0][1] - (pos[0][1] + 1));
		if ((mode == 1 && cur_dist <= best_dist) || (mode == 0 & cur_dist >= best_dist))
		{
			best_dist = cur_dist;
			path[1][1] = pos[0][1] + 1;
		}
	}
	if (Graph[pos[0][0] + (pos[0][1] * size_X)][3] == 1)
	{
		cur_dist = abs(tgt[0][0] - (pos[0][0] - 1)) + abs(tgt[0][1] - pos[0][1]);
		if ((mode == 1 && cur_dist <= best_dist) || (mode == 0 & cur_dist >= best_dist))
		{
			best_dist = cur_dist;
			path[1][0] = pos[0][0] - 1;
		}
	}
}

void Prim_MST(double AdjMat[graph_size][4])
{
	// Generate a maze by:
	//    - Generating a 4-connected graph of size size_X x size_Y nodes.
	//    - Initial edge weights are random
	//    - Using Prim's algorithm to generate an MST for the graph
	//      (this yields a maze, albeit a not very interesting one, with
	//      exactly one path between every pair of nodes)
	//    - Randomly inserting loops into the MST (which turns out maze
	//      into something a bit more interesting)
	//    - Binarizing the resulting graph's weights and storing them in
	//      the adjacency matrix provided by the function's argument.
	//
	// The adjacency matrix is stored in the following order:
	//    A[i][0]=top neighbour
	//    A[i][1]=right neighbour
	//    A[i][2]=bottom neighbour
	//    A[i][3]=left neighbour
	//    (clockwise from top)

	unsigned int MSTflg[graph_size];
	double MST[graph_size][4];
	int idx, idy, done;
	double minV;
	int minX, minY, minI, minJ, minK;

	memset(&MSTflg[0], 0, graph_size * sizeof(unsigned int));
	memset(&MST[0][0], 0, graph_size * 4 * sizeof(double));

	fprintf(stderr, "Creating maze of size %d x %d\n", size_X, size_Y);
	fprintf(stderr, "  -Initializing a random, 4-connected graph.\n");
	for (int i = 0; i < graph_size; i++)
		for (int k = 0; k < 4; k++)
			AdjMat[i][k] = drand48();

	// Remove links to locations outside the graph! (around map boundaries)
	for (int i = 0; i < size_X; i++)
	{
		AdjMat[0 + (size_X * i)][3] = 0;
		AdjMat[i][0] = 0;
		AdjMat[i + (size_X * (size_Y - 1))][2] = 0;
		AdjMat[size_X - 1 + (size_X * i)][1] = 0;
	}

	fprintf(stderr, "  -Prim's MST\n");
	MSTflg[0] = 1;
	done = 0;

	while (!done)
	{
		// This could be done much more efficiently with a priority queue... maybe
		// one day.
		// For now - for loops! whoo! (sorry B63!)
		done = 1;
		minV = 10000;
		minI = -1;
		minJ = -1;
		minK = -1;
		minX = -1;
		minY = -1;

		// Check for every node already in the MST (MSTflg==1)
		for (int i = 0; i < size_X; i++)
			for (int j = 0; j < size_Y; j++)
				if (MSTflg[i + (size_X * j)] == 1)
					for (int k = 0; k < 4; k++)
						if (AdjMat[i + (size_X * j)][k] > 0 && AdjMat[i + (size_X * j)][k] < minV)
						{
							if (k == 0)
							{
								idx = i;
								idy = j - 1;
							}
							else if (k == 1)
							{
								idx = i + 1;
								idy = j;
							}
							else if (k == 2)
							{
								idx = i;
								idy = j + 1;
							}
							else
							{
								idx = i - 1;
								idy = j;
							}
							if (MSTflg[idx + (idy * size_X)] == 0)
							{
								minV = AdjMat[i + (size_X * j)][k];
								minI = i;
								minJ = j;
								minX = idx;
								minY = idy;
								minK = k;
								done = 0;
							}
						}

		if (!done)
		{
			MSTflg[minX + (minY * size_X)] = 1;
			MST[minI + (minJ * size_X)][minK] = 1.0;
			if (minK == 0)
				MST[minX + (minY * size_X)][2] = 1.0;
			else if (minK == 1)
				MST[minX + (minY * size_X)][3] = 1.0;
			else if (minK == 2)
				MST[minX + (minY * size_X)][0] = 1.0;
			else
				MST[minX + (minY * size_X)][1] = 1.0;
		}
	} // End while(!done) - Prim's loop

	fprintf(stderr, "  -Inserting loops randomly\n");
	for (int i = 1; i < size_X - 1; i++)
		for (int j = 1; j < size_Y - 1; j++)
			for (int k = 0; k < 4; k++)
				if (drand48() < loop_fraction)
				{
					MST[i + (j * size_X)][k] = 1.0;
					if (k == 0)
						MST[i + ((j - 1) * size_X)][2] = 1.0;
					else if (k == 1)
						MST[i + 1 + (j * size_X)][3] = 1.0;
					else if (k == 2)
						MST[i + ((j + 1) * size_X)][0] = 1.0;
					else
						MST[i - 1 + (j * size_X)][1] = 1.0;
				}

	// Update Adjacency Matrix to contain the new MST
	memcpy(&AdjMat[0][0], &MST[0][0], graph_size * 4 * sizeof(double));
	fprintf(stderr, "  -Done!\n");
}

/**************************************************************************
 Image management functions

 Used to import .ppm ij+1mage data and handle basic drawing onto the framebuffer

 There is no need to read the code below for this assignment
***************************************************************************/
void updateFrame(int path[graph_size][2])
{
	// Render one frame for animation. Uses the maze image stored in 'maze', the agent
	// sprites, global data on the location of all agents, the search path, and the array
	// provided for students to update the search order.
	int scl_x;
	int scl_y;
	double max_vis, max_util, min_util;
	double bright;

	scl_x = framebuffer.sx / size_X;
	scl_y = framebuffer.sy / size_Y;

	// Copy maze onto framebuffer
	memcpy(framebuffer.imgdata, maze.imgdata, 1024 * 1024 * 3 * sizeof(unsigned char));

	// Render search ordering as given by visiting_order[][] array
	if (s_mode < 3)
	{
		max_vis = 0;
		for (int i = 0; i < size_X; i++)
			for (int j = 0; j < size_Y; j++)
				if (grid_value[i][j] > max_vis)
					max_vis = grid_value[i][j];

		if (max_vis > 0)
		{
			for (int i = 0; i < size_X; i++)
				for (int j = 0; j < size_Y; j++)
					if (grid_value[i][j] > 0)
					{
						bright = 255.0 * grid_value[i][j] / max_vis;
						block(&framebuffer, (i * scl_x) + 5, (j * scl_y) + 5, ((i + 1) * scl_x) - 5, ((j + 1) * scl_y) - 5, (unsigned char)round(bright * .5), (unsigned char)round(bright * .5), 129 - (unsigned char)round(bright * .5));
					}
		}
		// Render the path (of the mouse)
		for (int i = 1; i < graph_size; i++)
		{
			if (path[i][0] != -1)
				block(&framebuffer, (path[i][0] * scl_x) + 12, (path[i][1] * scl_y) + 12, ((path[i][0] + 1) * scl_x) - 12, ((path[i][1] + 1) * scl_y) - 12, 255, 255, 255);
			else
				break;
		}
	}
	else
	{
		// Render MiniMax utility as given by Minmax_cost[][] array
		max_util = -1000;
		min_util = 1000;
		for (int i = 0; i < size_X; i++)
			for (int j = 0; j < size_Y; j++)
			{
				if (grid_value[i][j] > max_util)
					max_util = grid_value[i][j];
				if (grid_value[i][j] < min_util)
					min_util = grid_value[i][j];
			}

		if (max_util > -1000)
		{
			for (int i = 0; i < size_X; i++)
				for (int j = 0; j < size_Y; j++)
					if (grid_value[i][j] != 0)
					{
						if (grid_value[i][j] > 0)
						{
							bright = 255.0 * (grid_value[i][j] / (max_util));
							block(&framebuffer, (i * scl_x) + 5, (j * scl_y) + 5, ((i + 1) * scl_x) - 5, ((j + 1) * scl_y) - 5, 0, (unsigned char)round(bright * .8), 0);
						}
						else
						{
							bright = 255.0 * (grid_value[i][j] / (min_util));
							block(&framebuffer, (i * scl_x) + 5, (j * scl_y) + 5, ((i + 1) * scl_x) - 5, ((j + 1) * scl_y) - 5, (unsigned char)round(bright * .8), 0, 0);
						}
					}
		}
	}

	// Overlay agents
	for (int i = 0; i < n_cats; i++)
		overlaySprite(&framebuffer, &catSprt, scl_x * cats[i][0], scl_y * cats[i][1]);
	for (int i = 0; i < n_cheese; i++)
		overlaySprite(&framebuffer, &cheeseSprt, scl_x * cheese[i][0], scl_y * cheese[i][1]);
	overlaySprite(&framebuffer, &mouseSprt, scl_x * mouse[0][0], scl_y * mouse[0][1]);
}

void renderMaze(struct sprite *buf, double AdjMat[graph_size][4], unsigned char R, unsigned char G, unsigned char B)
{
	// This renders the maze from the adjacency matrix representing our graph. Draw a wall
	// between each pair of neighboring nodes not connected by an edge in the graph.
	int xa, ya, xb, yb;
	double divx = buf->sx / size_X;
	double divy = buf->sy / size_Y;

	for (int i = 0; i < size_X; i++)
		for (int j = 0; j < size_Y; j++)
		{
			xa = (i * divx) + 1;
			ya = (j * divy) + 1;
			xb = ((i + 1) * divx) - 1;
			yb = ((j + 1) * divy) - 1;
			if (AdjMat[i + (j * size_X)][0] == 0)
				line(buf, xa, ya, xb, ya, R, G, B);
			if (AdjMat[i + (j * size_X)][1] == 0)
				line(buf, xb, ya, xb, yb, R, G, B);
			if (AdjMat[i + (j * size_X)][2] == 0)
				line(buf, xa, yb, xb, yb, R, G, B);
			if (AdjMat[i + (j * size_X)][3] == 0)
				line(buf, xa, ya, xa, yb, R, G, B);
		}
}

void tweakSprite(struct sprite *buf, unsigned char xR, unsigned char xG, unsigned char xB)
{
	// XOR non-zero pixels in the sprite with the specified byte values - easily changing sprite appearance
	for (int j = 0; j < buf->sy; j++)
		for (int i = 0; i < buf->sx; i++)
		{
			if (*(buf->imgdata + (3 * ((i) + ((j)*buf->sx))) + 0) > 0 || *(buf->imgdata + (3 * ((i) + ((j)*buf->sx))) + 1) > 0 || *(buf->imgdata + (3 * ((i) + ((j)*buf->sx))) + 2) > 0)
			{
				*(buf->imgdata + (3 * ((i) + ((j)*buf->sx))) + 0) = *(buf->imgdata + (3 * ((i) + ((j)*buf->sx))) + 0) ^ xR;
				*(buf->imgdata + (3 * ((i) + ((j)*buf->sx))) + 1) = *(buf->imgdata + (3 * ((i) + ((j)*buf->sx))) + 1) ^ xG;
				*(buf->imgdata + (3 * ((i) + ((j)*buf->sx))) + 2) = *(buf->imgdata + (3 * ((i) + ((j)*buf->sx))) + 2) ^ xB;
			}
		}
}

void overlaySprite(struct sprite *dest, struct sprite *src, int x, int y)
{
	// Pastes the sprite contained in 'src' onto the destination buffer in 'dest' at (top-left corner)
	// coordinates x,y.
	// Both the source and destination buffers are assumed to be RGB.

	for (int j = 0; j < src->sy; j++)
		for (int i = 0; i < src->sx; i++)
			if (x + i < dest->sx && x + i >= 0 && y + j < dest->sy && y + j >= 0)
			{
				*(dest->imgdata + (3 * ((x + i) + ((y + j) * dest->sx))) + 0) = *(src->imgdata + (3 * (i + (j * src->sx))) + 0);
				*(dest->imgdata + (3 * ((x + i) + ((y + j) * dest->sx))) + 1) = *(src->imgdata + (3 * (i + (j * src->sx))) + 1);
				*(dest->imgdata + (3 * ((x + i) + ((y + j) * dest->sx))) + 2) = *(src->imgdata + (3 * (i + (j * src->sx))) + 2);
			}
}

void line(struct sprite *buf, double x1, double y1, double x2, double y2, unsigned char R, unsigned char G, unsigned char B)
{
	/*
	  Herein the world's slowest draw-line function
	*/
	double vx, vy, xx, yy, ll;

	vx = (x2 - x1);
	vy = (y2 - y1);
	ll = sqrt((vx * vx) + (vy * vy));
	if (ll == 0)
		return;
	vx /= ll;
	vy /= ll;

	for (int i = 0; i <= ll; i++)
	{
		xx = x1 + (vx * i);
		yy = y1 + (vy * i);
		if ((int)round(xx) >= 0 && (int)round(xx) < buf->sx && (int)round(yy) >= 0 && (int)round(yy) < buf->sy)
		{
			*(buf->imgdata + (3 * ((int)round(xx) + ((int)round(yy) * buf->sx))) + 0) = R;
			*(buf->imgdata + (3 * ((int)round(xx) + ((int)round(yy) * buf->sx))) + 1) = G;
			*(buf->imgdata + (3 * ((int)round(xx) + ((int)round(yy) * buf->sx))) + 2) = B;
		}
	}
}

void block(struct sprite *buf, double x1, double y1, double x2, double y2, unsigned char R, unsigned char G, unsigned char B)
{
	/*
	  Herein the world's slowest filled-rectangle function. (x1,y1) is top-left corner, (x2,y2) is bottom right.
	*/

	int xx1, yy1, xx2, yy2;

	xx1 = (int)round(x1);
	xx2 = (int)round(x2);
	yy1 = (int)round(y1);
	yy2 = (int)round(y2);

	if (xx1 < 0)
		xx1 = 0;
	if (xx1 >= buf->sx)
		xx1 = buf->sx;
	if (xx2 < 0)
		xx2 = 0;
	if (xx2 >= buf->sx)
		xx2 = buf->sx;
	if (yy1 < 0)
		yy1 = 0;
	if (yy1 >= buf->sy)
		yy1 = buf->sy;
	if (yy2 < 0)
		yy2 = 0;
	if (yy2 >= buf->sy)
		yy2 = buf->sy;

	for (int j = yy1; j <= yy2; j++)
		for (int i = xx1; i <= xx2; i++)
		{
			*(buf->imgdata + (3 * (i + (j * buf->sx))) + 0) = R;
			*(buf->imgdata + (3 * (i + (j * buf->sx))) + 1) = G;
			*(buf->imgdata + (3 * (i + (j * buf->sx))) + 2) = B;
		}
}

unsigned char *readPPMimage(const char *filename, int *sizx, int *sizy)
{
	// Reads an image from a .ppm file. A .ppm file is a very simple image representation
	// format with a text header followed by the binary RGB data at 24bits per pixel.
	// The header has the following form:
	//
	// P6
	// # Optionally, one or more comment lines preceded by '#'
	// 340 200
	// 255
	//
	// The first line 'P6' is the .ppm format identifier, this is followed by one or more
	// lines with comments, typically used to inidicate which program generated the
	// .ppm file.
	// After the comments, a line with two integer values specifies the image resolution
	// as number of pixels in x and number of pixels in y.
	// The final line of the header stores the maximum value for pixels in the image,
	// usually 255.
	// After this last header line, binary data stores the RGB values for each pixel
	// in row-major order. Each pixel requires 3 bytes ordered R, G, and B.
	//
	// NOTE: Windows file handling is rather crotchetty. You may have to change the
	//       way this file is accessed if the images are being corrupted on read
	//       on Windows.
	//
	// readPPMdata converts the image colour information to floating point. This is so that
	// the texture mapping function doesn't have to do the conversion every time
	// it is asked to return the colour at a specific location.
	//
	// If an error occurs while reading the file, this function returns NULL, with sizx=sizy=0
	// otherwise the image array is returned, while sizx and sizy are updated to the imported
	// image's size.
	//

	FILE *f;
	unsigned char *im;
	char line[1024];

	f = fopen(filename, "rb+");
	if (f == NULL)
	{
		fprintf(stderr, "Unable to open file %s for reading, please check name and path\n", filename);
		return (NULL);
	}
	fgets(&line[0], 1000, f);
	if (strcmp(&line[0], "P6\n") != 0)
	{
		fprintf(stderr, "Wrong file format, not a .ppm file or header end-of-line characters missing\n");
		fclose(f);
		*sizx = 0;
		*sizy = 0;
		return (NULL);
	}

	// Skip over comments
	fgets(&line[0], 511, f);
	while (line[0] == '#')
	{
		fgets(&line[0], 511, f);
	}
	sscanf(&line[0], "%d %d\n", sizx, sizy); // Read file size

	fgets(&line[0], 9, f); // Read the remaining header line
	im = (unsigned char *)calloc((*sizx) * (*sizy) * 3, sizeof(unsigned char));
	if (im == NULL)
	{
		fprintf(stderr, "Out of memory allocating space for image\n");
		fclose(f);
		return (NULL);
	}

	fread(im, (*sizx) * (*sizy) * 3 * sizeof(unsigned char), 1, f);
	fclose(f);
	return (im);
}

/**************************************************************************
 Open GL initialization and management code

 Derived from CSC D18 OpenGL init code - no need to read below this part
 unless changes to the OpenGL setup are required.
**************************************************************************/
void initGlut(const char *winName, int sizeX, int sizeY, int positionX, int positionY)
{
	// This is the GLUT library initialization funtion. GLUT provides a simple
	// API for initializing OpenGL and setting up a window on screen with
	// specified properties.
	//
	// Input arguments:
	//    winName - Name of the OpenGL window being created (displayed on title bar)
	//    sizeX, sizeY - Window size in pixels
	//    positionX, positionY - Position of the window on the screen

	// Set video mode: double-buffered, color, depth-buffered
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	// Create window
	glutInitWindowSize(sizeX, sizeY);
	glutInitWindowPosition(positionX, positionY);
	windowID = glutCreateWindow(winName);

	// Setup callback functions to handle window-related events.
	// In particular, OpenGL has to be informed of which functions
	// to call when the image needs to be refreshed, and when the
	// image window is being resized.
	glutReshapeFunc(WindowReshape); // Call WindowReshape() whenever window resized
	glutDisplayFunc(WindowDisplay); // Call WindowDisplay() whenever new frame needed
	glutKeyboardFunc(kbHandler);	// Keyboard handler function
}

void kbHandler(unsigned char key, int x, int y)
{
	if (key == 'q')
	{
		free(catSprt.imgdata);
		free(mouseSprt.imgdata);
		free(cheeseSprt.imgdata);
		free(maze.imgdata);
		free(framebuffer.imgdata);
		exit(0);
	}
}

void WindowReshape(int width, int height)
{
	/*
	  This function is called whenever the window is resized. It takes care of setting up
	  OpenGL world-to-pixel-coordinate conversion matrices so that the image content is
	  properly displayed regardless of window size.

	  The width and height are provided by OpenGL's GLUT library upon the window being resized
	*/

	// Setup projection matrix for new window
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, 1024, 1024, 0);
	glViewport(0, 0, width, height);
	glutPostRedisplay();
}

void WindowDisplay(void)
{
	/*
	   This function is called whenever the frame needs to be updated. That means
	   once per each iteration of the glutMainLoop, as well as anytime that the
	   window is resized.

	 Unfortunately, GLUT's display function prototype allows no arguments! that means
	 any data initialized by main() and needed for the game has to be made global
		 or we would not be able to use it here. Apologies to Anya.
	*/
	static int frame = 0;
	static GLuint texture;
	static int n_angry_frames = 0;
	static int l_angry_frames = 0;
	static int endgame = 0;
	static int end_frames = 0;
	static int power_mode = 0;
	int loc_path[graph_size][2];
	static int DFSPath[graph_size][2]; // DFS fiddles if we go back and forth
	static int DFS_frames = 0;
	int tmpPath[1024][2], minimax_util;
	char line[10];

	for (int i = 0; i < 1024; i++)
	{
		tmpPath[i][0] = -1;
		tmpPath[i][1] = -1;
	}

	/***** Game state update ***********/
	memset(&grid_value[0][0], 0, size_X * size_Y * sizeof(double)); // Reset visiting order
	for (int i = 0; i < graph_size; i++)
	{
		Path[i][0] = -1;
		Path[i][1] = -1;
	}

	if (endgame == 0) // Game still on!
	{
		// Call student code to obtain a plan for the mouse's next move based on the
		// selected search mode
		switch (s_mode)
		{
		case 0:
			// A* with ZERO heuristic
			Heuristic_Search(zero);
			memcpy(tmpPath, Path, 1024 * 2 * sizeof(int));
			break;
		case 1:
			// A* with simple (admissible) heuristic
			Heuristic_Search(H_cost);
			memcpy(tmpPath, Path, 1024 * 2 * sizeof(int));
			break;
		case 2:
		{
			// A* with no-kitty heuristic
			Heuristic_Search(H_cost_nokitty);
			memcpy(tmpPath, Path, 1024 * 2 * sizeof(int));
			break;
		}
		case 3:
		{
			// MiniMax
			minimax_util = MiniMax(cats, n_cats, cheese, n_cheese, mouse, 0, utility, 0, 0, md, -50000, 50000);
			mouse[0][0] = Path[0][0];
			mouse[0][1] = Path[0][1];
			break;
		}
		case 4:
		{
			// MiniMax w. alpha-beta pruning
			minimax_util = MiniMax(cats, n_cats, cheese, n_cheese, mouse, 1, utility, 0, 0, md, -50000, 50000);
			mouse[0][0] = Path[0][0];
			mouse[0][1] = Path[0][1];
			break;
		}
		default:
		{
			fprintf(stderr, "That mode has not been implemented\n");
			Path[1][0] = mouse[0][0];
			Path[1][1] = mouse[0][1];
			break;
		}
		}
		// Update positions for mouse (first!), cats, and if the power cheese is on the
		// game field, the power cheese too.
		// At this point we also check for -> cats eating mouse, mouse eating cat, mouse eating cheese,
		//  all cheese eaten.
		// Note that all updates are done using the local (shadow) copies of the game-state variables.

		// Mouse update - from Path[1][:] - these are the only state variables that can change due to
		// functions in the search code.
		if (Path[1][0] != -1)
		{
			mouse[0][0] = Path[1][0];
			mouse[0][1] = Path[1][1];
		}
		else
		{
			if (s_mode < 3)
				fprintf(stderr, "No path! staying in place!\n");
		}
		// Check for termination to avoid cross-overs
		for (int i = 0; i < n_cats; i++)
			if (mouse[0][0] == cats[i][0] && mouse[0][1] == cats[i][1])
			{
				fprintf(stderr, "Mouse lunch!\n");
				endgame = 2;
			}

		// Cat position updates
		for (int i = 0; i < n_cats; i++)
			catMoves(i, cattitude);

		// Check for mouse win
		//  for (int i=0; i<ncheese; i++)
		for (int i = n_cheese - 1; i >= 0; i--)
		{
			if (mouse[0][0] == cheese[i][0] && mouse[0][1] == cheese[i][1])
			{
				if (n_cheese > 0)
				{
					cheese[i][0] = cheese[n_cheese - 1][0];
					cheese[i][1] = cheese[n_cheese - 1][1];
				}
				n_cheese--;
				if (n_cheese == 0)
					endgame = 1;
			}
		}

		// Check for termination now cat is on top of mouse
		for (int i = 0; i < n_cats; i++)
			if (mouse[0][0] == cats[i][0] && mouse[0][1] == cats[i][1])
			{
				fprintf(stderr, "Mouse lunch!\n");
				endgame = 2;
			}
	} // end if (endgame==0)
	else
	{
		if (endgame == 2) // Mouse lost
		{
			if (end_frames == 0)
				fprintf(stderr, "Oh no! poor poor mouse!\n");
			if (frame % 2)
				renderMaze(&maze, Graph, 255, 0, 0);
			else
				renderMaze(&maze, Graph, 0, 0, 255);
		}
		else // Mouse won
		{
			if (end_frames == 0)
				fprintf(stderr, "Happy, well-fed mouse!\n");
			if (frame % 2)
				renderMaze(&maze, Graph, 0, 255, 0);
			else
				renderMaze(&maze, Graph, 0, 0, 255);
		}
		end_frames++;
		if (end_frames >= 10)
		{
			free(catSprt.imgdata);
			free(mouseSprt.imgdata);
			free(cheeseSprt.imgdata);
			free(maze.imgdata);
			free(framebuffer.imgdata);
			exit(0);
		}
	}

	// Finally, render the current game state and display
	updateFrame(tmpPath);

	// Pause for display
	system("sleep .1");

	/***** End of game state update ****/

	/***** Scene drawing start *********/

	glClearColor(0.01f, 0.01f, 0.01f, 1.0f);
	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Set up texture only the first time through this function
	glEnable(GL_TEXTURE_2D);
	if (frame == 0)
	{
		glGenTextures(1, &texture);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glBindTexture(GL_TEXTURE_2D, texture);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_RGB, GL_UNSIGNED_BYTE, framebuffer.imgdata);
		frame++;
	}
	else // Afterwards just update texture - significantly faster
	{
		glBindTexture(GL_TEXTURE_2D, texture);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 1024, 1024, GL_RGB, GL_UNSIGNED_BYTE, framebuffer.imgdata);
		frame++;
	}
	// Single quad polygon canvas taking up the whole image
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(0.0, 0.0, 0.0);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(1024.0, 0.0, 0.0);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(1024.0, 1024.0, 0.0);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(0.0, 1028.0, 0.0);
	glEnd();

	glFlush();
	glutSwapBuffers();

	/***** Scene drawing end ***********/

	// Tell glut window to update itself
	glutSetWindow(windowID);
	glutPostRedisplay();
}