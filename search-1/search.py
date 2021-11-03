# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from sys import _current_frames
import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    open = []
    closed = [start]
    path = []
    currentpath = []

    first_children = problem.getSuccessors(start)
    for tuples in first_children:
        open.insert(0, tuples[0])
        path.insert(0, tuples[1])

    while open != []:
        current_state = open[0]
        currentpath.append(path[0])
        # print("Current State is: ", current_state)
        # Remove leftmost State from open, after defining it as current_state
        open.pop(0)
        path.pop(0)
        # print("Open Path is: ", open)

        if problem.isGoalState(current_state):
            return currentpath
        else:
            # Generate childen of current_state
            successors = problem.getSuccessors(current_state)
            # print("Successors are: ", successors)
            
            for tuples in successors:
                # If children are not in open or closed, add to left end of open
                if tuples[0] not in closed:
                    open.insert(0, tuples[0])
                    path.insert(0, tuples[1])
                else:
                    # Add current_state to closed
                    closed.append(current_state)
                    # print("Closed path is: ", closed)
            if len(successors) == 0:
                print(current_state)
                currentpath.pop(0)

    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    exploredState = []
    exploredState.append(start)
    states = util.Queue()
    stateTuple = (start, [])
    states.push(stateTuple)
    while not states.isEmpty():
        state, action = states.pop()
        if problem.isGoalState(state):
            return action
        successor = problem.getSuccessors(state)
        for i in successor:
            coordinates = i[0]
            if not coordinates in exploredState:
                direction = i[1]
                exploredState.append(coordinates)
                states.push((coordinates, action + [direction]))
    return action
    # current_state = problem.getStartState()
    
    # nodes_visited = []                      # Keeps track of nodes that have been visited
    # nodes_visited.append(current_state)
    # nodes_visit_path = []                   # nodes_visited but removes unwanted nodes
    # nodes_visit_path.append(current_state)
    # potential_actions = []                  # Contains successors for potential actions to take
    
    # successors_to_visit = []                # Successors of start state to visit
    # successors = problem.getSuccessors(current_state)
    # # Get the successors of the start state
    # for i in successors:
    #     successors_to_visit.append(i)
        
    # # While not empty
    # while successors_to_visit:
    #     # Keep track of what we are currently processing
    #     current_successor = successors_to_visit[0]
    #     successors_to_visit.remove(current_successor)
    #     current_state = current_successor[0]
    #     nodes_visited.append(current_state)
    #     nodes_visit_path.append(current_state)
        
    #     # Found
    #     if problem.isGoalState(current_state):
    #         potential_actions.append(current_successor) 
    #         path = []
    #         #  Filter paths from potential paths
    #         for i in range(len(nodes_visit_path) - 1):
    #             a = nodes_visit_path[i]
    #             b = nodes_visit_path[i + 1]
    #             for j in potential_actions:
    #                 # If state is in potential action, then that action is the valid action
    #                 if j[0] == b:
    #                     path.append(a)
    #                     break  
    #             if i == len(nodes_visit_path) - 2:
    #                 path.append(b)
    #         actions = []        
    #         # Filter actions from potential_actions
    #         for i in range(len(path) - 1):
    #             a = path[i]
    #             b = path[i + 1]
    #             for j in potential_actions:
    #                 if j[0] == b:
    #                     actions.append(j[1])
    #                     break
    #         return actions
        
    #     # Converts successors_to_visit to just the states so that we can check 
    #     # if successors are potentially visited 
    #     nodes_to_visit = []
    #     for s in successors_to_visit:
    #         nodes_to_visit.append(s[0])
        
    #     # Process through the successors of the current state
    #     successors = problem.getSuccessors(current_state)
    #     remove = []
    #     for i in successors:
    #         # Not been involved yet
    #         if i[0] not in nodes_visited and i[0] not in nodes_to_visit:
    #             potential_actions.append(current_successor)
    #             successors_to_visit.append(i)
    #         else: 
    #             remove.append(i)
    #     for i in remove:
    #         successors.remove(i)
            
    #     # There are no valid successors
    #     if not successors:
    #         # Remove last element from the path, it is a deadend
    #         nodes_visit_path.remove(nodes_visit_path[-1])
                   
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    open = util.PriorityQueue()     # stores tuple of states + action
    open.push((problem.getStartState(), []), nullHeuristic(problem.getStartState(), problem))
    closed = []                     # stores states
    
    # while open is not empty
    while not open.isEmpty():
        # current node retrieved from priority queue (lowest cost node)
        current_node = open.pop()
        current_state = current_node[0]
        current_action = current_node[1]
        
        # found
        if problem.isGoalState(current_state):
            return current_action
        
        # if state has not been explored
        if current_state not in closed:
            # then explore successors
            successors = problem.getSuccessors(current_state)
            for s in successors:
                s_state = s[0]
                
                # if the successor has not been explored
                if s_state not in closed:
                    # get all actions
                    s_action = s[1]
                    total_actions = current_action + [s_action]             # all actions including the current one
                    
                    # get cost of all actions
                    total_cost = problem.getCostOfActions(total_actions)    # cost of all actions
                    total_cost += heuristic(s_state, problem)               # including A* heuristic (distance from current state to final state)
                    
                    open.push((s_state, total_actions), total_cost)         # explore later, priority is the total cost
                    
        # explored
        closed.append(current_state)
    
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
