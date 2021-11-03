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
    startTuple = (start, [])
    open = util.Queue()             # States to check
    closed = []                     # Closed states
    closed.append(start)
    open.push(startTuple)
    
    while not open.isEmpty():       # While there are remaining states
        node = open.pop()
        current_state = node[0]      
        actions = node[1]               # list of actions
        if problem.isGoalState(current_state):  # Exits when reached goal state
            return actions
        
        successors = problem.getSuccessors(current_state)  # Generate children
        for s in successors:
            s_state = s[0]
            if not s_state in closed: # Check if children not checked already
                s_action = s[1]
                closed.append(s_state)
                open.push((s_state, actions + [s_action]))
    return actions
    
                   
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
