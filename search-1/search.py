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
    open = [problem.getStartState()]
    closed = []
    path = []
    # current_paths = []

    # successors = problem.getSuccessors(problem.getStartState())
    # for tuples in successors:
    #     open.append(tuples)
    #     current_paths.append(tuples[1])

    while open != []:
        current_state = open[0]
        # print("Current State is: ", current_state)
        # if current_state != problem.getStartState():
        #     path.append(current_paths[0])
        # Remove leftmost State from open, after defining it as current_state
        open.pop(0)
        # print("Open Path is: ", open)

        if problem.isGoalState(current_state):
            return path
        else:
            # Generate childen of current_state
            successors = problem.getSuccessors(current_state)
            # print("Successors are: ", successors)
            # Add current_state to closed
            closed.append(current_state)
            # print("Closed path is: ", closed)
            for tuples in successors:
                # If children are not in open or closed, add to left end of open
                if tuples[0] not in closed and open:
                    open.insert(0, tuples[0])
                    path.append(tuples[1])
                else:
                    # If path is a dead end
                    path.pop()

    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    current_state = problem.getStartState() # Start state
    
    nodes_visited = []
    nodes_visited.append(current_state)     # Start state is already visited
    nodes_visit_path = []                   # Path has unnecessary elements removed
    nodes_visit_path.append(current_state)
    potential_actions = []                  # Contains successors so you have both action and state
    
    successors_to_visit = []                # Successors of start state to visit
    successors = problem.getSuccessors(current_state)
    for i in successors:
        successors_to_visit.append(i)
        
    # While not empty
    while successors_to_visit:
        current_successor = successors_to_visit[0]
        successors_to_visit.remove(current_successor)
        current_state = current_successor[0]
        # current_action = current_successor[1]
        nodes_visited.append(current_state)
        nodes_visit_path.append(current_state)
        
        # Found
        if problem.isGoalState(current_state):
            potential_actions.append(current_successor) 
            path = []
            #  Filter paths from potential paths
            for i in range(len(nodes_visit_path) - 1):
                a = nodes_visit_path[i]
                b = nodes_visit_path[i + 1]
                
                for j in potential_actions:
                    # if state is in potential action, then that action is the valid action
                    if j[0] == b:
                        path.append(a)
                        break
                
                # successors = problem.getSuccessors(a)
                # for j in successors:
                #     if j[0] == b:
                #         if j[1] in potential_actions:
                #             path.append(a)
                
                # Goal state is the last element
                if i == len(nodes_visit_path) - 2:
                    path.append(b)
            
            actions = []        
            # Filter actions from potential_actions
            for i in range(len(path) - 1):
                a = path[i]
                b = path[i + 1]
                
                for j in potential_actions:
                    if j[0] == b:
                        actions.append(j[1])
                        break
                
                # successors = problem.getSuccessors(a)
                # for j in successors:
                #     if j[0] == b:
                #         actions.append(j[1])
            
            return actions
        
        nodes_to_visit = [] # converts successors_to_visit to just the states
        for s in successors_to_visit:
            nodes_to_visit.append(s[0])
        
        successors = problem.getSuccessors(current_state)
        remove = []
        for i in successors:
            # Not been involved yet
            if i[0] not in nodes_visited and i[0] not in nodes_to_visit:
                potential_actions.append(current_successor)
                successors_to_visit.append(i)
            else: 
                remove.append(i)
        
        for i in remove:
            successors.remove(i)
        # There are no valid successors
        if not successors:
            # Remove last element from the path, it is a deadend
            nodes_visit_path.remove(nodes_visit_path[-1])
                   
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
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
