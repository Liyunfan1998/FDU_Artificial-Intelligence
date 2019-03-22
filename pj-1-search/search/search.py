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
    return [s, s, w, s, w, w, s, w]


def add2frontier(node, priority, type, frontier):
    if type == "PUSH_TWO":
        frontier.push(node, priority)
    elif type == "PUSH_ONE":
        frontier.push(node)


def genericSearch(problem, frontier, heuristic=0, push_type="PUSH_ONE"):
    """
    state defined as structure node
    node implemented as structure tuple(state, cost, path)
    """
    startState = (problem.getStartState(), 0, [])  # this is a node
    add2frontier(startState, 0, push_type, frontier)

    # using set to make sure that a node can only be updated if it already in explored, not added again into explored
    explored = set()

    if frontier.isEmpty():
        raise Exception('no element in frontier!')
    while not frontier.isEmpty():
        # pop out a node from frontier each time to carry our search on its child-nodes
        (state, cost, path) = frontier.pop()
        if problem.isGoalState(state):
            return path
        # a node can only be in frontier or explored or neither at a certain time
        if state not in explored:
            explored.add(state)

            for child_node in problem.getSuccessors(state):
                # retrieve properties from child_node
                child_state, action, child_cost = child_node
                child_cost += cost
                child_path = path + [action]
                child_node = (child_state, child_cost, child_path)
                h = heuristic(child_node[0], problem)
                add2frontier(child_node, child_cost + h, push_type, frontier)

    return "There is nothing in frontier. Failure!"


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    # util.raiseNotDefined()

    # Using util.Stack to implement frontier in DFS
    frontier = util.Stack()
    return genericSearch(problem, frontier, nullHeuristic, "PUSH_ONE")


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # util.raiseNotDefined()

    # Using util.Queue to implement frontier in BFS
    frontier = util.Queue()
    return genericSearch(problem, frontier, nullHeuristic, "PUSH_ONE")


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    # util.raiseNotDefined()

    # Using util.PriorityQueue to implement frontier in UCS
    frontier = util.PriorityQueue()
    return genericSearch(problem, frontier, nullHeuristic, "PUSH_TWO")


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # util.raiseNotDefined()

    # Using util.Queue to implement frontier in BFS
    frontier = util.PriorityQueue()
    # heuristic(state, problem)
    return genericSearch(problem, frontier, heuristic, "PUSH_TWO")


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
