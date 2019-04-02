# searchAgents.py
# ---------------
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
This file contains all of the agents that can be selected to control Pacman.  To
select an agent, use the '-p' option when running pacman.py.  Arguments can be
passed to your agent using '-a'.  For example, to load a SearchAgent that uses
depth first search (dfs), run the following command:

> python pacman.py -p SearchAgent -a fn=depthFirstSearch

Commands to invoke other search strategies can be found in the project
description.

Please only change the parts of the file you are asked to.  Look for the lines
that say

"*** YOUR CODE HERE ***"

The parts you fill in start about 3/4 of the way down.  Follow the project
description for details.

Good luck and happy searching!
"""

import time

from game import Directions
from game import Agent
from game import Actions
from search import aStarSearch
import util
import search


class GoWestAgent(Agent):
    "An agent that goes West until it can't."

    def getAction(self, state):
        "The agent receives a GameState (defined in pacman.py)."
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP


#######################################################
# This portion is written for you, but will only work #
# after you fill in parts of search.py          #
#######################################################

class SearchAgent(Agent):
    """
    This very general search agent finds a path using a supplied search
    algorithm for a supplied search problem, then returns actions to follow that
    path.

    As a default, this agent runs DFS on a PositionSearchProblem to find
    location (1,1)

    Options for fn include:
      depthFirstSearch or dfs
      breadthFirstSearch or bfs


    Note: You should NOT change any code in SearchAgent
    """

    def __init__(self, fn='depthFirstSearch', prob='PositionSearchProblem',
                 heuristic='nullHeuristic'):
        # Warning: some advanced Python magic is employed below to find the right functions and problems

        # Get the search function from the name and heuristic
        if fn not in dir(search):
            raise AttributeError, fn + ' is not a search function in search.py.'
        func = getattr(search, fn)
        if 'heuristic' not in func.func_code.co_varnames:
            print('[SearchAgent] using function ' + fn)
            self.searchFunction = func
        else:
            if heuristic in globals().keys():
                heur = globals()[heuristic]
            elif heuristic in dir(search):
                heur = getattr(search, heuristic)
            else:
                raise AttributeError, heuristic + ' is not a function in searchAgents.py or search.py.'
            print('[SearchAgent] using function %s and heuristic %s' % (
                fn, heuristic))
            # Note: this bit of Python trickery combines the search algorithm and the heuristic
            self.searchFunction = lambda x: func(x, heuristic=heur)

        # Get the search problem type from the name
        if prob not in globals().keys() or not prob.endswith('Problem'):
            raise AttributeError, prob + ' is not a search problem type in SearchAgents.py.'
        self.searchType = globals()[prob]
        print('[SearchAgent] using problem type ' + prob)

    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        if self.searchFunction == None: raise Exception, "No search function provided for SearchAgent"
        starttime = time.time()
        problem = self.searchType(state)  # Makes a new search problem
        self.actions = self.searchFunction(problem)  # Find a path
        totalCost = problem.getCostOfActions(self.actions)
        print('Path found with total cost of %d in %.1f seconds' % (
            totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print(
            'Search nodes expanded: %d' % problem._expanded)

    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """
        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP


class PositionSearchProblem(search.SearchProblem):
    """
    A search problem defines the state space, start state, goal test, successor
    function and cost function.  This search problem can be used to find paths
    to a particular point on the pacman board.

    The state space consists of (x,y) positions in a pacman game.

    Note: this search problem is fully specified; you should NOT change it.
    """

    def __init__(self, gameState, costFn=lambda x: 1, goal=(1, 1), start=None,
                 warn=True, visualize=True):
        """
        Stores the start and goal.

        gameState: A GameState object (pacman.py)
        costFn: A function from a search state (tuple) to a non-negative number
        goal: A position in the gameState
        """
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        if start != None: self.startState = start
        self.goal = goal
        self.costFn = costFn
        self.visualize = visualize
        if warn and (
                        gameState.getNumFood() != 1 or not gameState.hasFood(
                        *goal)):
            print
            'Warning: this does not look like a regular search maze'

        # For display purposes
        self._visited, self._visitedlist, self._expanded = {}, [], 0  # DO NOT CHANGE

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal = state == self.goal

        # For display purposes only
        if isGoal and self.visualize:
            self._visitedlist.append(state)
            import __main__

            if '_display' in dir(__main__):
                if 'drawExpandedCells' in dir(
                        __main__._display):  # @UndefinedVariable
                    __main__._display.drawExpandedCells(
                        self._visitedlist)  # @UndefinedVariable

        return isGoal

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
             For a given state, this should return a list of triples,
         (successor, action, stepCost), where 'successor' is a
         successor to the current state, 'action' is the action
         required to get there, and 'stepCost' is the incremental
         cost of expanding to that successor
        """

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST,
                       Directions.WEST]:
            x, y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append(( nextState, action, cost))

        # Bookkeeping for display purposes
        self._expanded += 1  # DO NOT CHANGE
        if state not in self._visited:
            self._visited[state] = True
            self._visitedlist.append(state)

        return successors

    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions. If those actions
        include an illegal move, return 999999.
        """
        if actions == None: return 999999
        x, y = self.getStartState()
        cost = 0
        for action in actions:
            # Check figure out the next state and see whether its' legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x, y))
        return cost


class StayEastSearchAgent(SearchAgent):
    """
    An agent for position search with a cost function that penalizes being in
    positions on the West side of the board.

    The cost function for stepping into a position (x,y) is 1/2^x.
    """

    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: .5 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn,
                                                              (1, 1), None,
                                                              False)


class StayWestSearchAgent(SearchAgent):
    """
    An agent for position search with a cost function that penalizes being in
    positions on the East side of the board.

    The cost function for stepping into a position (x,y) is 2^x.
    """

    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: 2 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn)


def manhattanHeuristic(position, problem, info={}):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])


def euclideanHeuristic(position, problem, info={}):
    "The Euclidean distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5


#####################################################
# This portion is incomplete.  Time to write code!  #
#####################################################

class CornersState:
    def __init__(self, position, corners, leftCorners):
        self._corners = corners
        self._position = position
        self._leftCorners = leftCorners
        if position in self._corners and self._corners[position] == 0:
            self._corners.pop(position, None)
            self._leftCorners -= 1

    @property
    def corners(self):
        return self._corners

    @property
    def position(self):
        return self._position

    @property
    def leftCorners(self):
        return self._leftCorners

    def __eq__(self, other):
        return self.position == other.position and self.corners.keys() == other.corners.keys()


class CornersProblem(search.SearchProblem):
    """
    This search problem finds paths through all four corners of a layout.

    You must select a suitable state space and successor function
    """

    def __init__(self, startingGameState):
        """
        Stores the walls, pacman's starting position and corners.
        """
        self.reachedCorners = {}
        self.walls = startingGameState.getWalls()
        self.startingPosition = startingGameState.getPacmanPosition()
        top, right = self.walls.height - 2, self.walls.width - 2
        self.corners = ((1, 1), (1, top), (right, 1), (right, top))
        for corner in self.corners:
            if not startingGameState.hasFood(*corner):
                print
                'Warning: no food in corner ' + str(corner)
            else:
                self.reachedCorners[corner] = 0
        self._expanded = 0  # DO NOT CHANGE; Number of search nodes expanded
        # Please add any code here which you would like to use
        # in initializing the problem

    def getStartState(self):
        """
        Returns the start state (in your state space, not the full Pacman state
        space)
        """
        "*** YOUR CODE HERE ***"
        return CornersState(self.startingPosition, self.reachedCorners,
                            len(self.reachedCorners))

    def isGoalState(self, state):
        """
        Returns whether this search state is a goal state of the problem.
        """
        "*** YOUR CODE HERE ***"
        return state.leftCorners == 0

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
            For a given state, this should return a list of triples, (successor,
            action, stepCost), where 'successor' is a successor to the current
            state, 'action' is the action required to get there, and 'stepCost'
            is the incremental cost of expanding to that successor
        """

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST,
                       Directions.WEST]:
            x, y = state.position[0], state.position[1]
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            hitsWall = self.walls[nextx][nexty]
            if not hitsWall:
                newPosition = (int(x + dx), int(y + dy))
                successor = (CornersState(newPosition, state.corners.copy(),
                                          state.leftCorners), action, 1)
                successors.append(successor)

        self._expanded += 1  # DO NOT CHANGE
        return successors

    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999.  This is implemented for you.
        """
        if actions == None:
            return 999999
        x, y = self.startingPosition
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
        return len(actions)


def cornersHeuristic(state, problem):
    """
    A heuristic for the CornersProblem that you defined.

      state:   The current search state
               (a data structure you chose in your search problem)

      problem: The CornersProblem instance for this layout.

    This function should always return a number that is a lower bound on the
    shortest path from the state to a goal of the problem; i.e.  it should be
    admissible (as well as consistent).
    """

    corners = state.corners.keys()[:]
    walls = problem.walls  # These are the walls of the maze, as a Grid (game.py)

    result = 0
    lastPosition = state.position
    while len(corners) > 0:
        minDistance = 99999
        minPosition = None
        for curPosition in corners:
            distance = manhattanDistance(lastPosition, curPosition)
            if distance <= minDistance:
                minDistance = distance
                minPosition = curPosition
        if curPosition is not None:
            lastPosition = minPosition
            corners.remove(lastPosition)
            result += minDistance

    return result


def manhattanDistance(position1, position2):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position1
    xy2 = position2
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])


#####################################################
# Q7
#####################################################


class FoodSearchProblem:
    """
    A search problem associated with finding the a path that collects all of the
    food (dots) in a Pacman game.

    A search state in this problem is a tuple ( pacmanPosition, foodGrid ) where
      pacmanPosition: a tuple (x,y) of integers specifying Pacman's position
      foodGrid:       a Grid (see game.py) of either True or False, specifying remaining food
    """

    def __init__(self, startingGameState):
        self.start = (
            startingGameState.getPacmanPosition(), startingGameState.getFood())
        self.walls = startingGameState.getWalls()
        self.startingGameState = startingGameState
        self._expanded = 0  # DO NOT CHANGE
        self.heuristicInfo = {}  # A dictionary for the heuristic to store information

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count() == 0

    def getSuccessors(self, state):
        "Returns successor states, the actions they require, and a cost of 1."
        successors = []
        self._expanded += 1  # DO NOT CHANGE
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST,
                          Directions.WEST]:
            x, y = state[0]
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextFood = state[1].copy()
                nextFood[nextx][nexty] = False
                successors.append((((nextx, nexty), nextFood), direction, 1))
        return successors

    def getCostOfActions(self, actions):
        """Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999"""
        x, y = self.getStartState()[0]
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost


class FoodSubSearchState:
    def __init__(self, position, leftFood):
        self._position = position
        self._leftFood = leftFood

    @property
    def position(self):
        return self._position

    @property
    def leftFood(self):
        return self._leftFood

    def __eq__(self, other):
        return self.position == other.position and self.leftFood == other.leftFood


class FoodSubSearchProblem:
    """
    A search problem associated with finding the a path that collects all of the
    food (dots) in a Pacman game.

    A search state in this problem is a tuple ( pacmanPosition, foodGrid ) where
      pacmanPosition: a tuple (x,y) of integers specifying Pacman's position
      foodGrid:       a Grid (see game.py) of either True or False, specifying remaining food
    """

    def __init__(self, pacmanPostion, leftFood, verticalWalls, horizontalWalls):
        self.food = leftFood
        self.start = FoodSubSearchState(pacmanPostion, leftFood)
        self.verticalWalls = verticalWalls
        self.horizontalWalls = horizontalWalls
        self._expanded = 0  # DO NOT CHANGE
        self._result = []
        self._checked = {}

    @property
    def result(self):
        return self._result

    @result.setter
    def result(self, result):
        self._result = result

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return len(state.leftFood) == 0

    def getSuccessors(self, state):
        "Returns successor states, the actions they require, and a cost of 1."
        successors = []
        self._expanded += 1  # DO NOT CHANGE
        shortDistance = 99999
        shortNode = None
        for food in state.leftFood:
            distance = 0
            if (state.position, food) in self._checked or (
                    food, state.position) in self._checked:
                distance = self._checked[(state.position, food)]
            else:
                distance = manhattanDistanceWithWall(state.position, food,
                                                     self.horizontalWalls,
                                                     self.verticalWalls)
                self._checked[(state.position, food)] = distance
                self._checked[(food, state.position)] = distance
            newLeftFood = state.leftFood[:]
            newLeftFood.remove(food)
            successors.append(
                (FoodSubSearchState(food, newLeftFood), food, distance))
        return successors

    def getCostOfActions(self, actions):
        return self.getCostOfActions(self.start.position, actions)

    def getCostOfActions(self, currentPostion, actions):
        """Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999"""
        start = currentPostion
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's legal
            cost += manhattanDistanceWithWall(start, action,
                                              self.horizontalWalls,
                                              self.verticalWalls)
            start = action
        return cost


class AStarFoodSearchAgent(SearchAgent):
    "A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"

    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob,
                                                              foodHeuristic)
        self.searchType = FoodSearchProblem


def foodHeuristic2(state, problem):
    """
    Your heuristic for the FoodSearchProblem goes here.

    This heuristic must be consistent to ensure correctness.  First, try to come
    up with an admissible heuristic; almost all admissible heuristics will be
    consistent as well.

    If using A* ever finds a solution that is worse uniform cost search finds,
    your heuristic is *not* consistent, and probably not admissible!  On the
    other hand, inadmissible or inconsistent heuristics may find optimal
    solutions, so be careful.

    The state is a tuple ( pacmanPosition, foodGrid ) where foodGrid is a Grid
    (see game.py) of either True or False. You can call foodGrid.asList() to get
    a list of food coordinates instead.

    If you want access to info like walls, capsules, etc., you can query the
    problem.  For example, problem.walls gives you a Grid of where the walls
    are.

    If you want to *store* information to be reused in other calls to the
    heuristic, there is a dictionary called problem.heuristicInfo that you can
    use. For example, if you only want to count the walls once and store that
    value, try: problem.heuristicInfo['wallCount'] = problem.walls.count()
    Subsequent calls to this heuristic can access
    problem.heuristicInfo['wallCount']
    """
    position, foodGrid = state
    "*** YOUR CODE HERE ***"

    if 'foodCordinates' not in problem.heuristicInfo:
        foodCordinates = []
        for i in range(foodGrid.width):
            for j in range(foodGrid.height):
                if foodGrid[i][j] is True:
                    foodCordinates.append((i, j))
        problem.heuristicInfo['foodCordinates'] = foodCordinates

    if 'verticalWalls' not in problem.heuristicInfo:
        verticalWalls = []
        for i in range(problem.walls.width):
            start = -1
            finish = -1
            verticalWalls.append([])
            for j in range(problem.walls.height):
                if problem.walls[i][j] is True:
                    if start == -1:
                        start = j
                    finish += 1
                elif finish > -1:
                    verticalWalls[i].append(VerticalWall(start, start + finish))
                    start = -1
                    finish = -1
            verticalWalls[i].append(VerticalWall(start, start + finish))
        problem.heuristicInfo['verticalWalls'] = verticalWalls

    if 'horizontalWalls' not in problem.heuristicInfo:
        horizontalWalls = []
        for i in range(problem.walls.height):
            start = -1
            finish = -1
            horizontalWalls.append([])
            for j in range(problem.walls.width):
                if problem.walls[j][i] is True:
                    if start == -1:
                        start = j
                    finish += 1
                elif finish > -1:
                    horizontalWalls[i].append(
                        HorizontalWall(start, start + finish))
                    start = -1
                    finish = -1
            horizontalWalls[i].append(HorizontalWall(start, start + finish))
        problem.heuristicInfo['horizontalWalls'] = horizontalWalls

    # Remove eated food from foodCordinates
    verticalWalls = problem.heuristicInfo['verticalWalls']
    horizontalWalls = problem.heuristicInfo['horizontalWalls']
    foodCordinates = problem.heuristicInfo['foodCordinates'][:]
    eatedFood = []
    for food in foodCordinates:
        if foodGrid[food[0]][food[1]] is False:
            eatedFood.append(food)
    for food in eatedFood:
        foodCordinates.remove(food)

    if foodCordinates is None or len(foodCordinates) == 0:
        return 0

    if 'heuristicProblem' not in problem.heuristicInfo:
        foodToRemove = FoodSurrounded.foodToRemove(problem.walls, foodCordinates[:])
        foodToFind = foodCordinates[:]
        for f in foodToRemove:
            if f in foodToFind:
                foodToFind.remove(f)
        heuristicProblem = FoodSubSearchProblem(position, foodToFind, verticalWalls, horizontalWalls)
        heuristicProblem.result = aStarSearch(heuristicProblem, lambda node, problem: len(node.leftFood))
        print 'Expanded', heuristicProblem._expanded
        problem.heuristicInfo['heuristicProblem'] = heuristicProblem


    heuristicProblem = problem.heuristicInfo['heuristicProblem']
    heuristicProblemResult = heuristicProblem.result[:]
    for eated in eatedFood:
        if eated in heuristicProblemResult:
            heuristicProblemResult.remove(eated)

    result = heuristicProblem.getCostOfActions(position, heuristicProblemResult)
    return result


class Wall:
    def __init__(self, fromD, toD):
        self._fromD = fromD
        self._toD = toD

    @property
    def fromD(self):
        return self._fromD

    @property
    def toD(self):
        return self._toD

    def isCroosTheWall(self, p):
        return self.fromD <= p <= self.toD

    def size(self):
        return abs(self.fromD - self.toD)


class HorizontalWall(Wall):
    def __init__(self, fromD, toD):
        Wall.__init__(self, fromD, toD)

    def isCroosTheLine(self, point):
        return Wall.isCroosTheWall(self, point[0])


class VerticalWall(Wall):
    def __init__(self, fromD, toD):
        Wall.__init__(self, fromD, toD)

    def isCroosTheLine(self, point):
        return Wall.isCroosTheWall(self, point[1])


class FoodSurrounded:
    @staticmethod
    def positions(position):
        x = 0
        y = 1
        positions = {
            0: (position[x], position[y] + 1),
            1: (position[x] + 1, position[y] + 1),
            2: (position[x] + 1, position[y]),
            3: (position[x] + 1, position[y] - 1),
            4: (position[x], position[y] - 1),
            5: (position[x] - 1, position[y] - 1),
            6: (position[x] - 1, position[y]),
            7: (position[x] - 1, position[y] + 1)
        }
        return positions

    @staticmethod
    def isSurrounded(position, walls):
        """

        :rtype : tuple
        """
        surroundedTab = ''
        positions = FoodSurrounded.positions(position)
        positionsKeys = list(positions.keys())
        positionsKeys.sort()
        for k in positionsKeys:
            surroundedTab += FoodSurrounded.isTheWallInThisPosition(walls, positions[k])

        tmpStr = ("".join(surroundedTab)) * 2
        idx = tmpStr.find('XXXXX')
        if idx > -1:
            if tmpStr[0: 0 + 5] == 'XXXXX':
                return position, positions[6]
            elif tmpStr[2: 2 + 5] == 'XXXXX':
                return position, positions[0]
            elif tmpStr[4: 4+5] == 'XXXXX':
                return position, positions[2]
            elif tmpStr[6: 6 + 5] == 'XXXXX':
                return position, positions[4]
        return None, None


    @staticmethod
    def foodToRemove(walls, foodPositions):
        surroundedFoods = []
        for food in foodPositions:
            surroundedFood = FoodSurrounded.isSurrounded(food, walls)
            if surroundedFood != (None, None):
                surroundedFoods.append(surroundedFood)
                foodPositions.remove(food)

        foodToRemove = []
        for k in surroundedFoods:
            food = k[0]
            nextFood = k[1]
            foodToRemove.append(nextFood)
            while nextFood in foodPositions:
                from copy import copy, deepcopy
                tmpWalls = deepcopy(walls)
                tmpWalls[food[0]][food[1]] = True
                food, nextFood = FoodSurrounded.isSurrounded(nextFood, tmpWalls)
                if food is not None:
                    foodToRemove.append(food)

        return foodToRemove

    @staticmethod
    def isTheWallInThisPosition(walls, position):
        """

        :rtype : str
        """
        x = 0
        y = 1
        if walls[position[x]][position[y]]:
            return 'X'
        else:
            return '0'

    @staticmethod
    def isTheFoodInThisPosition(foods, position):
        x = 0
        y = 1
        return foods[position[x]][position[y]]


def manhattanDistanceWithWall(position1, position2, horizontalWalls,
                              verticalWalls):
    # The Manhattan distance heuristic for a PositionSearchProblem
    x = 0
    y = 1

    pathV = 0
    step = 1 if position1[x] < position2[x] else -1
    for i in range(position1[x], position2[x], step):
        for xWall in verticalWalls[i]:
            if xWall.isCroosTheWall(position1[y]) and xWall.isCroosTheWall(
                    position2[y]):
                if xWall.fromD == 0:
                    pathV += xWall.toD - position1[y] + xWall.toD - position2[
                        y] + 2
                elif xWall.toD == 0:
                    pathV += xWall.fromD - position1[y] + xWall.fromD - \
                             position2[y] + 2
                else:
                    firstPath = abs(xWall.toD - position1[y]) + abs(
                        xWall.toD - position2[y]) + 1
                    secondPath = abs(xWall.fromD - position1[y]) + abs(
                        xWall.fromD - position2[y]) + 1
                    pathV += min(firstPath, secondPath) + abs(
                        position1[x] - position2[x])
                break
        else:
            continue
        break

    pathH = 0
    step = 1 if position1[y] < position2[y] else -1
    for i in range(position1[y], position2[y], step):
        for xWall in horizontalWalls[i]:
            if xWall.isCroosTheWall(position1[x]) and xWall.isCroosTheWall(
                    position2[x]):
                if xWall.fromD == 0:
                    pathH = xWall.toD - position1[x] + xWall.toD - position2[
                        x] + 2
                elif xWall.toD == 0:
                    pathH = xWall.fromD - position1[x] + xWall.fromD - \
                            position2[x] + 2
                else:
                    firstPath = abs(xWall.toD - position1[x]) + abs(
                        xWall.toD - position2[x]) + 1
                    secondPath = abs(xWall.fromD - position1[x]) + abs(
                        xWall.fromD - position2[x]) + 1
                    pathH = min(firstPath, secondPath) + abs(
                        position1[y] - position2[y])
                break
        else:
            continue
        break

    if pathV != 0 or pathH != 0:
        return max(pathV, pathH)
    else:
        return abs(position1[x] - position2[x]) + abs(
            position1[y] - position2[y])


class ClosestDotSearchAgent(SearchAgent):
    "Search for all food using a sequence of searches"

    def registerInitialState(self, state):
        self.actions = []
        currentState = state
        while (currentState.getFood().count() > 0):
            nextPathSegment = self.findPathToClosestDot(currentState)  # The missing piece
            self.actions += nextPathSegment
            for action in nextPathSegment:
                legal = currentState.getLegalActions()
                if action not in legal:
                    t = (str(action), str(currentState))
                    raise Exception, 'findPathToClosestDot returned an illegal move: %s!\n%s' % t
                currentState = currentState.generateSuccessor(0, action)
        self.actionIndex = 0
        print 'Path found with cost %d.' % len(self.actions)

    def findPathToClosestDot(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        startPosition = gameState.getPacmanPosition()
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState)

        anyFoodSearchProblem = AnyFoodSearchProblem(gameState)
        result = aStarSearch(anyFoodSearchProblem)
        return result

class AnyFoodSearchProblem(PositionSearchProblem):
    """
    A search problem for finding a path to any food.

    This search problem is just like the PositionSearchProblem, but has a
    different goal test, which you need to fill in below.  The state space and
    successor function do not need to be changed.

    The class definition above, AnyFoodSearchProblem(PositionSearchProblem),
    inherits the methods of the PositionSearchProblem.

    You can use this search problem to help you fill in the findPathToClosestDot
    method.
    """

    def __init__(self, gameState):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0  # DO NOT CHANGE

    def isGoalState(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x, y = state
        return self.food[x][y]


def mazeDistance(point1, point2, gameState):
    """
    Returns the maze distance between any two points, using the search functions
    you have already built. The gameState can be any game state -- Pacman's
    position in that state is ignored.

    Example usage: mazeDistance( (2,4), (5,6), gameState)

    This might be a useful helper function for your ApproximateSearchAgent.
    """
    x1, y1 = point1
    x2, y2 = point2
    walls = gameState.getWalls()
    assert not walls[x1][y1], 'point1 is a wall: ' + str(point1)
    assert not walls[x2][y2], 'point2 is a wall: ' + str(point2)
    prob = PositionSearchProblem(gameState, start=point1, goal=point2,
                                 warn=False, visualize=False)
    return len(search.bfs(prob))