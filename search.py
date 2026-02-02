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
from game import Directions
from typing import List

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




def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
    
    stacked = util.Stack()
    visited = set()

    start_state = problem.getStartState()
    stacked.push((start_state, []))

    while not stacked.isEmpty():
        state, path = stacked.pop()

        if problem.isGoalState(state):
            return path
        if state not in visited:
            visited.add(state)

            for successor, action, cost in problem.getSuccessors(state):
                if successor not in visited:
                    stacked.push((successor, path + [action]))

    return []

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue

    queue = Queue()
    visited = set()

    start_state = problem.getStartState()
    queue.push((start_state, []))
    visited.add(start_state)

    while not queue.isEmpty():
        state, path = queue.pop()

        if problem.isGoalState(state):
            return path

        for successor, action, cost in problem.getSuccessors(state):
            if successor not in visited:
                visited.add(successor)
                queue.push((successor, path + [action]))

    return []


def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    Ab = util.PriorityQueue()
    Best_cost = {}

    start_state = problem.getStartState()
    Ab.push((start_state, [], 0), 0)
    Best_cost[start_state] = 0
    while not Ab.isEmpty():
        state, path, cost = Ab.pop()

        if problem.isGoalState(state):
            return path

        if cost > Best_cost.get(state, float('inf')):
            continue

        for successor, action, step_cost in problem.getSuccessors(state):
            new_cost = cost + step_cost

            if successor not in Best_cost or new_cost < Best_cost[successor]:
                Best_cost[successor] = new_cost
                Ab.push((successor, path + [action], new_cost), new_cost)

    return []
   

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    from util import PriorityQueue
    peq = util.PriorityQueue()
    best_cost = {}

    start_state = problem.getStartState()
    start_h = heuristic(start_state, problem)
    peq.push((start_state, [], 0), start_h)
    best_cost[start_state] = 0

    while not peq.isEmpty():
        state, path, cost = peq.pop()

        if problem.isGoalState(state):
            return path

        if cost > best_cost.get(state, float('inf')):
            continue

        for successor, action, step_cost in problem.getSuccessors(state):
            new_cost = cost + step_cost

            if successor not in best_cost or new_cost < best_cost[successor]:
                best_cost[successor] = new_cost
                priority = new_cost + heuristic(successor, problem)
                peq.push((successor, path + [action], new_cost), priority)

    return []

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
