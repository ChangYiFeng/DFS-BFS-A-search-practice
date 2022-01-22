# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util
import searchAgents

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).

  You do not need to change anything in this class, ever.
  """

  def getStartState(self):
	 """
	 Returns the start state for the search problem
	 """
	 util.raiseNotDefined()

  def isGoalState(self, state):
	 """
	   state: Search state

	 Returns True if and only if the state is a valid goal state
	 """
	 util.raiseNotDefined()

  def getSuccessors(self, state):
	 """
	   state: Search state

	 For a given state, this should return a list of triples,
	 (successor, action, stepCost), where 'successor' is a
	 successor to the current state, 'action' is the action
	 required to get there, and 'stepCost' is the incremental
	 cost of expanding to that successor
	 """
	 util.raiseNotDefined()

  def getCostOfActions(self, actions):
	 """
	  actions: A list of actions to take

	 This method returns the total cost of a particular sequence of actions.  The sequence must
	 be composed of legal moves
	 """
	 util.raiseNotDefined()


def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 85].

  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.7].

  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  """
  DFSstack = util.Stack()       # use Stack structure
  Visited = []                  # avoids expanding any already visited states

  Root = problem.getStartState()
  print "Start:", Root          # initialize and check start state
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  DFSstack.push((Root, []))     # note that DFSstack stores node and paths

  while not DFSstack.isEmpty():             # Returns true if the stack is empty
	CurrentNode, Path = DFSstack.pop()

	if CurrentNode not in Visited:          # avoids expanding any already visited states
	  Visited.append(CurrentNode)
	  if problem.isGoalState(CurrentNode):
		print "Found the goal at ", CurrentNode, "Path: ", Path
		return Path

	  for successor, action, stepCost in problem.getSuccessors(CurrentNode):
		NextAction = Path + [action]               # update paths
		DFSstack.push((successor, NextAction))     # add new node and update paths

def breadthFirstSearch(problem):
	"Search the shallowest nodes in the search tree first. [p 81]"
	BFSQueue = util.Queue()       # use Stack structure
	Visited = []                  # avoids expanding any already visited states

	Root = problem.getStartState()
	print "Start:", Root          # initialize and check start state
	print "Is the start a goal?", problem.isGoalState(problem.getStartState())
	print "Start's successors:", problem.getSuccessors(problem.getStartState())
	BFSQueue.push((Root, []))     # note that DFSstack stores node and paths

	while not BFSQueue.isEmpty():             # Returns true if the stack is empty
		CurrentNode, Path = BFSQueue.pop()

		if CurrentNode not in Visited:          # avoids expanding any already visited states
		  Visited.append(CurrentNode)
		  if problem.isGoalState(CurrentNode):
			# print "Found the goal at ", CurrentNode, "Path: ", Path
			return Path

		  for successor, action, stepCost in problem.getSuccessors(CurrentNode):
			NextAction = Path + [action]               # update paths
			BFSQueue.push((successor, NextAction))     # add new node and update paths

def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()

def nullHeuristic(state, problem):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
	"Search the node that has the lowest combined cost and heuristic first."
	aStar_Search = util.PriorityQueue()
	print "Start:", problem.getStartState()     # print start node

	aStar_Search.push( (problem.getStartState(), [], [], []), 0)

	while not aStar_Search.isEmpty():
		position, path, visited_goal, visited = aStar_Search.pop()

		if problem.isGoalState(position):	# if find Goals
			return path

		for (successor, action, stepCost) in problem.getSuccessors(position):
			if successor not in visited:
				visited += [successor]
				new_path = path + [action]
				cost = len(new_path) + heuristic(successor, visited_goal, problem)
				aStar_Search.push( (successor, new_path, visited_goal, visited), cost)

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
