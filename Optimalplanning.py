 #!/usr/bin/env python
 # Author: Luis G. Torres, Mark Moll
 
import dateutil.parser
from math import sin, cos
from functools import partial
import sys
try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
     # if the ompl module is not in the PYTHONPATH assume it is installed in a
     # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from matplotlib.lines import Line2D
import networkx as nx


def plot_graph(graph_file_path,mat_file_path,arrowlen):
    
    G = nx.read_graphml(graph_file_path)
    
    node_data = {} 
    for n in G.nodes:
        node_data[n] = G.nodes[n]['coords']
    
    edge_data=list(G.edges)
    
        
    ########################plot graph################################
    
    fig=plt.figure()
    fig.clf()
    
    xtotal=np.zeros(shape=G.number_of_nodes())
    ytotal=np.zeros(shape=G.number_of_nodes())
    
    ax = fig.add_subplot(111)
    
    i=0
    for k in node_data:
        xt=float(node_data[k].split(",")[0])
        yt=float(node_data[k].split(",")[1])
        at=float(node_data[k].split(",")[2])
        ax.annotate("",
            xy=(xt+arrowlen*math.cos(at), yt+arrowlen*math.sin(at)), xycoords='data',
            xytext=(xt, yt), textcoords='data',
            arrowprops=dict(arrowstyle="->", connectionstyle="arc3"),)
        xtotal[i]=xt
        ytotal[i]=yt
        i=i+1

    
    plt.scatter(xtotal, ytotal)
    #plt.axis('scaled')
    
    z=len(edge_data)
    
    for i in range(z):
        pt1,pt2=edge_data[i]
        pt1=node_data[pt1].split(",")[0:2]
        pt2=node_data[pt2].split(",")[0:2]
        x1,y1=pt1
        x2,y2=pt2
        line=Line2D([float(x1),float(x2)],[float(y1),float(y2)],color='k') 
        ax.add_line(line)
    
    data=np.loadtxt(mat_file_path)
    xa=data[:,0]
    ya=data[:,1]
    plt.plot(xa,ya,'r')
    circle1 = plt.Circle((0.5, 0.5), 0.25, color='g') 
    ax1 = fig.gca()
    ax1.add_artist(circle1)  
    
    #plt.axis('scaled')
    fig.set_size_inches(19.2, 10.8)
    fig.savefig('hi.png', dpi=100)

 
class ValidityChecker(ob.StateValidityChecker):
    # Returns whether the given state's position overlaps the
    # circular obstacle
    def isValid(self, state):
        return self.clearance(state) > 0.0
 
     # Returns the distance from the given state's position to the
     # boundary of the circular obstacle.
    def clearance(self, state):
         # Extract the robot's (x,y) position from its state
        x = state.getX()
        y = state.getY()
 
        # Distance formula between two points, offset by the circle's radius
        return math.sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25


## Returns a structure representing the optimization objective to use
#  for optimal motion planning. This method returns an objective
#  which attempts to minimize the length in configuration space of
#  computed paths.
def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)

## Returns an optimization objective which attempts to minimize path
#  length that is satisfied when a path of length shorter than 1.51
#  is found.
def getThresholdPathLengthObj(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostThreshold(ob.Cost(1.51))
    return obj

## Defines an optimization objective which attempts to steer the
#  robot away from obstacles. To formulate this objective as a
#  minimization of path cost, we can define the cost of a path as a
#  summation of the costs of each of the states along the path, where
#  each state cost is a function of that state's clearance from
#  obstacles.
#
#  The class StateCostIntegralObjective represents objectives as
#  summations of state costs, just like we require. All we need to do
#  then is inherit from that base class and define our specific state
#  cost function by overriding the stateCost() method.
#
class ClearanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(ClearanceObjective, self).__init__(si, True)
        self.si_ = si

    # Our requirement is to maximize path clearance from obstacles,
    # but we want to represent the objective as a path cost
    # minimization. Therefore, we set each state's cost to be the
    # reciprocal of its clearance, so that as state clearance
    # increases, the state cost decreases.
    def stateCost(self, s):
        return ob.Cost(1 / (self.si_.getStateValidityChecker().clearance(s) +
                            sys.float_info.min))

## Return an optimization objective which attempts to steer the robot
#  away from obstacles.
def getClearanceObjective(si):
    return ClearanceObjective(si)

## Create an optimization objective which attempts to optimize both
#  path length and clearance. We do this by defining our individual
#  objectives, then adding them to a MultiOptimizationObjective
#  object. This results in an optimization objective where path cost
#  is equivalent to adding up each of the individual objectives' path
#  costs.
#
#  When adding objectives, we can also optionally specify each
#  objective's weighting factor to signify how important it is in
#  optimal planning. If no weight is specified, the weight defaults to
#  1.0.
def getBalancedObjective1(si):
    lengthObj = ob.PathLengthOptimizationObjective(si)
    clearObj = ClearanceObjective(si)

    opt = ob.MultiOptimizationObjective(si)
    opt.addObjective(lengthObj, 5.0)
    opt.addObjective(clearObj, 1.0)

    return opt

## Create an optimization objective equivalent to the one returned by
#  getBalancedObjective1(), but use an alternate syntax.
#  THIS DOESN'T WORK YET. THE OPERATORS SOMEHOW AREN'T EXPORTED BY Py++.
# def getBalancedObjective2(si):
#     lengthObj = ob.PathLengthOptimizationObjective(si)
#     clearObj = ClearanceObjective(si)
#
#     return 5.0*lengthObj + clearObj


## Create an optimization objective for minimizing path length, and
#  specify a cost-to-go heuristic suitable for this optimal planning
#  problem.
def getPathLengthObjWithCostToGo(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostToGoHeuristic(ob.CostToGoHeuristic(ob.goalRegionCostToGo))
    return obj


# Keep these in alphabetical order and all lower case
def allocatePlanner(si, plannerType):
    if plannerType.lower() == "bfmtstar":
        return og.BFMT(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "prmstar":
        return og.PRMstar(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "sorrtstar":
        return og.SORRTstar(si)
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


# Keep these in alphabetical order and all lower case
def allocateObjective(si, objectiveType):
    if objectiveType.lower() == "pathclearance":
        return getClearanceObjective(si)
    elif objectiveType.lower() == "pathlength":
        return getPathLengthObjective(si)
    elif objectiveType.lower() == "thresholdpathlength":
        return getThresholdPathLengthObj(si)
    elif objectiveType.lower() == "weightedlengthandclearancecombo":
        return getBalancedObjective1(si)
    else:
        ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")
        
        
def plan(runTime, plannerType, objectiveType, fname):
     # Construct the robot state space in which we're planning. We're
     # planning in [0,1]x[0,1], a subset of R^2.
    space = ob.SE2StateSpace()
 
    # Set the bounds of space to be in [0,1]
    bounds=ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(1)
    space.setBounds(bounds)
 
     # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)
 
     # Set the object used to check which states in the space are valid
    validityChecker = ValidityChecker(si)
    si.setStateValidityChecker(validityChecker)

    si.setup()
 
    # Set our robot's starting state to be the bottom-left corner of
    # the environment, or (0,0).
    start = ob.State(space)
    start[0]=0
    start[1]=0
 
     # Set our robot's goal state to be the top-right corner of the
     # environment, or (1,1).
    goal = ob.State(space)
    goal[0]=1
    goal[1]=1
 
     # Create a problem instance
    pdef = ob.ProblemDefinition(si)
 
     # Set the start and goal states
    pdef.setStartAndGoalStates(start, goal)
 
     # Create the optimization objective specified by our command-line argument.
     # This helper function is simply a switch statement.
    pdef.setOptimizationObjective(allocateObjective(si, objectiveType))
 
     # Construct the optimal planner specified by us.
     # This helper function is simply a switch statement.
    optimizingPlanner = allocatePlanner(si, plannerType)
 
     # Set the problem we are trying to solve to the planner 
    optimizingPlanner.setProblemDefinition(pdef)
    #perform setup steps for the planner
    optimizingPlanner.setup()
    
    #print the settings for this space
    #print(si.settings())
    #print the problem settings
    #print(pdef)
     # attempt to solve the planning problem in the given runtime
    solved = optimizingPlanner.solve(runTime)
 
    if solved:
        
        #print solution path
        path=pdef.getSolutionPath()
        print("Found Solution:\n%s" % path)
        
        # Output the length of the path found
        print('{0} found solution of path length {1:.4f} with an optimization ' \
             'objective value of {2:.4f}'.format( \
             optimizingPlanner.getName(), \
             pdef.getSolutionPath().length(), \
             pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()))
 
         # If a filename was specified, output the path as a matrix to
         # that file for visualization
        if fname:
            with open(fname, 'w') as outFile:
                 outFile.write(pdef.getSolutionPath().printAsMatrix())
        print("saved final path as 'mat.txt' for matplotlib")
          
        # Extracting planner data from most recent solve attempt
        pd = ob.PlannerData(pdef.getSpaceInformation())
        optimizingPlanner.getPlannerData(pd)
        
        # Computing weights of all edges based on state space distance
        pd.computeEdgeWeights()
        
        #dataStorage=ob.PlannerDataStorage()
        #dataStorage.store(pd,"myPlannerData")
        
        graphml=pd.printGraphML()
        f = open("graph.graphml", 'w')
        f.write(graphml)
        f.close()
        print("saved")

    else:
         print("No solution found.")

# Create an argument parser

 
# Add a filename argument
runtime=5
#(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.

planner='RRTstar'
#(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.
#choices=['BFMTstar', 'BITstar', 'FMTstar', 'InformedRRTstar', 'PRMstar', 'RRTstar','SORRTstar']
     
objective='PathLength'
#(Optional) Specify the optimization objective, defaults to PathLength if not given.
#choices=['PathClearance', 'PathLength', 'ThresholdPathLength','WeightedLengthAndClearanceCombo']
     
file='/home/sean/scripts/mp_project/mat.txt'
#(Optional) Specify an output path for the found solution path.

#info=0
#(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG. Defaults to WARN.

 # Set the log level
#if info == 0:
#     ou.setLogLevel(ou.LOG_WARN)
#elif info == 1:
#     ou.setLogLevel(ou.LOG_INFO)
#elif info == 2:
#    ou.setLogLevel(ou.LOG_DEBUG)
#else:
#     ou.OMPL_ERROR("Invalid log-level integer.")



 # Solve the planning problem
plan(runtime, planner, objective, file) 
graph_file_path="graph.graphml"
mat_file_path="mat.txt"
plot_graph(graph_file_path,mat_file_path,0.09)

#circle1 = plt.Circle((0.5, 0.5), 0.25, color='g')
#ax = fig.gca()
#ax.add_artist(circle1)
#fig.savefig('plotcircles.png')