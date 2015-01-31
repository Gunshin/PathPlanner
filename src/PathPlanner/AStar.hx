package pathPlanner;

import de.polygonal.ds.PriorityQueue;

#if cs
import cs.Lib;
#end

/**
 * ...
 * @author Michael Stephens
 * 
 * A* algorithm as defined by page 70 of Heuristic Search: Theory and Applications by Stefan Edelkamp and Stefan Schrodl. Old version was based on the
 * wikipedia version, which worked just as well, but is not a great source.
 */
class AStar implements IPathfinder 
{
	//var improveTimer:DebugRunningTimer = new DebugRunningTimer();
	@:protected
	var heuristicFunction:
	#if cs
	cs.system.Func_3<Position,Position,Float>;
	#else
	Position -> Position -> Float;
	#end
	
	#if action_output
	var actionOutput:ActionOutput<Node>;
	#end
	
	public function new(heuristicFunction_:
		#if cs
		cs.system.Func_3<Position,Position,Float>
		#else
		Position -> Position -> Float
		#end
		)
	{
		heuristicFunction = heuristicFunction_;
	}
	
	public function FindPath(param_:PathplannerParameter):Array<Position>
	{
		//improveTimer.Reset();
		
		#if action_output
		actionOutput = new ActionOutput();
		#end
		
		param_.startNode.SetParent(null);
		
		var open:PriorityQueue<Node> = new PriorityQueue<Node>(true, 128);
		var closed:PriorityQueue<Node> = new PriorityQueue<Node>(true, 128);
		
		open.enqueue(param_.startNode);
		#if action_output
		actionOutput.AddAction("AddToOpen", param_.startNode, null);
		#end
		
		while (!open.isEmpty())
		{
			
			var currentNode:Node = open.dequeue();
			closed.enqueue(currentNode);
			#if action_output
			actionOutput.AddAction("AddToClosed", currentNode, null);
			#end
			
			var neighbours:Array<DistanceNode>;
			
			if (currentNode == param_.goalNode)
			{
				//trace("improve A*: " + (improveTimer.GetCurrentTotalTime() * 1000000));
				
				return PathUtility.ReconstructPathFromNodes(param_.goalNode);
			}
			else if((neighbours = currentNode.GetNeighbours()).length > 0)
			{
				for (i in 0...neighbours.length)
				{
					if (neighbours[i] != null && neighbours[i].connectedNode.GetTraversable() == true) // if node is traversable, expand it
					{
						//improveTimer.Start();
						Improve(currentNode, neighbours[i], param_.goalNode, open, closed);
						//improveTimer.Stop();
					}
					
				}
			}
			
		}
		
		return null;// no path is found
	}
	
	// Procedure Improve as listed on page 70 of Heuristic Search: Theory and Applications by Stefan Edelkamp and Stefan Schrodl
	function Improve(
	currentNode_:Node,
	successorNode_:DistanceNode,
	endNode_:Node,
	open_:PriorityQueue<Node>,
	closed_:PriorityQueue<Node>
	):Void
	{
		#if action_output
		actionOutput.AddAction("Expand", successorNode_.connectedNode, null);
		#end
		if (open_.contains(successorNode_.connectedNode))
		{
			if (currentNode_.GetPathCost() + successorNode_.distanceBetween < successorNode_.connectedNode.GetPathCost())
			{
				successorNode_.connectedNode.SetParent(currentNode_);
				#if action_output
				actionOutput.AddAction("SetParent", successorNode_.connectedNode, currentNode_);
				#end
				
				successorNode_.connectedNode.SetPathCost(currentNode_.GetPathCost() + successorNode_.distanceBetween);
				#if cs
				successorNode_.connectedNode.heuristic = heuristicFunction.Invoke(successorNode_.connectedNode.GetPosition(), endNode_.GetPosition());
				#else
				successorNode_.connectedNode.heuristic = heuristicFunction(successorNode_.connectedNode.GetPosition(), endNode_.GetPosition()); // the heuristic should not change, but under the assumption
				#end
				// that a function may change it, we will forced it to update everytime.
				
				// since it already belongs in the queue, we need to reset its priority
				open_.reprioritize(successorNode_.connectedNode, successorNode_.connectedNode.GetPathCost() + successorNode_.connectedNode.heuristic);
			}
		}
		else if (closed_.contains(successorNode_.connectedNode))
		{
			// if it is in the closed set, but we have found a better route to this neighbour, update it with the better route.
			if (currentNode_.GetPathCost() + successorNode_.distanceBetween < successorNode_.connectedNode.GetPathCost())
			{
				successorNode_.connectedNode.SetParent(currentNode_);
				#if action_output
				actionOutput.AddAction("SetParent", successorNode_.connectedNode, currentNode_);
				#end
				
				successorNode_.connectedNode.SetPathCost(currentNode_.GetPathCost() + successorNode_.distanceBetween);
				#if cs
				successorNode_.connectedNode.heuristic = heuristicFunction.Invoke(successorNode_.connectedNode.GetPosition(), endNode_.GetPosition());
				#else
				successorNode_.connectedNode.heuristic = heuristicFunction(successorNode_.connectedNode.GetPosition(), endNode_.GetPosition()); // the heuristic should not change, but under the assumption
				#end
				// that a function may change it, we will forced it to update everytime.
				
				closed_.remove(successorNode_.connectedNode); // remove it from the closed list so it can be explored again to update values
				successorNode_.connectedNode.priority = successorNode_.connectedNode.GetPathCost() + successorNode_.connectedNode.heuristic;
				open_.enqueue(successorNode_.connectedNode); // add to open list so we can allow exploration
				#if action_output
				actionOutput.AddAction("AddToOpen", successorNode_.connectedNode, null);
				#end
			}
		}
		else
		{
			// if the neighbour is not in the open set, add it.
			successorNode_.connectedNode.SetParent(currentNode_);
			#if action_output
			actionOutput.AddAction("SetParent", successorNode_.connectedNode, currentNode_);
			#end
			
			successorNode_.connectedNode.SetPathCost(currentNode_.GetPathCost() + successorNode_.distanceBetween);
			#if cs
			successorNode_.connectedNode.heuristic = heuristicFunction.Invoke(successorNode_.connectedNode.GetPosition(), endNode_.GetPosition());
			#else
			successorNode_.connectedNode.heuristic = heuristicFunction(successorNode_.connectedNode.GetPosition(), endNode_.GetPosition()); // set the heuristic for the node
			#end
			
			// since we are adding it to the queue for the first time, we need to set its priority
			successorNode_.connectedNode.priority = successorNode_.connectedNode.GetPathCost() + successorNode_.connectedNode.heuristic;
			open_.enqueue(successorNode_.connectedNode);
			#if action_output
			actionOutput.AddAction("AddToOpen", successorNode_.connectedNode, null);
			#end
		}
	}
	
	public function GetActionOutput():ActionOutput<Node>
	{
		#if action_output
		return actionOutput;
		#else
		throw "PathPlanner library has not been compiled with action output as needed. Recompile with compiler command -D action_output";
		#end
	}
}