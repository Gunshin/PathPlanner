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
	var improveTimer:DebugRunningTimer = new DebugRunningTimer();
	
	public function new()
	{
	}
	
	#if cs
	public function FindPath(param_:PathplannerParameter, heuristicFunction_:cs.system.Func_3<Node,Node,Float>):Array<Node>
	#else
	public function FindPath(param_:PathplannerParameter, heuristicFunction_: Node -> Node -> Float):Array<Node>
	#end
	{
		improveTimer.Reset();
		
		#if action_output
		DebugLogger.GetInstance().ResetActionList();
		#end
		
		param_.startNode.SetParent(null);
		
		var open:PriorityQueue<Node> = new PriorityQueue<Node>(true, 128);
		var closed:PriorityQueue<Node> = new PriorityQueue<Node>(true, 128);
		
		open.enqueue(param_.startNode);
		#if action_output
		DebugLogger.GetInstance().AddToOpen(param_.startNode);
		#end
		
		while (!open.isEmpty())
		{
			
			var currentNode:Node = open.dequeue();
			closed.enqueue(currentNode);
			#if action_output
			DebugLogger.GetInstance().AddToClosed(currentNode);
			#end
			
			var neighbours:Array<DistanceNode>;
			
			if (currentNode == param_.goalNode)
			{
				trace("improve A*: " + (improveTimer.GetCurrentTotalTime() * 1000000));
				
				return PathUtility.ReconstructPath(param_.goalNode);
			}
			else if((neighbours = currentNode.GetNeighbours()).length > 0)
			{
				for (i in 0...neighbours.length)
				{
					if (neighbours[i] != null && neighbours[i].connectedNode.GetTraversable() == true) // if node is traversable, expand it
					{
						improveTimer.Start();
						Improve(currentNode, neighbours[i], param_.goalNode, open, closed, heuristicFunction_);
						improveTimer.Stop();
					}
					
				}
			}
			
		}
		
		return null;// no path is found
	}
	
	// Procedure Improve as listed on page 70 of Heuristic Search: Theory and Applications by Stefan Edelkamp and Stefan Schrodl
	#if cs
	function Improve(currentNode_:Node, successorNode_:DistanceNode, endNode_:Node, open_:PriorityQueue<Node>, closed_:PriorityQueue<Node>, heuristicFunction_:cs.system.Func_3<Node,Node,Float>):Void
	#else
	function Improve(currentNode_:Node, successorNode_:DistanceNode, endNode_:Node, open_:PriorityQueue<Node>, closed_:PriorityQueue<Node>, heuristicFunction_: Node -> Node -> Float):Void
	#end
	{
		#if action_output
		DebugLogger.GetInstance().Expand(successorNode_.connectedNode);
		#end
		if (open_.contains(successorNode_.connectedNode))
		{
			if (currentNode_.GetPathCost() + successorNode_.distanceBetween < successorNode_.connectedNode.GetPathCost())
			{
				successorNode_.connectedNode.SetParent(currentNode_);
				#if action_output
				DebugLogger.GetInstance().SetParent(successorNode_.connectedNode, currentNode_);
				#end
				successorNode_.connectedNode.SetPathCost(currentNode_.GetPathCost() + successorNode_.distanceBetween);
				#if cs
				successorNode_.connectedNode.heuristic = heuristicFunction_.Invoke(successorNode_.connectedNode, endNode_);
				#else
				successorNode_.connectedNode.heuristic = heuristicFunction_(successorNode_.connectedNode, endNode_); // the heuristic should not change, but under the assumption
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
				DebugLogger.GetInstance().SetParent(successorNode_.connectedNode, currentNode_);
				#end
				successorNode_.connectedNode.SetPathCost(currentNode_.GetPathCost() + successorNode_.distanceBetween);
				#if cs
				successorNode_.connectedNode.heuristic = heuristicFunction_.Invoke(successorNode_.connectedNode, endNode_);
				#else
				successorNode_.connectedNode.heuristic = heuristicFunction_(successorNode_.connectedNode, endNode_); // the heuristic should not change, but under the assumption
				#end
				// that a function may change it, we will forced it to update everytime.
				
				closed_.remove(successorNode_.connectedNode); // remove it from the closed list so it can be explored again to update values
				successorNode_.connectedNode.priority = successorNode_.connectedNode.GetPathCost() + successorNode_.connectedNode.heuristic;
				open_.enqueue(successorNode_.connectedNode); // add to open list so we can allow exploration
				#if action_output
				DebugLogger.GetInstance().AddToOpen(successorNode_.connectedNode);
				#end
			}
		}
		else
		{
			// if the neighbour is not in the open set, add it.
			successorNode_.connectedNode.SetParent(currentNode_);
			#if action_output
			DebugLogger.GetInstance().SetParent(successorNode_.connectedNode, currentNode_);
			#end
			successorNode_.connectedNode.SetPathCost(currentNode_.GetPathCost() + successorNode_.distanceBetween);
			#if cs
			successorNode_.connectedNode.heuristic = heuristicFunction_.Invoke(successorNode_.connectedNode, endNode_);
			#else
			successorNode_.connectedNode.heuristic = heuristicFunction_(successorNode_.connectedNode, endNode_); // set the heuristic for the node
			#end
			
			// since we are adding it to the queue for the first time, we need to set its priority
			successorNode_.connectedNode.priority = successorNode_.connectedNode.GetPathCost() + successorNode_.connectedNode.heuristic;
			open_.enqueue(successorNode_.connectedNode);
			#if action_output
			DebugLogger.GetInstance().AddToOpen(successorNode_.connectedNode);
			#end
		}
	}
}