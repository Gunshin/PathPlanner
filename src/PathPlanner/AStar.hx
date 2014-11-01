package pathPlanner;

import de.polygonal.ds.PriorityQueue;

/**
 * ...
 * @author Michael Stephens
 * 
 * A* algorithm as defined by page 70 of Heuristic Search: Theory and Applications by Stefan Edelkamp and Stefan Schrodl. Old version was based on the
 * wikipedia version, which worked just as well, but is not a great source.
 */
class AStar implements IPathfinder 
{
	
	var heuristicFunction: Node -> Node -> Float;
	var endNode:Node;

	public function new()
	{
		
	}
	
	public function FindPath(startNode_:Node, endNode_:Node, heuristicFunction_: Node -> Node -> Float):Array<Node>
	{
		heuristicFunction = heuristicFunction_;
		endNode = endNode_;
		
		startNode_.parent = null;
		
		var open:PriorityQueue<Node> = new PriorityQueue<Node>(true, 128);
		var closed:PriorityQueue<Node> = new PriorityQueue<Node>(true, 128);
		
		open.enqueue(startNode_);
		
		while (!open.isEmpty())
		{
			var currentNode:Node = open.dequeue();
			
			closed.enqueue(currentNode);
			
			var neighbours:Array<DistanceNode>;
			
			if (currentNode == endNode_)
			{
				return PathUtility.ReconstructPath(endNode_);
			}
			else if((neighbours = currentNode.GetNeighbours()).length > 0)
			{
				for (i in 0...neighbours.length)
				{
					Improve(currentNode, neighbours[i], open, closed);
				}
			}
			
		}
		
		return null;// no path is found
	}
	
	// Procedure Improve as listed on page 70 of Heuristic Search: Theory and Applications by Stefan Edelkamp and Stefan Schrodl
	function Improve(currentNode_:Node, successorNode_:DistanceNode, open_:PriorityQueue<Node>, closed_:PriorityQueue<Node>):Void
	{
		
		if (successorNode_.connectedNode.traversable == false)
		{
			return; // ignore any non-traversable nodes
		}
		
		if (open_.contains(successorNode_.connectedNode))
		{
			if (currentNode_.pathCost + successorNode_.distanceBetween < successorNode_.connectedNode.pathCost)
			{
				successorNode_.connectedNode.parent = currentNode_;
				successorNode_.connectedNode.pathCost = currentNode_.pathCost + successorNode_.distanceBetween;
				successorNode_.connectedNode.heuristic = heuristicFunction(successorNode_.connectedNode, endNode); // the heuristic should not change, but under the assumption
				// that a function may change it, we will forced it to update everytime.
				
				// since we are adding it to the queue for the first time, we need to set its priority
				open_.reprioritize(successorNode_.connectedNode, successorNode_.connectedNode.pathCost + successorNode_.connectedNode.heuristic);
			}
		}
		else if (closed_.contains(successorNode_.connectedNode))
		{
			// if it is in the closed set, but we have found a better route to this neighbour, update it with the better route.
			if (currentNode_.pathCost + successorNode_.distanceBetween < successorNode_.connectedNode.pathCost)
			{
				successorNode_.connectedNode.parent = currentNode_;
				successorNode_.connectedNode.pathCost = currentNode_.pathCost + successorNode_.distanceBetween;
				successorNode_.connectedNode.heuristic = heuristicFunction(successorNode_.connectedNode, endNode); // the heuristic should not change, but under the assumption
				// that a function may change it, we will forced it to update everytime.
				
				closed_.remove(successorNode_.connectedNode); // remove it from the closed list so it can be explored again to update values
				successorNode_.connectedNode.priority = successorNode_.connectedNode.pathCost + successorNode_.connectedNode.heuristic;
				open_.enqueue(successorNode_.connectedNode); // add to open list so we can allow exploration
			}
		}
		else
		{
			// if the neighbour is not in the open set, add it.
			successorNode_.connectedNode.parent = currentNode_;
			successorNode_.connectedNode.pathCost = currentNode_.pathCost + successorNode_.distanceBetween;
			successorNode_.connectedNode.heuristic = heuristicFunction(successorNode_.connectedNode, endNode); //set the heuristic for the node
			
			// since we are adding it to the queue for the first time, we need to set its priority
			successorNode_.connectedNode.priority = successorNode_.connectedNode.pathCost + successorNode_.connectedNode.heuristic;
			open_.enqueue(successorNode_.connectedNode);
		}
	}
}