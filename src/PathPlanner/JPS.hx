package pathPlanner;

import de.polygonal.ds.PriorityQueue;

#if cs
import cs.Lib;
#end

/**
 * ...
 * @author Michael Stephens
 */
class JPS implements IPathfinder 
{
	public function new()
	{
		
	}
	
	#if cs
	public function FindPath(startNode_:Node, endNode_:Node, heuristicFunction_:cs.system.Func_3<Node,Node,Float>):Array<Node>
	#else
	public function FindPath(startNode_:Node, endNode_:Node, heuristicFunction_: Node -> Node -> Float):Array<Node>
	#end
	{
		trace(Type.getClass(startNode_.neighboursStructure));
		trace(Type.getClassName(GraphStructureIndirect));
		trace(Type.getClassName(Type.getClass(startNode_.neighboursStructure)) == Type.getClassName(GraphStructureIndirect));
		return null;
		
		/*startNode_.parent = null;
		
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
					if (neighbours[i].connectedNode.traversable == true) // if node is traversable, expand it
					{
						Improve(currentNode, neighbours[i], endNode_, open, closed, heuristicFunction_);
					}
					
				}
			}
			
		}
		
		return null;// no path is found*/
	}
	
	// Procedure Improve as listed on page 70 of Heuristic Search: Theory and Applications by Stefan Edelkamp and Stefan Schrodl
	#if cs
	function Improve(currentNode_:Node, successorNode_:DistanceNode, endNode_:Node, open_:PriorityQueue<Node>, closed_:PriorityQueue<Node>, heuristicFunction_:cs.system.Func_3<Node,Node,Float>):Void
	#else
	function Improve(currentNode_:Node, successorNode_:DistanceNode, endNode_:Node, open_:PriorityQueue<Node>, closed_:PriorityQueue<Node>, heuristicFunction_: Node -> Node -> Float):Void
	#end
	{
		if (open_.contains(successorNode_.connectedNode))
		{
			if (currentNode_.pathCost + successorNode_.distanceBetween < successorNode_.connectedNode.pathCost)
			{
				successorNode_.connectedNode.parent = currentNode_;
				successorNode_.connectedNode.pathCost = currentNode_.pathCost + successorNode_.distanceBetween;
				#if cs
				successorNode_.connectedNode.heuristic = heuristicFunction_.Invoke(successorNode_.connectedNode, endNode_);
				#else
				successorNode_.connectedNode.heuristic = heuristicFunction_(successorNode_.connectedNode, endNode_); // the heuristic should not change, but under the assumption
				#end
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
				#if cs
				successorNode_.connectedNode.heuristic = heuristicFunction_.Invoke(successorNode_.connectedNode, endNode_);
				#else
				successorNode_.connectedNode.heuristic = heuristicFunction_(successorNode_.connectedNode, endNode_); // the heuristic should not change, but under the assumption
				#end
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
			#if cs
			successorNode_.connectedNode.heuristic = heuristicFunction_.Invoke(successorNode_.connectedNode, endNode_);
			#else
			successorNode_.connectedNode.heuristic = heuristicFunction_(successorNode_.connectedNode, endNode_); // set the heuristic for the node
			#end
			
			// since we are adding it to the queue for the first time, we need to set its priority
			successorNode_.connectedNode.priority = successorNode_.connectedNode.pathCost + successorNode_.connectedNode.heuristic;
			open_.enqueue(successorNode_.connectedNode);
		}
	}
	
}