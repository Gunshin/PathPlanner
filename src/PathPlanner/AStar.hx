package pathPlanner;

import de.polygonal.ds.PriorityQueue;

/**
 * ...
 * @author Michael Stephens
 */
class AStar implements IPathfinder 
{

	public function new()
	{
		
	}
	
	public function FindPath(startNode_:Node, endNode_:Node, heuristicFunction: Node -> Node -> Float):Array<Node>
	{
		startNode_.parent = null;
		
		var open:PriorityQueue<Node> = new PriorityQueue<Node>(true, 128);
		var closed:PriorityQueue<Node> = new PriorityQueue<Node>(true, 128);
		
		open.enqueue(startNode_);
		
		var targetReached:Bool = false;
		
		while (!targetReached && !open.isEmpty())
		{
			var currentNode:Node = open.dequeue();
			trace(currentNode.x + " _ " + currentNode.y);
			
			if (currentNode == endNode_)
			{
				trace("found path");
				targetReached = true;
			}
			else
			{
				closed.enqueue(currentNode);
				
				var G:Float = currentNode.pathCost;
				
				var neighbours:Array<DistanceNode> = currentNode.neighbours;
				
				trace("neighbour count = " + neighbours.length);
				
				for (i in 0...neighbours.length)
				{
					
					var tempG:Float = G + neighbours[i].distanceBetween;
					//var closedIndex:Int = PathUtility.Contains(closed, neighbours[i].connectedNode);
					
					if (neighbours[i].connectedNode.traversable == false)
					{
						//dont bother with this node if it is not traversable
					}
					else
					{
						if (closed.contains(neighbours[i].connectedNode))
						{
							// if it is in the closed set, but we have found a better route to this neighbour, update it with the better route.
							if (tempG < neighbours[i].connectedNode.pathCost)
							{
								neighbours[i].connectedNode.parent = currentNode;
								neighbours[i].connectedNode.pathCost = tempG;
								neighbours[i].connectedNode.heuristic = heuristicFunction(neighbours[i].connectedNode, endNode_);
								
								// since it already exists in the queue, we must call reprioritize. Check http://polygonal.github.io/ds/doc/ for more info
								closed.reprioritize(neighbours[i].connectedNode, neighbours[i].connectedNode.pathCost + neighbours[i].connectedNode.heuristic);
							}
						}
						else
						{
							// if the neighbour is not in the open set, add it.
							if (!open.contains(neighbours[i].connectedNode))
							{
								neighbours[i].connectedNode.parent = currentNode;
								neighbours[i].connectedNode.pathCost = tempG;
								neighbours[i].connectedNode.heuristic = heuristicFunction(neighbours[i].connectedNode, endNode_);
								
								// since we are adding it to the queue for the first time, we need to set its priority
								neighbours[i].connectedNode.priority = neighbours[i].connectedNode.pathCost + neighbours[i].connectedNode.heuristic;
								open.enqueue(neighbours[i].connectedNode);
								
								trace("added neighbour " + neighbours[i].connectedNode.x + " _ " + neighbours[i].connectedNode.y + " to open");
							}
						}
					}
				}
			}
			
		}
		
		return PathUtility.ReconstructPath(endNode_);
	}
}