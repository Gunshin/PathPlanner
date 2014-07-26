package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 */
class AStar extends IPathfinder 
{

	public function new()
	{
		
	}
	
	override public function FindPath(startNode_:Node, endNode_:Node):Array<Node>
	{
		var open:Array<Node> = new Array<Node>();
		var closed:Array<Node> = new Array<Node>();
		
		open.push(startNode_);
		
		var targetReached:Bool = false;
		
		while (!targetReached)
		{
			var currentNode:Node = open.pop();
			
			if (currentNode == endNode_)
			{
				targetReached = true;
			}
			else
			{
				closed.push(currentNode);
				
				var G:Float = currentNode.get_pathCost() + 1;
				
				var neighbours:Array<DistanceNode> = currentNode.get_neighbours();
				
				for (i in 0...neighbours.length)
				{
					
					var tempG:Float = G + neighbours[i].distanceBetween;
					var closedIndex:Int = Contains(closed, neighbours[i].connectedNode);
					
					if (neighbours[i].connectedNode.get_traversable() == false)
					{
						//dont bother with this node if it is not traversable
						continue;
					}
					
					if (closedIndex > -1)
					{
						// if it is in the closed set, but we have found a better route to this neighbour, update it with the better route.
						if (tempG < neighbours[i].connectedNode.get_pathCost())
						{
							neighbours[i].connectedNode.set_parent(currentNode);
							neighbours[i].connectedNode.set_pathCost(tempG);
							neighbours[i].connectedNode.CalculateHeuristic(endNode_);
						}
					}
					else
					{
						// if the neighbour is not in the open set, add it.
						var openIndex:Int = Contains(open, neighbours[i].connectedNode);
						if (openIndex == -1)
						{
							neighbours[i].connectedNode.set_parent(currentNode);
							neighbours[i].connectedNode.set_pathCost(tempG);
							neighbours[i].connectedNode.CalculateHeuristic(endNode_);
							
							Insert(open, neighbours[i].connectedNode);
						}
					}
					
					
				}
			}
			
		}
		
		return ReconstructPath(endNode_);
	}
	
	
	
}