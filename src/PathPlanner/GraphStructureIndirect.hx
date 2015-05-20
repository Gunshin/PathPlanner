package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 * 
 * This graph structure assumes an indirect graph eg. a graph that is not rectangular and can have nodes connecting anywhere with any pathcost.
 * 
 * Make sure you attach the node it belongs to afterwards as it is required for DistanceNode in AddNeighbour
 */
class GraphStructureIndirect implements IGraphStructure
{
	@:protected
	var neighbours:Array<DistanceNode> = new Array<DistanceNode>();

	public function new() 
	{
	}
	
	public function GetNeighbours(node_:Node):Array<DistanceNode>
	{
		return neighbours;
	}
	
	public function AddNeighbour(node_:Node, newNeighbour_:Node, ?distance_:Float):Void
	{
		if (PathUtility.ContainsNode(neighbours, newNeighbour_) != -1 || newNeighbour_ == null)
		{
			return;
		}
		
		var distanceNode:DistanceNode;
		
		if (distance_ != null)
		{
			distanceNode = new DistanceNode(newNeighbour_, distance_);
		}
		else
		{
			distanceNode = new DistanceNode(newNeighbour_, node_);
		}
		
		neighbours.push(distanceNode);
	}
	
	public function RemoveNeighbour(node_:Node, neighbour_:Node):Bool
	{
		for (i in 0...neighbours.length)
		{
			if (neighbours[i].connectedNode == neighbour_)
			{
				return neighbours.remove(neighbours[i]);
			}
		}
		
		return false;
	}
	
}