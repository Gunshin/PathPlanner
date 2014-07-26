package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 */
class Node 
{

	var neighbours(get, null):Array<DistanceNode> = new Array<DistanceNode>();
	
	var x(get, null):Float;
	var y(get, null):Float;
	
	@:isVar var traversable(get, set):Bool;
	
	var heuristic(get, null):Float = 0;
	@:isVar var pathCost(get, set):Float = 0;
	
	@:isVar var parent(get, set):Node = null;
		
	public function new(x_:Float, y_:Float, traversable_:Bool) 
	{
		x = x_;
		y = y_;
		
		traversable = traversable_;
	}
	
	public function get_x():Float
	{
		return x;
	}
	
	public function get_y():Float
	{
		return y;
	}
	
	public function get_traversable():Bool
	{
		return traversable;
	}
	
	public function set_traversable(traversable_:Bool):Bool
	{
		return traversable = traversable_;
	}
	
	public function get_neighbours():Array<DistanceNode>
	{
		return neighbours;
	}
	
	public function AddNeighbour(newNeighbour_:Node, ?distance_:Float):Void
	{
		
		var distanceNode:DistanceNode;
		
		if (distance_ != null)
		{
			distanceNode = new DistanceNode(newNeighbour_, distance_);
		}
		else
		{
			distanceNode = new DistanceNode(newNeighbour_, this);
		}
		
		neighbours.push(distanceNode);
	}
	
	public function RemoveNeighbour(neighbour_:Node):Bool
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
	
	public function get_heuristic()
	{
		return heuristic;
	}
	
	public function CalculateHeuristic(targetNode_:Node)
	{
		heuristic = Math.sqrt(Math.pow(get_x() - targetNode_.get_x(), 2) + Math.pow(get_y() - targetNode_.get_y(), 2)) + get_pathCost();
	}
	
	public function get_pathCost():Float
	{
		return pathCost;
	}
	
	public function set_pathCost(pathCost_:Float):Float
	{
		return pathCost = pathCost_;
	}
	
	public function get_parent():Node
	{
		return parent;
	}
	
	public function set_parent(parent_:Node):Node
	{
		return parent = parent_;
	}
}

