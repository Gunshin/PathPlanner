package pathPlanner;

import de.polygonal.ds.Prioritizable;

/**
 * ...
 * @author Michael Stephens
 */
class Node implements Prioritizable
{
	
	// prioritizable implements
	public var position:Int;
	public var priority:Float = 0;
	
	public var neighbours(get, null):Array<DistanceNode> = new Array<DistanceNode>();
	
	public var x(get, null):Float;
	public var y(get, null):Float;
	
	@:isVar public  var traversable(get, set):Bool;
	
	public var heuristic:Float = 0;
	@:isVar public var pathCost(get, set):Float = 0;
	
	@:isVar public var parent(get, set):Node = null;
		
	public function new(x_:Float, y_:Float, traversable_:Bool) 
	{
		x = x_;
		y = y_;
		
		traversable = traversable_;
	}
	
	function get_x():Float
	{
		return x;
	}
	
	function get_y():Float
	{
		return y;
	}
	
	function get_traversable():Bool
	{
		return traversable;
	}
	
	function set_traversable(traversable_:Bool):Bool
	{
		return traversable = traversable_;
	}
	
	function get_neighbours():Array<DistanceNode>
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
	
	function get_pathCost():Float
	{
		return pathCost;
	}
	
	function set_pathCost(pathCost_:Float):Float
	{
		return pathCost = pathCost_;
	}
	
	function get_parent():Node
	{
		return parent;
	}
	
	function set_parent(parent_:Node):Node
	{
		return parent = parent_;
	}
	
	/*public function CalculateHeuristic(targetNode_:Node)
	{
		heuristic = Math.sqrt(Math.pow(get_x() - targetNode_.get_x(), 2) + Math.pow(get_y() - targetNode_.get_y(), 2)) + get_pathCost();
	}*/
	
}

