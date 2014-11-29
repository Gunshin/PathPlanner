package pathPlanner;

import de.polygonal.ds.HashableItem;
import de.polygonal.ds.Prioritizable;
import de.polygonal.ds.Cloneable;

/**
 * ...
 * @author Michael Stephens
 */
class Node implements Prioritizable
extends HashableItem
{
	
	// prioritizable implements
	public var position:Int;
	public var priority:Float = 0;
	//
	
	public var x(get, null):Float;
	public var y(get, null):Float;
	
	@:isVar public var neighboursStructure(get, set):IGraphStructure;
	
	@:isVar public var traversable(get, set):Bool;
	
	public var heuristic:Float = 0;
	@:isVar public var pathCost(get, set):Float = 0;
	
	@:isVar public var parent(get, set):Node = null;
		
	public function new(x_:Float, y_:Float, traversable_:Bool, neighboursStructure_:IGraphStructure) 
	{
		// initialise HashableItem so that a unique key is generated for this node
		super();
		
		x = x_;
		y = y_;
		
		traversable = traversable_;
		
		neighboursStructure = neighboursStructure_;
		
	}
	
	public function GetNeighbours():Array<DistanceNode>
	{
		return neighboursStructure.GetNeighbours(this);
	}
	
	public function AddNeighbour(newNeighbour_:Node, ?distance_:Float):Void
	{
		neighboursStructure.AddNeighbour(this, newNeighbour_, distance_);
	}
	
	public function RemoveNeighbour(neighbour_:Node):Bool
	{
		return neighboursStructure.RemoveNeighbour(this, neighbour_);
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
	
	function get_neighboursStructure():IGraphStructure
	{
		return neighboursStructure;
	}
	
	function set_neighboursStructure(structure_:IGraphStructure):IGraphStructure
	{
		return neighboursStructure = structure_;
	}
	
}

