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
	
	//TODO: fix this damn class
	//public var searched:Bool = false;
	
	@:protected
	var positionOfNode:Position;
	
	@:protected
	var neighboursStructure:IGraphStructure;
	
	@:protected
	var traversable:Bool;
	
	public var heuristic:Float = 0;
	
	@:protected
	var pathCost:Float = 0;
	
	@:protected
	var parent:Node = null;
		
	public function new(positionOfNode_:Position, traversable_:Bool, neighboursStructure_:IGraphStructure) 
	{
		// initialise HashableItem so that a unique key is generated for this node
		super();
		
		positionOfNode = positionOfNode_;
		
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
	
	public function GetPosition():Position
	{
		return positionOfNode;
	}
	
	public function SetPosition(position_:Position):Void
	{
		positionOfNode = position_;
	}
	
	public function GetTraversable():Bool
	{
		return traversable;
	}
	
	public function SetTraversable(traversable_:Bool):Bool
	{
		return traversable = traversable_;
	}
	
	public function GetPathCost():Float
	{
		return pathCost;
	}
	
	public function SetPathCost(pathCost_:Float):Float
	{
		return pathCost = pathCost_;
	}
	
	public function GetParent():Node
	{
		return parent;
	}
	
	public function SetParent(parent_:Node):Node
	{
		return parent = parent_;
	}
	
	public function GetNeighboursStructure():IGraphStructure
	{
		return neighboursStructure;
	}
	
	public function SetNeighboursStructure(structure_:IGraphStructure):IGraphStructure
	{
		return neighboursStructure = structure_;
	}
	
}

