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
	
	var map:Map;
	var endNode:Node;
		
	public function new()
	{
		
	}
	
	#if cs
	public function FindPath(startNode_:Node, endNode_:Node, heuristicFunction_:cs.system.Func_3<Node,Node,Float>):Array<Node>
	#else
	public function FindPath(startNode_:Node, endNode_:Node, heuristicFunction_: Node -> Node -> Float):Array<Node>
	#end
	{
		if (!Std.is(startNode_.neighboursStructure, Map))
		{
			throw "The nodes used to find a path do not have a \'Map\' graph structure";
			return null;
		}
		
		map = cast(startNode_.neighboursStructure, Map);
		map.ResetForPathplanning(); //TODO: correct Node implementation
		
		endNode = endNode_;
		
		startNode_.parent = null;
		
		var open:PriorityQueue<Node> = new PriorityQueue<Node>(true, 128);
		var closed:PriorityQueue<Node> = new PriorityQueue<Node>(true, 128);
		
		open.enqueue(startNode_);
		
		while (!open.isEmpty())
		{
			var currentNode:Node = open.dequeue();
			
			if (currentNode == endNode_) //if we have the goal, return?
			{
				return PathUtility.ReconstructPath(endNode_);
			}
			
			closed.enqueue(currentNode);
			
			Improve(currentNode, open, closed, heuristicFunction_);
		}
		
		return null;// no path is found
	}
	
	// Procedure Improve as listed on page 70 of Heuristic Search: Theory and Applications by Stefan Edelkamp and Stefan Schrodl
	#if cs
	function Improve(currentNode_:Node, open_:PriorityQueue<Node>, closed_:PriorityQueue<Node>, heuristicFunction_:cs.system.Func_3<Node,Node,Float>):Void
	#else
	function Improve(currentNode_:Node, open_:PriorityQueue<Node>, closed_:PriorityQueue<Node>, heuristicFunction_: Node -> Node -> Float):Void
	#end
	{
		
		var neighbours:Array<DistanceNode> = map.GetDirectNeighbours(currentNode_);
		for (i in 0...neighbours.length)
		{
			
		}
		
	}
	
	function Jump(node_:Node, parentNode_:Node, length_:Int, open_:PriorityQueue<Node>):Void
	{
		var x:Int = node_.x;
		var y:Int = node_.y;
		var dx:Int = (parentNode_.x - x) / Math.abs(parentNode_.x - x);
		var dy:Int = (parentNode_.y - y) / Math.abs(parentNode_.y - y);
		
		if (dx != 0) // check horizontal
		{
			var result = JumpHorizontal(x, y, dx, length_);
			
			if (result.found)
			{
				open_.enqueue(map.GetNodeByIndex(result.pos.x, result.pos.y));
			}
		}
		
		if (dy != 0) // check horizontal
		{
			var result = JumpVertical(x, y, dy, length_);
			
			if (result.found)
			{
				open_.enqueue(map.GetNodeByIndex(result.pos.x, result.pos.y));
			}
		}
		
		if (dx != 0 && dy != 0)
		{
			var result = JumpDiagonal(x, y, dx, dy, length_);
			
			if (result.found)
			{
				open_.enqueue(map.GetNodeByIndex(result.pos.x, result.pos.y));
			}
		}
	}
	
	function JumpHorizontal(x_:Int, y_:Int, dx_:Int, length_:Int)
	{
		//var endTile_:Int = x_ + (length_ * dx_);
		/*
		 * for now im going to make it simple and just have it search as far as possible
		 * may be stupid on huge open terrain spaces as checking this horizontal line may not provide a good
		 * result
		 */
		var endTile:Int = (dx_ > 0) ? map.width : 0;
		var incrementAmount:Int = cast(Math.abs(endTile - x_), Int);
		var currentX:Int = x_;
		
		for (i in 0...incrementAmount)
		{
			var currentNode:Node = map.GetNodeByIndex(currentX, y_);
			
			//check to see if current node is traversable
			if (!currentNode.traversable || currentNode.parent != null)
			{
				// we hit a dead end
				return {found: false, pos: {x: 0, y: 0}};
			}
			
			currentNode.parent = map.GetNodeByIndex(currentX - dx_, y_); // set parent so we know we traversed it
			
			// check to see if the current node has a forced neighbour, or is the end node
			if (((y_ + 1 < map.height) && (currentX + dx_ != endTile) && (!map.GetNodeByIndex(currentX, y_ + 1).traversable && map.GetNodeByIndex(currentX + dx_, y_ + 1).traversable) || // forced neighbour above
			((y_ - 1 >= 0) && (currentX + dx_ != endTile) && !map.GetNodeByIndex(currentX, y_ - 1).traversable && map.GetNodeByIndex(currentX + dx_, y_ - 1).traversable)))// forced neighbour below
			{
				return {found: true, pos: {x: currentX, y: y_}};
			}
			
			currentX += dx_;
		}
	
		return {found: false, pos: {x: 0, y: 0}}; // we hit the end of the map, either 0 or map.width
	}
	
	function JumpVertical(x_:Int, y_:Int, dy_:Int, length_:Int)
	{
		//var endTile_:Int = x_ + (length_ * dx_);
		/*
		 * for now im going to make it simple and just have it search as far as possible
		 * may be stupid on huge open terrain spaces as checking this vertical line may not provide a good
		 * result
		 */
		var endTile:Int = (dy_ > 0) ? map.height : 0;
		var incrementAmount:Int = cast(Math.abs(endTile - y_), Int);
		var currentY:Int = y_;
		
		for (i in 0...incrementAmount)
		{
			var currentNode:Node = map.GetNodeByIndex(x_, currentY);
			
			//check to see if current node is traversable
			if (!currentNode.traversable || currentNode.parent != null)
			{
				// we hit a dead end
				return {found: false, pos: {x: 0, y: 0}};
			}
			
			currentNode.parent = map.GetNodeByIndex(x_, currentY - dy_); // set parent so we know we traversed it
			
			// check to see if the current node has a forced neighbour
			if (((currentY + dy_ != endTile) && (x_ + 1 < map.width) && (!map.GetNodeByIndex(x_ + 1, currentY).traversable && map.GetNodeByIndex(x_ + 1, currentY + dy_).traversable)) || // forced neighbour right
			((currentY + dy_ != endTile) && (x_ - 1 >= 0) && (!map.GetNodeByIndex(x_ - 1, currentY).traversable && map.GetNodeByIndex(x_ - 1, currentY + dy_).traversable)))// forced neighbour left
			{
				return {found: true, pos: {x: x_, y: currentY}};
			}
			
			currentY += dy_;
		}
	
		return {found: false, pos: {x: 0, y: 0}}; // we hit the end of the map, either 0 or map.height
	}
	
	function JumpDiagonal(x_:Int, y_:Int, dx_:Int, dy_:Int, length_:Int)
	{
		
		//var endTile_:Int = x_ + (length_ * dx_);
		/*
		 * for now im going to make it simple and just have it search as far as possible
		 * may be stupid on huge open terrain spaces as checking this vertical line may not provide a good
		 * result
		 * 
		 * Also, haxe does not have the standard great for loops inherent in most languages, so we have do do some magic?
		 */
		var endTileX:Int = (dx_ > 0) ? map.width : 0;
		var endTileY:Int = (dy_ > 0) ? map.height : 0;
		var incrementAmount:Int = ((Math.abs(endTileX - x_) >= Math.abs(endTileY - y_))) ? cast(Math.abs(endTileX - x_), Int) : cast(Math.abs(endTileY - y_), Int);
		
		var currentX:Int = x_;
		var currentY:Int = y_;
		for (i in 0...incrementAmount)
		{
			var currentNode:Node = map.GetNodeByIndex(currentX, currentY);
			//check to see if current node is traversable
			if (!currentNode.traversable || currentNode.parent != null)
			{
				// we hit a dead end
				return {found: false, pos: {x: 0, y: 0}};
			}
			
			currentNode.parent = map.GetNodeByIndex(currentX - dx_, currentY - dy_); // set parent so we know we traversed it
			
			//check horizontal + vertical directions
			var horizontal = JumpHorizontal(x_, y_, dx_, length_);
			var vertical = JumpVertical(x_, y_, dy_, length_);
			if (horizontal.found || vertical.found)
			{
				return {found: true, pos: {x: currentX, y: currentY}};
			}
			
			// check to see if the current node has a forced neighbour
			if ((!map.GetNodeByIndex(currentX - dx_, currentY).traversable && map.GetNodeByIndex(currentX - dx_, currentY + dy_).traversable) &&
			(!map.GetNodeByIndex(currentX, currentY - dy_).traversable && map.GetNodeByIndex(currentX + dx_, currentY - dy_).traversable))
			{
				return {found: true, pos: {x: currentX, y: currentY}};
			}
			
			currentX += dx_;
			currentY += dy_;
		}
	
		return {found: false, pos: {x: 0, y: 0}}; // we hit the end of the map, either 0 or map.height
		
	}
	
}