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
			
			Improve(currentNode, open, heuristicFunction_);
		}
		
		return null;// no path is found
	}
	
	// Procedure Improve as listed on page 70 of Heuristic Search: Theory and Applications by Stefan Edelkamp and Stefan Schrodl
	#if cs
	function Improve(currentNode_:Node, open_:PriorityQueue<Node>, heuristicFunction_:cs.system.Func_3<Node,Node,Float>):Void
	#else
	function Improve(currentNode_:Node, open_:PriorityQueue<Node>, heuristicFunction_: Node -> Node -> Float):Void
	#end
	{
		
		var neighbours:Array<Node> = map.GetRawNeighbours(currentNode_);
		for (i in 0...neighbours.length)
		{
			if (neighbours[i] != null)
			{
				var result = Jump(neighbours[i], currentNode_, 0);
				if (result.found)
				{
					for (point in result.jumpPoints)
					{
						// for now
						point.heuristic = heuristicFunction_(point, endNode);
						point.priority = point.heuristic;
						
						trace("found jump point at = " + point.x + " _ " + point.y);
						open_.enqueue(point);
					}
				}
			}
		}
		
	}
	
	function Jump(node_:Node, parentNode_:Node, length_:Int)
	{
		var indexResultNode = map.GetIndexOfNode(node_);
		var indexResultParentNode = map.GetIndexOfNode(parentNode_);
		
		var x:Int = indexResultNode.index.x;
		var y:Int = indexResultNode.index.y;
		var dx:Int = cast(Math.min(Math.max((x - indexResultParentNode.index.x), -1), 1), Int);
		var dy:Int = cast(Math.min(Math.max((y - indexResultParentNode.index.y), -1), 1), Int);
		
		trace("jumping on " + x + " _ " + y + " with direction " + dx + " _ " + dy);
		
		if (dx != 0 && dy != 0) // check diag first since we apply a horizontal and vertical search on the node inside JumpDiagonal anyways (dont need to do it twice)
		{
			var result = JumpDiagonal(x, y, dx, dy, length_);
			if (result.found)
			{
				trace("diag found");
				return result;
			}
		}
		else if (dx != 0)
		{
			var result = JumpHorizontal(x, y, dx, length_);
			if (result.found)
			{
				trace("horiz found");
				return result;
			}
		}else if (dy != 0)
		{
			var result = JumpVertical(x, y, dy, length_);
			if (result.found)
			{
				trace("vert found");
				return result;
			}
		}
		
		trace("nothing found");
		return {found: false, jumpPoints: null};
	}
	
	function JumpHorizontal(x_:Int, y_:Int, dx_:Int, length_:Int)
	{
		//var endTile_:Int = x_ + (length_ * dx_);
		/*
		 * for now im going to make it simple and just have it search as far as possible
		 * may be stupid on huge open terrain spaces as checking this horizontal line may not provide a good
		 * result
		 */
		var endTile:Int = (dx_ > 0) ? map.width - 1 : 0;
		var incrementAmount:Int = cast(Math.abs(endTile - x_), Int);
		var currentX:Int = x_;
		
		trace("endtile = " + endTile + "inc amount = " + incrementAmount);
		
		for (i in 0...incrementAmount)
		{
			var currentNode:Node = map.GetNodeByIndex(currentX, y_);
			
			//check to see if current node is traversable
			if (!currentNode.traversable || currentNode.parent != null)
			{
				// we hit a dead end
				trace("running deadend " + currentX + " _ " + y_ + " trav = " + currentNode.traversable + " _ " + (currentNode.parent != null));
				return {found: false, jumpPoints: null};
			}
			
			currentNode.parent = map.GetNodeByIndex(currentX - dx_, y_); // set parent so we know we traversed it
			
			// check to see if the current node has a forced neighbour, or is the end node
			if ((currentNode == endNode) ||
			((y_ + 1 < map.height) && (currentX + dx_ != endTile) && (!map.GetNodeByIndex(currentX, y_ + 1).traversable && map.GetNodeByIndex(currentX + dx_, y_ + 1).traversable) || // forced neighbour above
			((y_ - 1 >= 0) && (currentX + dx_ != endTile) && !map.GetNodeByIndex(currentX, y_ - 1).traversable && map.GetNodeByIndex(currentX + dx_, y_ - 1).traversable)))// forced neighbour below
			{
				return {found: true, jumpPoints: [currentNode]};
			}
			
			trace("horiz " + currentX + " _ " + y_);
			currentX += dx_;
		}
	
		return {found: false, jumpPoints: null}; // we hit the end of the map, either 0 or map.width
	}
	
	function JumpVertical(x_:Int, y_:Int, dy_:Int, length_:Int)
	{
		//var endTile_:Int = x_ + (length_ * dx_);
		/*
		 * for now im going to make it simple and just have it search as far as possible
		 * may be stupid on huge open terrain spaces as checking this vertical line may not provide a good
		 * result
		 */
		var endTile:Int = (dy_ > 0) ? map.height - 1 : 0;
		var incrementAmount:Int = cast(Math.abs(endTile - y_), Int);
		var currentY:Int = y_;
		
		for (i in 0...incrementAmount)
		{
			var currentNode:Node = map.GetNodeByIndex(x_, currentY);
			
			//check to see if current node is traversable
			if (!currentNode.traversable || currentNode.parent != null)
			{
				// we hit a dead end
				return {found: false, jumpPoints: null};
			}
			
			currentNode.parent = map.GetNodeByIndex(x_, currentY - dy_); // set parent so we know we traversed it
			
			// check to see if the current node has a forced neighbour
			if ((currentNode == endNode) ||
			((currentY + dy_ != endTile) && (x_ + 1 < map.width) && (!map.GetNodeByIndex(x_ + 1, currentY).traversable && map.GetNodeByIndex(x_ + 1, currentY + dy_).traversable)) || // forced neighbour right
			((currentY + dy_ != endTile) && (x_ - 1 >= 0) && (!map.GetNodeByIndex(x_ - 1, currentY).traversable && map.GetNodeByIndex(x_ - 1, currentY + dy_).traversable)))// forced neighbour left
			{
				return {found: true, jumpPoints: [currentNode]};
			}
			
			currentY += dy_;
		}
	
		return {found: false, jumpPoints: null}; // we hit the end of the map, either 0 or map.height
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
		var endTileX:Int = (dx_ > 0) ? map.width - 1 : 0;
		var endTileY:Int = (dy_ > 0) ? map.height - 1 : 0;
		var incrementAmount:Int = ((Math.abs(endTileX - x_) >= Math.abs(endTileY - y_))) ? cast(Math.abs(endTileX - x_), Int) : cast(Math.abs(endTileY - y_), Int);
		
		var currentX:Int = x_;
		var currentY:Int = y_;
		
		trace("jumping diagonal at " + currentX + " _ " + currentY + " with " + dx_ + " _ " + dy_ + " inc amount " + incrementAmount);
		for (i in 0...incrementAmount)
		{
			var currentNode:Node = map.GetNodeByIndex(currentX, currentY);
			//check to see if current node is traversable
			if (!currentNode.traversable || currentNode.parent != null)
			{
				// we hit a dead end
				return {found: false, jumpPoints: null};
			}
			
			currentNode.parent = map.GetNodeByIndex(currentX - dx_, currentY - dy_); // set parent so we know we traversed it
			
			//check horizontal + vertical directions
			var horizontal = JumpHorizontal(currentX + dx_, currentY, dx_, length_);
			var vertical = JumpVertical(currentX, currentY + dy_, dy_, length_);
			if (horizontal.found || vertical.found)
			{
				return {found: true, jumpPoints: [currentNode, horizontal.found ? horizontal.jumpPoints[0] : vertical.jumpPoints[0]]};
			}
			
			// check to see if the current node has a forced neighbour
			if ((currentNode == endNode) ||
			(!map.GetNodeByIndex(currentX - dx_, currentY).traversable && map.GetNodeByIndex(currentX - dx_, currentY + dy_).traversable) &&
			(!map.GetNodeByIndex(currentX, currentY - dy_).traversable && map.GetNodeByIndex(currentX + dx_, currentY - dy_).traversable))
			{
				return {found: true, jumpPoints: [currentNode]};
			}
			
			currentX += dx_;
			currentY += dy_;
		}
	
		return {found: false, jumpPoints: null}; // we hit the end of the map, either 0 or map.height
		
	}
	
}