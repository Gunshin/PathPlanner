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
	
	#if debug
	var logger:DebugLogger;
	#end
		
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
		
		#if debugging
		DebugLogger.Assert(startNode_.parent != null, "warning, map not correctly reset, start node has parent");
		DebugLogger.Assert(endNode_.parent != null, "warning, map not correctly reset, end node has parent");
		#end
		
		endNode = endNode_;
		
		startNode_.parent = null;
		startNode_.searched = true; // if we dont do this, the algorithm attempts to search the start node again which results in a cyclic reference
		
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
			
			#if debugging
			DebugLogger.instance.closedSet.push(currentNode);
			#end
			
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
		
		#if debugging
		DebugLogger.Assert(currentNode_ == null, "JPS:Improve: currentNode is null");
		DebugLogger.Assert(open_ == null, "JPS:Improve: open_ is null");
		DebugLogger.Assert(heuristicFunction_ == null, "JPS:Improve: heuristicFunction_ is null");
		
		DebugLogger.instance.expandedSet.push(currentNode_);
		#end
		
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
						#if cs
						point.heuristic = heuristicFunction_.Invoke(point, endNode);
						#else
						point.heuristic = heuristicFunction_(point, endNode);
						#end
						point.priority = point.pathCost + point.heuristic;
						
						open_.enqueue(point);
						
						#if debugging
						DebugLogger.instance.openSet.push(point);
						#end
					}
				}
			}
		}
		
	}
	
	function Jump(node_:Node, parentNode_:Node, length_:Int)
	{
		#if debugging
		DebugLogger.Assert(node_ == null, "JPS:Jump: node_ is null");
		DebugLogger.Assert(parentNode_ == null, "JPS:Jump: parentNode_ is null");
		#end
		
		var indexResultNode = map.GetIndexOfNode(node_);
		var indexResultParentNode = map.GetIndexOfNode(parentNode_);
		
		var x:Int = indexResultNode.index.x;
		var y:Int = indexResultNode.index.y;
		var dx:Int = cast(Math.min(Math.max((x - indexResultParentNode.index.x), -1), 1), Int);
		var dy:Int = cast(Math.min(Math.max((y - indexResultParentNode.index.y), -1), 1), Int);
		
		if (dx != 0 && dy != 0) // check diag first since we apply a horizontal and vertical search on the node inside JumpDiagonal anyways (dont need to do it twice)
		{
			var result = JumpDiagonal(x, y, dx, dy, length_);
			if (result.found)
			{
				return result;
			}
		}
		else if (dx != 0)
		{
			var result = JumpHorizontal(x, y, dx, length_);
			if (result.found)
			{
				return result;
			}
		}else if (dy != 0)
		{
			var result = JumpVertical(x, y, dy, length_);
			if (result.found)
			{
				return result;
			}
		}
		
		return {found: false, jumpPoints: null};
	}
	
	function JumpHorizontal(x_:Int, y_:Int, dx_:Int, length_:Int)
	{
		if (x_ < 0 || x_ >= map.width)
		{
			return {found: false, jumpPoints: null};
		}
		
		//var endTile_:Int = x_ + (length_ * dx_);
		/*
		 * for now im going to make it simple and just have it search as far as possible
		 * may be stupid on huge open terrain spaces as checking this horizontal line may not provide a good
		 * result
		 */
		var endTile:Int = (dx_ > 0) ? map.width - 1 : 0;
		var incrementAmount:Int = cast(Math.abs(endTile - x_), Int);
		var currentX:Int = x_;
		
		#if debugging
		DebugLogger.Assert((currentX < 0) || (currentX >= map.width), "JPS:JumpHorizontal: currentX outside of bounds: " + currentX);
		DebugLogger.Assert(incrementAmount < 0, "JPS:JumpHorizontal: incrementAmount is out of bounds: " + incrementAmount);
		#end
		
		for (i in 0...incrementAmount)
		{
			var currentNode:Node = map.GetNodeByIndex(currentX, y_);
			
			#if debugging
			DebugLogger.Assert(currentNode == null, "JPS:JumpHorizontal: currentNode is null: " + currentX + " _ " + y_);
			DebugLogger.instance.Print("JumpHorizontal: on node: " + currentX + " _ " + y_  + " with direction x: " + dx_ + " and the node is !null: " + (currentNode != null));
			#end
			
			//check to see if current node is traversable
			if (!currentNode.traversable || currentNode.searched == true)
			{
				// we hit a dead end
				return {found: false, jumpPoints: null};
			}
			
			currentNode.parent = map.GetNodeByIndex(currentX - dx_, y_);
			currentNode.searched = true;
			currentNode.pathCost = currentNode.parent.pathCost + 1;
			
			// check to see if the current node has a forced neighbour, or is the end node
			if ((currentNode == endNode) ||
			((y_ + 1 < map.height) && (currentX + dx_ != endTile) && (!map.GetNodeByIndex(currentX, y_ + 1).traversable && map.GetNodeByIndex(currentX + dx_, y_ + 1).traversable) || // forced neighbour above
			((y_ - 1 >= 0) && (currentX + dx_ != endTile) && !map.GetNodeByIndex(currentX, y_ - 1).traversable && map.GetNodeByIndex(currentX + dx_, y_ - 1).traversable)))// forced neighbour below
			{
				return {found: true, jumpPoints: [currentNode]};
			}
			
			currentX += dx_;
		}
	
		return {found: false, jumpPoints: null}; // we hit the end of the map, either 0 or map.width
	}
	
	function JumpVertical(x_:Int, y_:Int, dy_:Int, length_:Int)
	{
		if (y_ < 0 || y_ >= map.height)
		{
			return {found: false, jumpPoints: null};
		}
		
		//var endTile_:Int = x_ + (length_ * dx_);
		/*
		 * for now im going to make it simple and just have it search as far as possible
		 * may be stupid on huge open terrain spaces as checking this vertical line may not provide a good
		 * result
		 */
		var endTile:Int = (dy_ > 0) ? map.height - 1 : 0;
		var incrementAmount:Int = cast(Math.abs(endTile - y_), Int);
		var currentY:Int = y_;
		
		#if debugging
		DebugLogger.Assert((currentY < 0) || (currentY >= map.height), "JPS:JumpVertical: currentY outside of bounds: " + currentY);
		DebugLogger.Assert(incrementAmount < 0, "JPS:JumpVertical: incrementAmount is out of bounds: " + incrementAmount);
		#end
		
		for (i in 0...incrementAmount)
		{
			var currentNode:Node = map.GetNodeByIndex(x_, currentY);
			
			#if debugging
			DebugLogger.Assert(currentNode == null, "JPS:JumpVertical: currentNode is null: " + x_ + " _ " + currentY);
			DebugLogger.instance.Print("JumpVertical: on node: " + x_ + " _ " + currentY  + " with direction y: " + dy_ + " and the node is !null: " + (currentNode != null));
			#end
			
			//check to see if current node is traversable
			if (!currentNode.traversable || currentNode.searched == true)
			{
				// we hit a dead end
				return {found: false, jumpPoints: null};
			}
			
			currentNode.parent = map.GetNodeByIndex(x_, currentY - dy_);
			currentNode.searched = true;
			currentNode.pathCost = currentNode.parent.pathCost + 1;
			
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
		if (y_ < 0 || y_ >= map.height || x_ < 0 || x_ >= map.width)
		{
			return {found: false, jumpPoints: null};
		}
		
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
		
		#if debugging
		DebugLogger.Assert((currentX < 0) || (currentX >= map.width), "JPS:JumpDiagonal: currentX outside of bounds: " + currentX);
		DebugLogger.Assert((currentY < 0) || (currentY >= map.height), "JPS:JumpDiagonal: currentY outside of bounds: " + currentY);
		DebugLogger.Assert(incrementAmount < 0, "JPS:JumpDiagonal: incrementAmount is out of bounds: " + incrementAmount);
		#end
		
		for (i in 0...incrementAmount)
		{
			var currentNode:Node = map.GetNodeByIndex(currentX, currentY);
			#if debugging
			DebugLogger.Assert(currentNode == null, "JPS:JumpDiagonal: currentNode is null: " + currentX + " _ " + currentY);
			DebugLogger.instance.Print("JumpDiagonal: on node: " + currentX + " _ " + currentY  + " with direction: " + dx_ + " _ " + dy_ + " and the node is !null: " + (currentNode != null));
			#end
			
			//check to see if current node is traversable
			if (!currentNode.traversable || currentNode.searched == true)
			{
				// we hit a dead end
				return {found: false, jumpPoints: null};
			}
			
			currentNode.parent = map.GetNodeByIndex(currentX - dx_, currentY - dy_);
			currentNode.searched = true;
			currentNode.pathCost = currentNode.parent.pathCost + 1.4;
			
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