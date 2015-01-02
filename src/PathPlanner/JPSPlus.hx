package pathPlanner;

package pathPlanner;

import de.polygonal.ds.PriorityQueue;

#if cs
import cs.Lib;
#end

typedef JumpResult = { found:Bool, jumpPoint:Node }

/**
 * ...
 * @author Michael Stephens
 */
class JPSPlus implements IPathfinder 
{
	@:protected
	var map:GraphGridMapMinimalist;
	@:protected
	var mapRotated:GraphGridMapMinimalist;
	
	@:protected
	var searchedMap:GraphGridMapMinimalist;
	
	@:protected
	var endNode:Node;
		
	public function new(map_:GraphGridMapMinimalist)
	{
		map = map_;
		mapRotated = map.RotateMap();
	}
	
	#if cs
	public function FindPath(param_:PathplannerParameter, heuristicFunction_:cs.system.Func_3<Node,Node,Float>):Array<Node>
	#else
	public function FindPath(param_:PathplannerParameter, heuristicFunction_: Node -> Node -> Float):Array<Node>
	#end
	{
		/*verticalTimer.Reset();
		horizontalTimer.Reset();
		diagonalTimer.Reset();
		jumpTimer.Reset();
		improveTimer.Reset();*/
		//findTimer.Reset();
		//reconstructTimer.Reset();
		
		//findTimer.Start();
		
		var startNode:Node;
		
		startNode = param_.startNode != null ? param_.startNode : map.GetNodeByIndex(param_.startX, param_.startY);
		endNode = param_.goalNode != null ? param_.goalNode : map.GetNodeByIndex(param_.goalX, param_.goalY);
		
		if (endNode == null || startNode == null)
		{
			throw "start or goal has not been set by parameter input!";
			return null;
		}
		
		//map.ResetForPathplanning(); //TODO: correct Node implementation
		searchedMap.SetMap(false);
		
		#if debugging
		//DebugLogger.Assert(startNode.GetParent() != null, "warning, map not correctly reset, start node has parent");
		//DebugLogger.Assert(endNode.GetParent() != null, "warning, map not correctly reset, end node has parent");
		#end
		#if action_output
		DebugLogger.GetInstance().ResetActionList();
		#end
		
		startNode.SetParent(null);
		var index = map.GetIndexOfNode(startNode);
		
		searchedMap.SetTraversableTrue(index.x, index.y); // if we dont do this, the algorithm attempts to search the start node again which results in a cyclic reference
		
		var open:PriorityQueue<Node> = new PriorityQueue<Node>(true, 128);
		
		// the start node is being treat as a jump point, so that its neighbours are added to the list
		ExpandJumpPoint(startNode, open, heuristicFunction_);
		#if action_output
		DebugLogger.GetInstance().Expand(startNode);
		#end
		
		while (!open.isEmpty())
		{
			
			var currentNode:Node = open.dequeue();
			
			if (currentNode == endNode) //if we have the goal, return?
			{
				
				
				//trace("jps __________________________________");
				/*trace("vert: " + (verticalTimer.GetCurrentTotalTime() * 1000000));
				trace("hori: " + (horizontalTimer.GetCurrentTotalTime() * 1000000));
				trace("diag: " + (diagonalTimer.GetCurrentTotalTime() * 1000000));
				trace("jump: " + (jumpTimer.GetCurrentTotalTime() * 1000000));
				trace("improve: " + (improveTimer.GetCurrentTotalTime() * 1000000));
				
				
				reconstructTimer.Start();*/
				//findTimer.Stop();
				var path = PathUtility.ReconstructPath(endNode);
				/*reconstructTimer.Stop();
				
				trace("reconstruct: " + (reconstructTimer.GetCurrentTotalTime() * 1000000));*/
				
				//findTimer.Stop();
				//trace("find: " + (findTimer.GetCurrentTotalTime() * 1000000));
				
				return path;
			}
			
			//improveTimer.Start();
			Improve(currentNode, open, heuristicFunction_);
			//improveTimer.Stop();
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
		#end
		#if action_output
		DebugLogger.GetInstance().Expand(currentNode_);
		#end
		
		//jumpTimer.Start();
		var result = Jump(currentNode_, currentNode_.GetParent(), 0);
		//jumpTimer.Stop();
		
		if (result != null)
		{
			// special case if the result is the end node
			if (result == endNode)
			{
				result.priority = 0;
				open_.enqueue(result);
			}
			
			// found a jump point, lets expand it.
			ExpandJumpPoint(result, open_, heuristicFunction_);
		}
		
	}
	
	// we expand the jump point by adding all of its neighbours to the open list.
	#if cs
	function ExpandJumpPoint(jumpPoint_:Node, open_:PriorityQueue<Node>, heuristicFunction_:cs.system.Func_3<Node,Node,Float>):Void
	#else
	function ExpandJumpPoint(jumpPoint_:Node, open_:PriorityQueue<Node>, heuristicFunction_: Node -> Node -> Float):Void
	#end
	{
		var neighbours:Array<Node> = map.GetRawNeighbours(jumpPoint_);
		for (i in 0...neighbours.length)
		{
			if (neighbours[i] != null)
			{
				var traverseCost = (i % 2) == 0 ? 1.4 : 1;
				var index = map.GetIndexOfNode(neighbours[i]);
				if (neighbours[i].GetTraversable() == true && 
				(searchedMap.GetTraversable(index.x, index.y) == false || map.GetNodeByIndex(index.x, index.y).GetPathCost() > jumpPoint_.GetPathCost() + traverseCost))
				{
					// map.GetRawNeighbours(jumpPoint_); returns an array of neighbours with every even index being a corner neighbour
					neighbours[i].SetPathCost(jumpPoint_.GetPathCost() + traverseCost);
					
					// for now
					#if cs
					neighbours[i].heuristic = heuristicFunction_.Invoke(neighbours[i], endNode);
					#else
					neighbours[i].heuristic = heuristicFunction_(neighbours[i], endNode);
					#end
					neighbours[i].priority = neighbours[i].GetPathCost() + neighbours[i].heuristic;
					
					neighbours[i].SetParent(jumpPoint_);
					open_.enqueue(neighbours[i]);
					
					#if action_output
					DebugLogger.GetInstance().SetParent(neighbours[i], jumpPoint_);
					DebugLogger.GetInstance().AddToOpen(neighbours[i]);
					#end
				}
			}
		}
		
	}
	
	function Jump(node_:Node, parentNode_:Node, length_:Int):Node
	{
		#if debugging
		DebugLogger.Assert(node_ == null, "JPS:Jump: node_ is null");
		DebugLogger.Assert(parentNode_ == null, "JPS:Jump: parentNode_ is null");
		#end
		
		var indexResultNode = map.GetIndexOfNode(node_);
		var indexResultParentNode = map.GetIndexOfNode(parentNode_);
		
		var x:Int = indexResultNode.x;
		var y:Int = indexResultNode.y;
		var dx:Int = cast(Math.min(Math.max((x - indexResultParentNode.x), -1), 1), Int);
		var dy:Int = cast(Math.min(Math.max((y - indexResultParentNode.y), -1), 1), Int);
		
		if (dx != 0 && dy != 0) // check diag first since we apply a horizontal and vertical search on the node inside JumpDiagonal anyways (dont need to do it twice)
		{
			//diagonalTimer.Start();
			var result = JumpDiagonal(x, y, dx, dy, length_);
			//diagonalTimer.Stop();
			if (result != null)
			{
				return result;
			}
		}
		else if (dx != 0)
		{
			//horizontalTimer.Start();
			var result = JumpHorizontal(x, y, dx, length_, true);
			//horizontalTimer.Stop();
			if (result != null)
			{
				return result;
			}
		}else if (dy != 0)
		{
			//verticalTimer.Start();
			var result = JumpVertical(x, y, dy, length_, true);
			//verticalTimer.Stop();
			if (result != null)
			{
				return result;
			}
		}
		
		return null;
	}
	
	function JumpHorizontal(x_:Int, y_:Int, dx_:Int, length_:Int, expanding_:Bool):Node
	{
		
		//var endTile_:Int = x_ + (length_ * dx_);
		/*
		 * for now im going to make it simple and just have it search as far as possible
		 * may be stupid on huge open terrain spaces as checking this horizontal line may not provide a good
		 * result
		 */
		var currentX:Int = x_;
		
		while (currentX >= 0 && currentX < map.GetWidth())
		{
			var currentNode:Node = map.GetNodeByIndex(currentX, y_);
			
			#if debugging
			DebugLogger.GetInstance().Print("JumpHorizontal: on node: " + currentX + " _ " + y_  + " with direction x: " + dx_ + " and the node is !null: " + (currentNode != null));
			#end
			
			//check to see if current node is traversable
			if (!currentNode.GetTraversable() || 
			(searchedMap.GetTraversable(currentX, y_) == true && map.GetNodeByIndex(currentX - dx_, y_).GetPathCost() + 1 >= currentNode.GetPathCost()))
			{
				// we hit a dead end
				return null;
			}
			
			if (expanding_)
			{
				currentNode.SetParent(map.GetNodeByIndex(currentX - dx_, y_));
				#if action_output
				DebugLogger.GetInstance().SetParent(currentNode, map.GetNodeByIndex(currentX - dx_, y_));
				#end
				searchedMap.SetTraversableTrue(currentX, y_);
				currentNode.SetPathCost(currentNode.GetParent().GetPathCost() + 1);
			}
			
			// check to see if the current node has a forced neighbour, or is the end node
			if ((currentNode == endNode) ||
			((currentX + dx_ >= 0 && currentX + dx_ < map.GetWidth()) &&
			((y_ + 1 < map.GetHeight()) && (!map.GetNodeByIndex(currentX, y_ + 1).GetTraversable() && map.GetNodeByIndex(currentX + dx_, y_ + 1).GetTraversable()) || // forced neighbour above
			((y_ - 1 >= 0) && !map.GetNodeByIndex(currentX, y_ - 1).GetTraversable() && map.GetNodeByIndex(currentX + dx_, y_ - 1).GetTraversable()))))// forced neighbour below
			{
				return currentNode;
			}
			
			currentX += dx_;
		}
	
		return null; // we hit the end of the map, either 0 or map.width
	}
	
	function JumpVertical(x_:Int, y_:Int, dy_:Int, length_:Int, expanding_:Bool):Node
	{
		
		//var endTile_:Int = x_ + (length_ * dx_);
		/*
		 * for now im going to make it simple and just have it search as far as possible
		 * may be stupid on huge open terrain spaces as checking this vertical line may not provide a good
		 * result
		 */
		var currentY:Int = y_;
		
		while (currentY >= 0 && currentY < map.GetHeight())
		{
			var currentNode:Node = map.GetNodeByIndex(x_, currentY);
			
			#if debugging
			DebugLogger.GetInstance().Print("JumpVertical: on node: " + x_ + " _ " + currentY  + " with direction y: " + dy_ + " and the node is !null: " + (currentNode != null));
			#end
			
			//check to see if current node is traversable
			if (!currentNode.GetTraversable() || 
			(searchedMap.GetTraversable(x_, currentY) == true && map.GetNodeByIndex(x_, currentY - dy_).GetPathCost() + 1 >= currentNode.GetPathCost()))
			{
				// we hit a dead end
				return null;
			}
			
			if (expanding_)
			{
				currentNode.SetParent(map.GetNodeByIndex(x_, currentY - dy_));
				#if action_output
				DebugLogger.GetInstance().SetParent(currentNode, map.GetNodeByIndex(x_, currentY - dy_));
				#end
				searchedMap.SetTraversableTrue(x_, currentY);
				currentNode.SetPathCost(currentNode.GetParent().GetPathCost() + 1);
			}
			
			// check to see if the current node has a forced neighbour
			if ((currentNode == endNode) ||
			((currentY + dy_ >= 0 && currentY + dy_ < map.GetHeight()) &&
			((x_ + 1 < map.GetWidth()) && (!map.GetNodeByIndex(x_ + 1, currentY).GetTraversable() && map.GetNodeByIndex(x_ + 1, currentY + dy_).GetTraversable()) || // forced neighbour right
			((x_ - 1 >= 0) && (!map.GetNodeByIndex(x_ - 1, currentY).GetTraversable() && map.GetNodeByIndex(x_ - 1, currentY + dy_).GetTraversable())))))// forced neighbour left
			{
				return currentNode;
			}
			
			currentY += dy_;
		}
	
		return null; // we hit the end of the map, either 0 or map.height
	}
	
	function JumpDiagonal(x_:Int, y_:Int, dx_:Int, dy_:Int, length_:Int):Node
	{
		
		//var endTile_:Int = x_ + (length_ * dx_);
		/*
		 * for now im going to make it simple and just have it search as far as possible
		 * may be stupid on huge open terrain spaces as checking this vertical line may not provide a good
		 * result
		 * 
		 * Also, haxe does not have the standard great for loops inherent in most languages, so we have do do some magic?
		 */
		
		var currentX:Int = x_;
		var currentY:Int = y_;
		
		while (currentY >= 0 && currentY < map.GetHeight() && currentX >= 0 && currentX < map.GetWidth())
		{
			var currentNode:Node = map.GetNodeByIndex(currentX, currentY);
			#if debugging
			DebugLogger.GetInstance().Print("JumpDiagonal: on node: " + currentX + " _ " + currentY  + " with direction: " + dx_ + " _ " + dy_ + " and the node is !null: " + (currentNode != null));
			#end
			
			//check to see if current node is traversable
			if ((currentY < 0 || currentY >= map.GetHeight() || currentX < 0 || currentX >= map.GetWidth()) || 
			!currentNode.GetTraversable() ||
			(searchedMap.GetTraversable(currentX, currentY) == true && map.GetNodeByIndex(currentX - dx_, currentY - dy_).GetPathCost() + 1 >= currentNode.GetPathCost()))
			{
				// we hit a dead end
				return null;
			}
			
			currentNode.SetParent(map.GetNodeByIndex(currentX - dx_, currentY - dy_));
			#if action_output
			DebugLogger.GetInstance().SetParent(currentNode, map.GetNodeByIndex(currentX - dx_, currentY - dy_));
			#end
			searchedMap.SetTraversableTrue(currentX, currentY);
			currentNode.SetPathCost(currentNode.GetParent().GetPathCost() + 1.4);
			
			//check horizontal + vertical directions
			var horizontal = JumpHorizontal(currentX + dx_, currentY, dx_, length_, false);
			var vertical = JumpVertical(currentX, currentY + dy_, dy_, length_, false);
			if (horizontal != null || vertical != null)
			{
				return currentNode;// , horizontal.found ? horizontal.jumpPoints[0] : vertical.jumpPoints[0]] };
			}
			
			// check to see if the current node has a forced neighbour
			if ((currentNode == endNode) ||
			((currentY + dy_ >= 0 && currentY + dy_ < map.GetHeight() && currentX + dx_ >= 0 && currentX + dx_ < map.GetWidth()) && 
			((!map.GetNodeByIndex(currentX - dx_, currentY).GetTraversable() && map.GetNodeByIndex(currentX - dx_, currentY + dy_).GetTraversable()) &&
			(!map.GetNodeByIndex(currentX, currentY - dy_).GetTraversable() && map.GetNodeByIndex(currentX + dx_, currentY - dy_).GetTraversable()))))
			{
				return currentNode;
			}
			
			currentX += dx_;
			currentY += dy_;
		}
	
		return null; // we hit the end of the map, either 0 or map.height
		
	}
	
}