package pathPlanner;

import de.polygonal.ds.PriorityQueue;
import haxe.ds.Vector;

#if cs
import cs.Lib;
#end

/**
 * ...
 * @author Michael Stephens
 */
class JPSM implements IPathfinder 
{
	@:protected
	var map:GraphGridMap;
	@:protected
	var endNode:Node;
	
	@:protected
	var searchedMap:GraphGridMapMinimalist;
	
	var heuristicFunction:
	#if cs
	cs.system.Func_3<Position,Position,Float>;
	#else
	Position -> Position -> Float;
	#end
	
	#if action_output
	var actionOutput:ActionOutput;
	#end
	
	/*var verticalTimer:DebugRunningTimer = new DebugRunningTimer();
	var horizontalTimer:DebugRunningTimer = new DebugRunningTimer();
	var diagonalTimer:DebugRunningTimer = new DebugRunningTimer();
	var jumpTimer:DebugRunningTimer = new DebugRunningTimer();
	var improveTimer:DebugRunningTimer = new DebugRunningTimer();*/
	//var findTimer:DebugRunningTimer = new DebugRunningTimer();
	//var reconstructTimer:DebugRunningTimer = new DebugRunningTimer();
		
	public function new(map_:GraphGridMap, heuristicFunction_:
		#if cs
		cs.system.Func_3<Position,Position,Float>
		#else
		Position -> Position -> Float
		#end
		)
	{
		map = map_;
		searchedMap = new GraphGridMapMinimalist(map.GetWidth(), map.GetHeight(), false);
		
		heuristicFunction = heuristicFunction_;
	}
	
	public function FindPath(param_:PathplannerParameter):Array<Node>
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
		//trace("searchmap: " + searchedMap.GetWidth() + " _ " + searchedMap.GetHeight());
		searchedMap.SetMap(false);
		//trace("__ " + searchedMap.GetTraversable(0, 0));

		#if action_output
		actionOutput = new ActionOutput();
		#end
		
		startNode.SetParent(null);
		//trace("
		
		// if we dont do this, the algorithm attempts to search the start node again which results in a cyclic reference
		searchedMap.SetTraversableTrue(startNode.GetPosition().GetX(), startNode.GetPosition().GetY());
		
		var open:PriorityQueue<Node> = new PriorityQueue<Node>(true, 128);
		
		// the start node is being treat as a jump point, so that its neighbours are added to the list
		//ExpandJumpPoint(startNode, open);
		
		#if action_output
		actionOutput.Expand(startNode);
		#end
		for ( i in 0...map.GetRawNeighbours(startNode).length)
		{
			var traverseCost = (i % 2) == 0 ? 1.4 : 1;
			map.GetRawNeighbours(startNode)[i].SetPathCost(traverseCost);
			
			map.GetRawNeighbours(startNode)[i].heuristic = 
			#if cs
			heuristicFunction.Invoke(map.GetRawNeighbours(startNode)[i].GetPosition(), endNode.GetPosition());
			#else
			heuristicFunction(map.GetRawNeighbours(startNode)[i].GetPosition(), endNode.GetPosition());
			#end
			map.GetRawNeighbours(startNode)[i].priority = map.GetRawNeighbours(startNode)[i].GetPathCost() + map.GetRawNeighbours(startNode)[i].heuristic;
			
			#if action_output
			actionOutput.SetParent(map.GetRawNeighbours(startNode)[i], startNode);
			actionOutput.AddToOpen(map.GetRawNeighbours(startNode)[i]);
			#end
			
			open.enqueue(map.GetRawNeighbours(startNode)[i]);
		}
		
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
			Improve(currentNode, open);
			//improveTimer.Stop();
		}
		
		return null;// no path is found
	}
	
	function Improve(currentNode_:Node, open_:PriorityQueue<Node>):Void
	{
		#if debugging
		DebugLogger.Assert(currentNode_ != null, "JPS:Improve: currentNode is null");
		DebugLogger.Assert(open_ != null, "JPS:Improve: open_ is null");
		DebugLogger.Assert(heuristicFunction != null, "JPS:Improve: heuristicFunction_ is null");
		#end
		#if action_output
		actionOutput.Expand(currentNode_);
		#end
		
		//jumpTimer.Start();
		var result = Jump(currentNode_, currentNode_.GetParent(), 0);
		//jumpTimer.Stop();
		
		if (result != null)
		{
			for (i in 0...result.length)
			{
			
				if (result[i] != null)
				{
					// special case if the result is the end node
					if (result[i] == endNode)
					{
						result[i].priority = 0;
						open_.enqueue(result[i]);
						return;
					}
					
					// found a jump point, lets expand it.
					AddToQueue(result[i], open_);
				}
			}
		}
		
	}
	
	function AddToQueue(node_:Node, open_:PriorityQueue<Node>)
	{
		#if debugging
		DebugLogger.Assert(node_ != null, "node_: " + node_.GetPosition().ToString() + " is null");
		DebugLogger.Assert(node_.GetParent() != null, "node_: " + node_.GetPosition().ToString() + " GetParent is null");
		#end
		
		node_.SetPathCost(node_.GetParent().GetPathCost() + DistanceNode.Distance(node_, node_.GetParent()));
			
		node_.heuristic = 
		#if cs
		heuristicFunction.Invoke(node_.GetPosition(), endNode.GetPosition());
		#else
		heuristicFunction(node_.GetPosition(), endNode.GetPosition());
		#end
		node_.priority = node_.GetPathCost() + node_.heuristic;
		
		open_.enqueue(node_);
	}
	
	// we expand the jump point by adding all of its neighbours to the open list.
	/*function ExpandJumpPoint(jumpPoint_:Node, open_:PriorityQueue<Node>):Void
	{
		var neighbours:Array<Node> = map.GetRawNeighbours(jumpPoint_);
		for (i in 0...neighbours.length)
		{
			if (neighbours[i] != null)
			{
				var traverseCost = (i % 2) == 0 ? 1.4 : 1;
				if (neighbours[i].GetTraversable() == true && 
				(!searchedMap.GetTraversable(neighbours[i].GetPosition().GetX(), neighbours[i].GetPosition().GetY()) ||
				map.GetNodeByIndex(neighbours[i].GetPosition().GetX(), neighbours[i].GetPosition().GetY()).GetPathCost() > jumpPoint_.GetPathCost() + traverseCost))
				{
					// map.GetRawNeighbours(jumpPoint_); returns an array of neighbours with every even index being a corner neighbour
					neighbours[i].SetPathCost(jumpPoint_.GetPathCost() + traverseCost);
					
					// for now
					#if cs
					neighbours[i].heuristic = heuristicFunction_.Invoke(neighbours[i], endNode);
					#else
					neighbours[i].heuristic = heuristicFunction(neighbours[i].GetPosition(), endNode.GetPosition());
					#end
					neighbours[i].priority = neighbours[i].GetPathCost() + neighbours[i].heuristic;
					
					neighbours[i].SetParent(jumpPoint_);
					open_.enqueue(neighbours[i]);
					
					#if action_output
					actionOutput.SetParent(neighbours[i], jumpPoint_);
					actionOutput.AddToOpen(neighbours[i]);
					#end
				}
			}
		}
	}*/
	
	function Jump(node_:Node, parentNode_:Node, length_:Int):Array<Node>
	{
		#if debugging
		DebugLogger.Assert(node_ != null, "JPS:Jump: node_ is null");
		DebugLogger.Assert(parentNode_ != null, "JPS:Jump: parentNode_ is null");
		#end

		var x:Int = node_.GetPosition().GetX();
		var y:Int = node_.GetPosition().GetY();
		var dx:Int = cast(Math.min(Math.max((x - parentNode_.GetPosition().GetX()), -1), 1), Int);
		var dy:Int = cast(Math.min(Math.max((y - parentNode_.GetPosition().GetY()), -1), 1), Int);
		
		if (dx != 0 && dy != 0) // check diag first since we apply a horizontal and vertical search on the node inside JumpDiagonal anyways (dont need to do it twice)
		{
			//diagonalTimer.Start();
			var result = JumpDiagonal(x, y, dx, dy, length_);
			//diagonalTimer.Stop();
			return result;
		}
		else if (dx != 0)
		{
			//horizontalTimer.Start();
			var result = JumpHorizontal(x, y, dx, length_, true);
			//horizontalTimer.Stop();
			return result;
		}else if (dy != 0)
		{
			//verticalTimer.Start();
			var result = JumpVertical(x, y, dy, length_, true);
			//verticalTimer.Stop();
			return result;
		}
		
		return null;
	}
	
	function JumpHorizontal(x_:Int, y_:Int, dx_:Int, length_:Int, expanding_:Bool):Array<Node>
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
				actionOutput.SetParent(currentNode, map.GetNodeByIndex(currentX - dx_, y_));
				#end
				searchedMap.SetTraversableTrue(currentX, y_);
				currentNode.SetPathCost(currentNode.GetParent().GetPathCost() + 1);
			}
			
			
			
			// check to see if the current node has a forced neighbour, or is the end node
			if ((currentNode == endNode) ||
			((currentX + dx_ >= 0 && currentX + dx_ < map.GetWidth()) &&
			(((y_ + 1 < map.GetHeight()) && (!map.GetNodeByIndex(currentX, y_ + 1).GetTraversable() && map.GetNodeByIndex(currentX + dx_, y_ + 1).GetTraversable())) || // forced neighbour above
			((y_ - 1 >= 0) && (!map.GetNodeByIndex(currentX, y_ - 1).GetTraversable() && map.GetNodeByIndex(currentX + dx_, y_ - 1).GetTraversable())))))// forced neighbour below
			{
				// could be null since might be out of bounds
				var neighbourAbove:Bool = (!map.GetNodeByIndex(currentX, y_ + 1).GetTraversable() && map.GetNodeByIndex(currentX + dx_, y_ + 1).GetTraversable());
				var neighbourBelow:Bool = (!map.GetNodeByIndex(currentX, y_ - 1).GetTraversable() && map.GetNodeByIndex(currentX + dx_, y_ - 1).GetTraversable());
				
				//since we have found a jump point at the current node, we are taking the continuation node (the one we would have visited next) and any forced neighbours
				var returnee = [map.GetNodeByIndex(currentX + dx_, y_).GetTraversable() ? map.GetNodeByIndex(currentX + dx_, y_) : null,
				neighbourAbove ? map.GetNodeByIndex(currentX + dx_, y_ + 1) : null,
				neighbourBelow ? map.GetNodeByIndex(currentX + dx_, y_ - 1) : null];
				
				if (expanding_)
				{
					// since during diagonal axis checking we do no 'expand' the nodes, the return is also ignored, so avoid setting parents since nullnull
					for (i in returnee)
					{
						if (i != null)
						{
							i.SetParent(currentNode);
							#if action_output
							actionOutput.SetParent(i, currentNode);
							#end
						}
					}
				}
				
				return returnee;
			}
			
			currentX += dx_;
		}
	
		return null; // we hit the end of the map, either 0 or map.width
	}
	
	function JumpVertical(x_:Int, y_:Int, dy_:Int, length_:Int, expanding_:Bool):Array<Node>
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
				actionOutput.SetParent(currentNode, map.GetNodeByIndex(x_, currentY - dy_));
				#end
				searchedMap.SetTraversableTrue(x_, currentY);
				currentNode.SetPathCost(currentNode.GetParent().GetPathCost() + 1);
			}
			
			// check to see if the current node has a forced neighbour
			if ((currentNode == endNode) ||
			((currentY + dy_ >= 0 && currentY + dy_ < map.GetHeight()) &&
			(((x_ + 1 < map.GetWidth()) && (!map.GetNodeByIndex(x_ + 1, currentY).GetTraversable() && map.GetNodeByIndex(x_ + 1, currentY + dy_).GetTraversable())) || // forced neighbour right
			((x_ - 1 >= 0) && (!map.GetNodeByIndex(x_ - 1, currentY).GetTraversable() && map.GetNodeByIndex(x_ - 1, currentY + dy_).GetTraversable())))))// forced neighbour left
			{
				// could be null since might be out of bounds
				var neighbourLeft:Bool = (!map.GetNodeByIndex(x_ - 1, currentY).GetTraversable() && map.GetNodeByIndex(x_ - 1, currentY + dy_).GetTraversable());
				var neighbourRight:Bool = (!map.GetNodeByIndex(x_ + 1, currentY).GetTraversable() && map.GetNodeByIndex(x_ + 1, currentY + dy_).GetTraversable());
				
				//since we have found a jump point at the current node, we are taking the continuation node (the one we would have visited next) and any forced neighbours
				var returnee = [map.GetNodeByIndex(x_, currentY + dy_).GetTraversable() ? map.GetNodeByIndex(x_, currentY + dy_) : null,
				neighbourLeft ? map.GetNodeByIndex(x_ - 1, currentY + dy_) : null,
				neighbourRight ? map.GetNodeByIndex(x_ + 1, currentY + dy_) : null];
				
				if (expanding_)
				{
					// since during diagonal axis checking we do no 'expand' the nodes, the return is also ignored, so avoid setting parents since nullnull
					for (i in returnee)
					{
						if (i != null)
						{
							i.SetParent(currentNode);
							#if action_output
							actionOutput.SetParent(i, currentNode);
							#end
						}
					}
				}
				
				return returnee;
			}
			
			currentY += dy_;
		}
	
		return null; // we hit the end of the map, either 0 or map.height
	}
	
	function JumpDiagonal(x_:Int, y_:Int, dx_:Int, dy_:Int, length_:Int):Array<Node>
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
			actionOutput.SetParent(currentNode, map.GetNodeByIndex(currentX - dx_, currentY - dy_));
			#end
			searchedMap.SetTraversableTrue(currentX, currentY);
			currentNode.SetPathCost(currentNode.GetParent().GetPathCost() + 1.4);
			
			//check horizontal + vertical directions
			var horizontal = JumpHorizontal(currentX + dx_, currentY, dx_, length_, false);
			var vertical = JumpVertical(currentX, currentY + dy_, dy_, length_, false);
			if (horizontal != null || vertical != null)
			{
				//since we have found a jump point at the current node, we are taking the continuation node (the one we would have visited next) and the direction neighbours relating to the axis
				var returnee:Array<Node> = [map.GetNodeByIndex(currentX + dx_, currentY + dy_).GetTraversable() ? map.GetNodeByIndex(currentX + dx_, currentY + dy_) : null,
				horizontal != null ? map.GetNodeByIndex(currentX + dx_, currentY) : null,
				vertical != null ? map.GetNodeByIndex(currentX, currentY + dy_) : null];
				
				// set the parents
				for (i in returnee)
				{
					if (i != null)
					{
						i.SetParent(currentNode);
						#if action_output
						actionOutput.SetParent(i, currentNode);
						#end
					}
				}
				
				return returnee;
			}
			
			// check to see if the current node has a forced neighbour
			if ((currentNode == endNode) ||
			((currentY + dy_ >= 0 && currentY + dy_ < map.GetHeight() && currentX + dx_ >= 0 && currentX + dx_ < map.GetWidth()) && 
			((!map.GetNodeByIndex(currentX - dx_, currentY).GetTraversable() && map.GetNodeByIndex(currentX - dx_, currentY + dy_).GetTraversable()) ||
			(!map.GetNodeByIndex(currentX, currentY - dy_).GetTraversable() && map.GetNodeByIndex(currentX + dx_, currentY - dy_).GetTraversable()))))
			{
				// a bit irritating but these could be out of bounds
				var neighbourAboveLeft:Bool = (!map.GetNodeByIndex(currentX - dx_, currentY).GetTraversable() && map.GetNodeByIndex(currentX - dx_, currentY + dy_).GetTraversable());
				var neighbourBelowRight:Bool = (!map.GetNodeByIndex(currentX, currentY - dy_).GetTraversable() && map.GetNodeByIndex(currentX + dx_, currentY - dy_).GetTraversable());
				
				//since we have found a jump point at the current node, we are taking the continuation node (the one we would have visited next) and any forced neighbours
				var returnee:Array<Node> = [map.GetNodeByIndex(currentX + dx_, currentY + dy_).GetTraversable() ? map.GetNodeByIndex(currentX + dx_, currentY + dy_) : null,
				neighbourAboveLeft ? map.GetNodeByIndex(currentX - dx_, currentY + dy_) : null,
				neighbourBelowRight ? map.GetNodeByIndex(currentX + dx_, currentY - dy_) : null];
				
				// set the parents
				for (i in returnee)
				{
					if (i != null)
					{
						i.SetParent(currentNode);
						#if action_output
						actionOutput.SetParent(i, currentNode);
						#end
					}
				}
				
				return returnee;
			}
			
			currentX += dx_;
			currentY += dy_;
		}
	
		return null; // we hit the end of the map, either 0 or map.height
		
	}
	
	public function GetActionOutput():ActionOutput
	{
		#if action_output
		return actionOutput;
		#else
		throw "PathPlanner library has not been compiled with action output as needed. Recompile with compiler command -D action_output";
		#end
	}
	
}