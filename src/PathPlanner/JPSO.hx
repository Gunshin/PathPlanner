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
class JPSO implements IPathfinder 
{
	@:protected
	var map:GraphGridMap;
	@:protected
	var endNode:Node;
	
	@:protected
	var searchedMap:GraphGridMapMinimalist;
	
	@:protected
	var heuristicFunction:
	#if cs
	cs.system.Func_3<Position,Position,Float>;
	#else
	Position -> Position -> Float;
	#end
	
	#if action_output
	@:protected
	var actionOutput:ActionOutput;
	#end
	
	/*var verticalTimer:DebugRunningTimer = new DebugRunningTimer();
	var horizontalTimer:DebugRunningTimer = new DebugRunningTimer();
	var diagonalTimer:DebugRunningTimer = new DebugRunningTimer();
	var diagHorizTimer:DebugRunningTimer = new DebugRunningTimer();
	var diagVertTimer:DebugRunningTimer = new DebugRunningTimer();
	var jumpTimer:DebugRunningTimer = new DebugRunningTimer();
	var improveTimer:DebugRunningTimer = new DebugRunningTimer();
	var findTimer:DebugRunningTimer = new DebugRunningTimer();
	var reconstructTimer:DebugRunningTimer = new DebugRunningTimer();
	var t:DebugRunningTimer = new DebugRunningTimer();*/
		
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
	
	public function FindPath(param_:PathplannerParameter):Array<Position>
	{
		/*verticalTimer.Reset();
		horizontalTimer.Reset();
		diagonalTimer.Reset();
		diagHorizTimer.Reset();
		diagVertTimer.Reset();
		jumpTimer.Reset();
		improveTimer.Reset();
		findTimer.Reset();
		reconstructTimer.Reset();
		t.Reset();*/
		
		//findTimer.Start();
		
		var startNode:Node;
		
		startNode = param_.startNode != null ? param_.startNode : map.GetNodeByIndex(param_.startX, param_.startY);
		endNode = param_.goalNode != null ? param_.goalNode : map.GetNodeByIndex(param_.goalX, param_.goalY);
		
		if (endNode == null || startNode == null)
		{
			throw "start or goal has not been set by parameter input!";
			return null;
		}
		
		searchedMap.SetMap(false);
		
		#if action_output
		actionOutput = new ActionOutput();
		#end
		
		startNode.SetParent(null);
		
		// if we dont do this, the algorithm attempts to search the start node again which results in a cyclic reference
		searchedMap.SetTraversableTrue(startNode.GetPosition().GetX(), startNode.GetPosition().GetY());
		
		var open:PriorityQueue<Node> = new PriorityQueue<Node>(true, 128);
		
		open.enqueue(startNode);
		#if action_output
		actionOutput.AddAction("AddToOpen", startNode.GetPosition(), null);
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
				trace("diagVert: " + (diagVertTimer.GetCurrentTotalTime() * 1000000));
				trace("diagHoriz: " + (diagHorizTimer.GetCurrentTotalTime() * 1000000));
				trace("jump: " + (jumpTimer.GetCurrentTotalTime() * 1000000));
				trace("improve: " + (improveTimer.GetCurrentTotalTime() * 1000000));
				trace("_______t______: " + (t.GetCurrentTotalTime() * 1000000));*/
				
				
				//reconstructTimer.Start();
				var path = PathUtility.ReconstructPathFromNodes(endNode);
				//reconstructTimer.Stop();
				
				//trace("reconstruct: " + (reconstructTimer.GetCurrentTotalTime() * 1000000));
				
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
		actionOutput.AddAction("Expand", currentNode_.GetPosition(), null);
		#end
		
		for (neighbour in map.GetRawNeighbours(currentNode_))
		{
			if (neighbour != null)
			{
				//jumpTimer.Start();
				var result = Jump(neighbour, currentNode_, 0);
				//jumpTimer.Stop();
				
				// we dont want to do anything with this jump point if we already searched it
				if (result != null && (!searchedMap.GetTraversable(result.GetPosition().GetX(), result.GetPosition().GetY()) || 
				currentNode_.GetPathCost() + Position.Distance(result.GetPosition(), currentNode_.GetPosition()) < result.GetPathCost()))
				{
					searchedMap.SetTraversableTrue(result.GetPosition().GetX(), result.GetPosition().GetY());
					// set the new jump point (result) to have its parent be the current expanded jump point
					result.SetParent(currentNode_);
					#if action_output
					actionOutput.AddAction("SetParent", result.GetPosition(), currentNode_.GetPosition());
					#end
					
					// special case if the result is the end node
					if (result == endNode)
					{
						result.priority = 0;
						open_.enqueue(result);
						return;
					}
					
					// found a jump point, lets add it.
					AddToQueue(result, open_);
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
		
		node_.SetPathCost(node_.GetParent().GetPathCost() + Position.Distance(node_.GetPosition(), node_.GetParent().GetPosition()));
		
		node_.heuristic = 
		#if cs
		heuristicFunction.Invoke(node_.GetPosition(), endNode.GetPosition());
		#else
		heuristicFunction(node_.GetPosition(), endNode.GetPosition());
		#end
		
		#if action_output
		actionOutput.AddAction("AddToOpen", node_.GetPosition(), null);
		#end
		
		if (!open_.contains(node_))
		{
			node_.priority = node_.GetPathCost() + node_.heuristic;
			open_.enqueue(node_);
		}
		else
		{
			open_.reprioritize(node_, node_.GetPathCost() + node_.heuristic);
		}
	}
	
	function Jump(node_:Node, parentNode_:Node, length_:Int):Node
	{
		#if debugging
		DebugLogger.Assert(node_ != null, "JPS:Jump: node_ is null");
		DebugLogger.Assert(parentNode_ != null, "JPS:Jump: parentNode_ is null");
		#end

		var x:Int = node_.GetPosition().GetX();
		var y:Int = node_.GetPosition().GetY();
		var dx:Int = cast(Math.min(Math.max((x - parentNode_.GetPosition().GetX()), -1), 1), Int);
		var dy:Int = cast(Math.min(Math.max((y - parentNode_.GetPosition().GetY()), -1), 1), Int);
		
		var result:Node = null;
		
		if (dx != 0 && dy != 0) // check diag first since we apply a horizontal and vertical search on the node inside JumpDiagonal anyways (dont need to do it twice)
		{
			//diagonalTimer.Start();
			result = JumpDiagonal(x, y, dx, dy, length_);
			//diagonalTimer.Stop();
		}
		else if (dx != 0)
		{
			//horizontalTimer.Start();
			result = JumpHorizontal(x, y, dx, length_, true);
			//horizontalTimer.Stop();
		}else if (dy != 0)
		{
			//verticalTimer.Start();
			result = JumpVertical(x, y, dy, length_, true);
			//verticalTimer.Stop();
		}
		
		return result;
	}
	
	function JumpHorizontal(x_:Int, y_:Int, dx_:Int, length_:Int, expanding_:Bool):Node
	{
		//horizontalTimer.Start();
		//var endTile_:Int = x_ + (length_ * dx_);
		/*
		 * for now im going to make it simple and just have it search as far as possible
		 * may be stupid on huge open terrain spaces as checking this horizontal line may not provide a good
		 * result
		 */
		var currentX:Int = x_;
		var aboveInMap:Bool = (y_ + 1 < map.GetHeight());
		var belowInMap:Bool = (y_ - 1 >= 0);
		
		// cache efforts
		var currentNodeAbove:Node = map.GetNodeByIndex(currentX, y_ + 1);
		var currentNodeBelow:Node = map.GetNodeByIndex(currentX, y_ - 1);
		var currentNodeAboveRight:Node = map.GetNodeByIndex(currentX + dx_, y_ + 1);
		var currentNodeBelowRight:Node = map.GetNodeByIndex(currentX + dx_, y_ - 1);
		
		while (currentX >= 0 && currentX < map.GetWidth())
		{
			
			var currentNode:Node = map.GetNodeByIndex(currentX, y_);
			
			//check to see if current node is traversable
			if (!currentNode.GetTraversable())
			{
				// we hit a dead end
				//horizontalTimer.Stop();
				return null;
			}
			
			#if action_output
			actionOutput.AddAction("Explored", currentNode.GetPosition(), null);
			#end
			
			// removed from if statement so i could time
			var flag:Bool = (currentNode == endNode) ||
			((currentX + dx_ >= 0 && currentX + dx_ < map.GetWidth()) &&
			((aboveInMap && (!currentNodeAbove.GetTraversable() && currentNodeAboveRight.GetTraversable())) || // forced neighbour above
			(belowInMap && (!currentNodeBelow.GetTraversable() && currentNodeBelowRight.GetTraversable()))));
			
			// check to see if the current node has a forced neighbour, or is the end node
			if (flag)// forced neighbour below
			{
				//horizontalTimer.Stop();
				return currentNode;
			}
			
			currentX += dx_;

			// attempting all caching efforts to reduce time
			currentNodeAbove = currentNodeAboveRight;
			currentNodeAboveRight = map.GetNodeByIndex(currentX + dx_, y_ + 1);
			currentNodeBelow = currentNodeBelowRight;
			currentNodeBelowRight = map.GetNodeByIndex(currentX + dx_, y_ - 1);
		}
		//horizontalTimer.Stop();
		return null; // we hit the end of the map, either 0 or map.width
	}
	
	function JumpVertical(x_:Int, y_:Int, dy_:Int, length_:Int, expanding_:Bool):Node
	{
		//verticalTimer.Start();
		//var endTile_:Int = x_ + (length_ * dx_);
		/*
		 * for now im going to make it simple and just have it search as far as possible
		 * may be stupid on huge open terrain spaces as checking this vertical line may not provide a good
		 * result
		 */
		var currentY:Int = y_;
		var rightInMap:Bool = (x_ + 1 < map.GetWidth());
		var leftInMap:Bool = (x_ - 1 >= 0);
		
		// cache efforts
		var currentNodeRight:Node = map.GetNodeByIndex(x_ + 1, currentY);
		var currentNodeLeft:Node = map.GetNodeByIndex(x_ - 1, currentY);
		var currentNodeAboveRight:Node = map.GetNodeByIndex(x_ + 1, currentY + dy_);
		var currentNodeAboveLeft:Node = map.GetNodeByIndex(x_ - 1, currentY + dy_);
		
		while (currentY >= 0 && currentY < map.GetHeight())
		{
			
			var currentNode:Node = map.GetNodeByIndex(x_, currentY);
			
			//check to see if current node is traversable
			if (!currentNode.GetTraversable())
			{
				//verticalTimer.Stop();
				// we hit a dead end
				return null;
			}
			
			#if action_output
			actionOutput.AddAction("Explored", currentNode.GetPosition(), null);
			#end
			
			//diagVertTimer.Start();
			var flag:Bool = (currentNode == endNode) ||
			((currentY + dy_ >= 0 && currentY + dy_ < map.GetHeight()) &&
			((rightInMap && (!currentNodeRight.GetTraversable() && currentNodeAboveRight.GetTraversable())) || // forced neighbour right
			(leftInMap && (!currentNodeLeft.GetTraversable() && currentNodeAboveLeft.GetTraversable()))));
			//diagVertTimer.Stop();
			// check to see if the current node has a forced neighbour
			if (flag)// forced neighbour left
			{
				//verticalTimer.Stop();
				return currentNode;
			}
			
			currentY += dy_;
			
			// attempting all caching efforts to reduce time
			currentNodeRight = currentNodeAboveRight;
			currentNodeAboveRight = map.GetNodeByIndex(x_ + 1, currentY + dy_);
			currentNodeLeft = currentNodeAboveLeft;
			currentNodeAboveLeft = map.GetNodeByIndex(x_ - 1, currentY + dy_);
		}
		//verticalTimer.Stop();
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
			
			//check to see if current node is traversable
			if ((currentY < 0 || currentY >= map.GetHeight() || currentX < 0 || currentX >= map.GetWidth()) || 
			!currentNode.GetTraversable())
			{
				// we hit a dead end
				return null;
			}
			
			#if action_output
			actionOutput.AddAction("Explored", currentNode.GetPosition(), null);
			#end
			
			//check horizontal + vertical directions
			var horizontal = JumpHorizontal(currentX + dx_, currentY, dx_, length_, false);
			var vertical = JumpVertical(currentX, currentY + dy_, dy_, length_, false);
			if (horizontal != null || vertical != null)
			{
				return currentNode;
			}
			
			// check to see if the current node has a forced neighbour
			if ((currentNode == endNode) ||
			((currentY + dy_ >= 0 && currentY + dy_ < map.GetHeight() && currentX + dx_ >= 0 && currentX + dx_ < map.GetWidth()) && 
			((!map.GetNodeByIndex(currentX - dx_, currentY).GetTraversable() && map.GetNodeByIndex(currentX - dx_, currentY + dy_).GetTraversable()) ||
			(!map.GetNodeByIndex(currentX, currentY - dy_).GetTraversable() && map.GetNodeByIndex(currentX + dx_, currentY - dy_).GetTraversable()))))
			{
				return currentNode;
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