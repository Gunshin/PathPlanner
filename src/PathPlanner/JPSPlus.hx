package pathPlanner;

import de.polygonal.ds.PriorityQueue;

#if cs
import cs.Lib;
#end

// the priority queue of polygonal requires me to implement two variables so that the element can be sorted correctly. However, i do not want
// to bloat my Position classes with unnecessary data. In the future, i may swap out Position for an index since that is all i use position for.
class PositionWrapper
{
	public var position:Int;
	public var priority:Float = 0;
	
	public var element:Position;
	
	public function new(priority_:Float, element_:Position)
	{
		priority = priority_;
		element = element_;
	}
}

/**
 * ...
 * @author Michael Stephens
 * 
 * For now we will be simplifying the types to allow for simplicity, but further on i may turn towards vectors and remove these classes
 * 
 */
class JPSPlus implements IPathfinder 
{
	@:protected
	var map:GraphGridMapMinimalist;
	@:protected
	var mapInv:GraphGridMapMinimalist;
	
	@:protected
	var mapRotated:GraphGridMapMinimalist;
	@:protected
	var mapRotatedInv:GraphGridMapMinimalist;
	
	@:protected
	var searchedMap:GraphGridMapMinimalist;
	
	@:protected
	var parentGrid:Array<Position> = new Array<Position>();
	
	@:protected
	var pathCostGrid:Array<Float> = new Array<Float>();
	
	@:protected
	var heuristicFunction:
	#if cs
	cs.system.Func_3<Position,Position,Float>;
	#else
	Position -> Position -> Float;
	#end
	
	@:protected
	var endCoord:Position;
	
	#if action_output
	@:protected
	var actionOutput:ActionOutput<Position>;
	#end
		
	public function new(map_:GraphGridMapMinimalist, heuristicFunction_:
		#if cs
		cs.system.Func_3<Position,Position,Float>
		#else
		Position -> Position -> Float
		#end
		)
	{
		map = map_;
		
		heuristicFunction = heuristicFunction_;
	}
	
	public function FindPath(param_:PathplannerParameter):Array<Position>
	{
		#if debugging
		DebugLogger.Assert(param_.startX < map.GetWidth() && param_.startX >= 0, "Input param.startX is not inside map bounds: " + param_.startX);
		DebugLogger.Assert(param_.startY < map.GetHeight() && param_.startY >= 0, "Input param.startY is not inside map bounds: " + param_.startY);
		DebugLogger.Assert(param_.goalX < map.GetWidth() && param_.goalX >= 0, "Input param.goalX is not inside map bounds: " + param_.goalX);
		DebugLogger.Assert(param_.goalY < map.GetHeight() && param_.goalY >= 0, "Input param.goalY is not inside map bounds: " + param_.goalY);
		#end
		
		/*verticalTimer.Reset();
		horizontalTimer.Reset();
		diagonalTimer.Reset();
		jumpTimer.Reset();
		improveTimer.Reset();*/
		//findTimer.Reset();
		//reconstructTimer.Reset();
		
		//findTimer.Start();
		
		// generate a fresh rotated map as there may have been changes to the original map not reflected into the rotated
		// i should probably just merge the two and take the slight performance hit when updating the graph
		mapRotated = map.RotateMap();
		mapInv = map.InvertMap();
		mapRotatedInv = mapRotated.InvertMap();
		
		parentGrid = new Array<Position>();
		pathCostGrid = new Array<Float>();
		
		var startCoord:Position = new Position(param_.startX, param_.startY);
		endCoord = new Position(param_.goalX, param_.goalY);
		
		searchedMap.SetMap(false);

		#if action_output
		actionOutput.ResetActionList();
		#end
		
		searchedMap.SetTraversableTrue(startCoord.GetX(), startCoord.GetY()); // if we dont do this, the algorithm attempts to search the start node again which results in a cyclic reference
		
		var open:PriorityQueue<PositionWrapper> = new PriorityQueue<PositionWrapper>(true, 128);
		
		// the start node is being treat as a jump point, so that its neighbours are added to the list
		open.enqueue(new PositionWrapper(0, startCoord));
		#if action_output
		actionOutput.AddAction("AddToOpen", startCoord, null);
		#end
		
		while (!open.isEmpty())
		{
			
			var currentPosition:Position = open.dequeue().element;
			
			if (Position.Equal(currentPosition, endCoord)) //if we have the goal, return?
			{
				
				
				//trace("jps __________________________________");
				/*trace("vert: " + (verticalTimer.GetCurrentTotalTime() * 1000000));
				trace("hori: " + (horizontalTimer.GetCurrentTotalTime() * 1000000));
				trace("diag: " + (diagonalTimer.GetCurrentTotalTime() * 1000000));
				trace("jump: " + (jumpTimer.GetCurrentTotalTime() * 1000000));
				trace("improve: " + (improveTimer.GetCurrentTotalTime() * 1000000));
				
				
				reconstructTimer.Start();*/
				//findTimer.Stop();
				var path = PathUtility.ReconstructPathFromPositionMap(endCoord, parentGrid, map.GetWidth());
				/*reconstructTimer.Stop();
				
				trace("reconstruct: " + (reconstructTimer.GetCurrentTotalTime() * 1000000));*/
				
				//findTimer.Stop();
				//trace("find: " + (findTimer.GetCurrentTotalTime() * 1000000));
				
				return path;
			}
			
			//improveTimer.Start();
			Improve(currentPosition, open);
			//improveTimer.Stop();
		}
		return null;// no path is found
	}
	
	function Improve(currentPosition_:Position, open_:PriorityQueue<PositionWrapper>):Void
	{
		#if debugging
		DebugLogger.Assert(currentPosition_ != null, "JPSPlus:Improve: currentNode is null");
		DebugLogger.Assert(open_ != null, "JPSPlus:Improve: open_ is null");
		#end
		#if action_output
		actionOutput.AddAction("Expand", currentPosition_, null);
		#end
		
		// loop through this positions direct neighbours
		for (i in -1...2)
		{
			for (j in -1...2)
			{
				var x:Int = j + currentPosition_.GetX();
				var y:Int = i + currentPosition_.GetY();
				
				// make sure we are in the map as a neighbour of a node could reference something not inside
				// make sure we dont try to run a neighbour on the currentPosition (i != 0 && j != 0)
				if (x >= 0 && x < map.GetWidth() &&
				y >= 0 && x < map.GetHeight() &&
				i != 0 && j != 0)
				{
					
					//jumpTimer.Start();
					var result = Jump(currentPosition_, x, y);
					//jumpTimer.Stop();
					
					// we dont want to do anything with this jump point if we already searched it
					if (result != null && !searchedMap.GetTraversable(result.GetX(), result.GetY()))
					{
						searchedMap.SetTraversableTrue(result.GetX(), result.GetY());
						
						// set the new jump point (result) to have its parent be the current expanded jump point
						parentGrid[result.GetX() + result.GetY() * map.GetWidth()] = currentPosition_;
						#if action_output
						actionOutput.AddAction("SetParent", result, currentPosition_);
						#end
						
						// special case if the result is the end node
						if (Position.Equal(result, endCoord))
						{
							open_.enqueue(new PositionWrapper(0, result));
							return;
						}
						
						// found a jump point, lets add it.
						AddToQueue(result, open_);
					}
					
				}
			}
		}
	}
	
	function AddToQueue(position_:Position, open_:PriorityQueue<PositionWrapper>)
	{
		var parent:Position = parentGrid[position_.GetX() + position_.GetY() * map.GetWidth()];
		
		#if debugging
		DebugLogger.Assert(node_ != null, "position_: " + position_.ToString() + " is null");
		DebugLogger.Assert(parent != null, "position_: " + position_.ToString() + " parent map is null");
		#end
		
		var pathCost:Float = pathCostGrid[position_.GetX() + position_.GetY() * map.GetWidth()] = pathCostGrid[parent.GetX() + parent.GetY() * map.GetWidth()] + Position.Distance(position_, parent);
		
		var heuristic:Float = 
		#if cs
		heuristicFunction.Invoke(position_, endCoord);
		#else
		heuristicFunction(position_, endCoord);
		#end
		
		#if action_output
		actionOutput.AddAction("AddToOpen", position_, null);
		#end
		
		open_.enqueue(new PositionWrapper(pathCost + heuristic, position_));
	}
	
	function Jump(currentPosition_:Position, neighbourX_:Int, neighbourY_:Int):Position
	{
		#if debugging
		DebugLogger.Assert(currentPosition_ != null, "JPSPlus:Jump: currentPosition_ is null");
		DebugLogger.Assert(neighbourPosition_ != null, "JPSPlus:Jump: neighbourPosition_ is null");
		#end

		var dx:Int = cast(Math.min(Math.max((neighbourX_ - currentPosition_.GetX()), -1), 1), Int);
		var dy:Int = cast(Math.min(Math.max((neighbourY_ - currentPosition_.GetY()), -1), 1), Int);
		
		var result:Position = null;
		
		if (dx != 0 && dy != 0) // check diag first since we apply a horizontal and vertical search on the node inside JumpDiagonal anyways (dont need to do it twice)
		{
			//diagonalTimer.Start();
			result = JumpDiagonal(neighbourX_, neighbourY_, dx, dy);
			//diagonalTimer.Stop();
		}
		else if (dx != 0)
		{
			//horizontalTimer.Start();
			result = JumpHorizontal(neighbourX_, neighbourY_, dx);
			//horizontalTimer.Stop();
		}else if (dy != 0)
		{
			//verticalTimer.Start();
			result = JumpVertical(neighbourX_, neighbourY_, dy);
			//verticalTimer.Stop();
		}
		
		return result;
	}
	
	function JumpHorizontal(x_:Int, y_:Int, dx_:Int):Position
	{
		var aboveInMap:Bool = (y_ + 1 < map.GetHeight());
		var belowInMap:Bool = (y_ - 1 >= 0);
		
		// if we are searching right
		if (dx_ > 0)
		{
			var possibleLength:Int = map.CheckBitsRight(x_, y_);
			var jumpPositionAbove:Int = map.GetWidth() + 1; // put the default index outside of possible values so that the min returns the smallest correct number
			if (aboveInMap)
			{
				var aboveCheck:Int = map.CheckBitsRight(x_, y_ + 1);
				jumpPositionAbove =  aboveCheck + mapInv.CheckBitsRight(x_ + aboveCheck, y_ + 1) - 1;
			}
			
			var jumpPositionBelow:Int = map.GetWidth() + 1; // put the default index outside of possible values so that the min returns the smallest correct number
			if (aboveInMap)
			{
				var belowCheck:Int = map.CheckBitsRight(x_, y_ - 1);
				jumpPositionBelow = belowCheck + mapInv.CheckBitsRight(x_ + belowCheck, y_ - 1) - 1;
			}
			
			// this is guaranteed to work unless the map is some stupid 1 row configuration
			// (if 2 or more rows are present, regardless of which row it runs in, one of the rows will return a value in map)
			var closestJumpPosition:Int = cast(Math.min(jumpPositionAbove, jumpPositionBelow), Int);
			
			// we check to see if it is less than since the node could be at the very end of the map.
			// may produce an irritating result if we expect to be able to traverse through diagonals if there are no horizontal or vertical traversals
			// F T // this is an example where T = traversable
			// T F
			if (closestJumpPosition < possibleLength)
			{
				return new Position(x_ + closestJumpPosition, y_);
			}
		}
		else // we are searching left
		{
			var possibleLength:Int = map.CheckBitsLeft(x_, y_);
			var jumpPositionAbove:Int = map.GetWidth() + 1; // put the default index outside of possible values so that the min returns the smallest correct number
			if (aboveInMap)
			{
				var aboveCheck:Int = map.CheckBitsLeft(x_, y_ + 1);
				jumpPositionAbove =  aboveCheck + mapInv.CheckBitsLeft(x_ - aboveCheck, y_ + 1) - 1;
			}
			
			var jumpPositionBelow:Int = map.GetWidth() + 1; // put the default index outside of possible values so that the min returns the smallest correct number
			if (aboveInMap)
			{
				var belowCheck:Int = map.CheckBitsLeft(x_, y_ - 1);
				jumpPositionBelow = belowCheck + mapInv.CheckBitsLeft(x_ - belowCheck, y_ - 1) - 1;
			}
			
			// this is guaranteed to work unless the map is some stupid 1 row configuration
			// (if 2 or more rows are present, regardless of which row it runs in, one of the rows will return a value in map)
			var closestJumpPosition:Int = cast(Math.min(jumpPositionAbove, jumpPositionBelow), Int);
			
			// we check to see if it is less than since the node could be at the very end of the map.
			// may produce an irritating result if we expect to be able to traverse through diagonals if there are no horizontal or vertical traversals
			// F T // this is an example where T = traversable
			// T F
			if (closestJumpPosition < possibleLength)
			{
				return new Position(x_ - closestJumpPosition, y_);
			}
		}
		
		return null; // we hit the end of the map, either 0 or map.width
	}
	
	function JumpVertical(x_:Int, y_:Int, dy_:Int):Node
	{
		var rightInMap:Bool = (x_ + 1 < map.GetWidth());
		var leftInMap:Bool = (x_ - 1 >= 0);
		
		// if we are searching up
		if (dy_ > 0)
		{
			var possibleLength:Int = mapRotated.CheckBitsRight(y_, x_);
			var jumpPositionRight:Int = map.GetHeight() + 1; // put the default index outside of possible values so that the min returns the smallest correct number
			if (rightInMap)
			{
				var rightCheck:Int = map.CheckBitsRight(y_, x_ + 1); // the positive x axis on the inverted map equals the positive y axis of the normal map
				jumpPositionRight =  rightCheck + mapInv.CheckBitsRight(y_ + rightCheck, x_ + 1) - 1;
			}
			
			var jumpPositionLeft:Int = map.GetHeight() + 1; // put the default index outside of possible values so that the min returns the smallest correct number
			if (leftInMap)
			{
				var leftCheck:Int = map.CheckBitsRight(y_, x_ - 1);
				jumpPositionLeft = leftCheck + mapInv.CheckBitsRight(y_ + leftCheck, x_ - 1) - 1;
			}
			
			// this is guaranteed to work unless the map is some stupid 1 row configuration
			// (if 2 or more rows are present, regardless of which row it runs in, one of the rows will return a value in map)
			var closestJumpPosition:Int = cast(Math.min(jumpPositionRight, jumpPositionLeft), Int);
			
			// we check to see if it is less than since the node could be at the very end of the map.
			// may produce an irritating result if we expect to be able to traverse through diagonals if there are no horizontal or vertical traversals
			// F T // this is an example where T = traversable
			// T F
			if (closestJumpPosition < possibleLength)
			{
				return new Position(x_, y_ + closestJumpPosition);
			}
		}
		else // we are searching left
		{
			var possibleLength:Int = map.CheckBitsLeft(y_, x_);
			var jumpPositionRight:Int = map.GetHeight() + 1; // put the default index outside of possible values so that the min returns the smallest correct number
			if (rightInMap)
			{
				var rightCheck:Int = map.CheckBitsLeft(y_, x_ + 1);
				jumpPositionRight =  rightCheck + mapInv.CheckBitsLeft(y_ - rightCheck, x_ + 1) - 1;
			}
			
			var jumpPositionLeft:Int = map.GetHeight() + 1; // put the default index outside of possible values so that the min returns the smallest correct number
			if (leftInMap)
			{
				var leftCheck:Int = map.CheckBitsLeft(y_, x_ - 1);
				jumpPositionLeft = leftCheck + mapInv.CheckBitsLeft(y_ - leftCheck, x_ - 1) - 1;
			}
			
			// this is guaranteed to work unless the map is some stupid 1 row configuration
			// (if 2 or more rows are present, regardless of which row it runs in, one of the rows will return a value in map)
			var closestJumpPosition:Int = cast(Math.min(jumpPositionRight, jumpPositionLeft), Int);
			
			// we check to see if it is less than since the node could be at the very end of the map.
			// may produce an irritating result if we expect to be able to traverse through diagonals if there are no horizontal or vertical traversals
			// F T // this is an example where T = traversable
			// T F
			if (closestJumpPosition < possibleLength)
			{
				return new Position(x_, y_ - closestJumpPosition);
			}
		}
		
		return null; // we hit the end of the map, either 0 or map.width
	}
	
	function JumpDiagonal(x_:Int, y_:Int, dx_:Int, dy_:Int):Node
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
			actionOutput.AddAction("Explored", currentNode, null);
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
	
	function TracePath(endGoal_:Position):Array<Position>
	{
		
	}
	
	public function GetActionOutput():ActionOutput<Position>
	{
		#if action_output
		return actionOutput;
		#else
		throw "PathPlanner library has not been compiled with action output as needed. Recompile with compiler command -D action_output";
		#end
	}
	
	
}