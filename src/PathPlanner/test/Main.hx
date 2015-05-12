package pathPlanner.test;

import haxe.ds.Vector;
import haxe.io.Eof;
import haxe.io.Path;
import haxe.macro.Expr.Position;
import haxe.Timer;
import pathPlanner.DebugRunningTimer;
import pathPlanner.GraphGridMapMinimalist;
import pathPlanner.GraphHierarchical;
import pathPlanner.JPSM;
import pathPlanner.PathplannerParameter;

import pathPlanner.IPathfinder;
import pathPlanner.Node;
import pathPlanner.test.Main.PathResult;
import pathPlanner.Position;
import sys.io.File;
import sys.io.FileOutput;

import pathPlanner.GraphGridMap;

typedef Tester = {x:Int, y:Int, flag:Bool}
typedef Path = { start:Node, end:Node, optimalLength:Float }
typedef PathResult = { pathplanner:IPathfinder, pathLength: Float, timeTaken: Float, actionCount: Int, path: Array<Position> }

/**
 * ...
 * @author Michael Stephens
 */
class Main 
{
	
	#if debug
	function Print(message_:String)
	{
		trace(message_);
	}
	#end
	
	public function new()
	{
		
		var graph:GraphGridMap = new GraphGridMap(16, 16);
		var hier:GraphHierarchical = new GraphHierarchical();
		hier.GenerateFromGridGraph(graph);
		hier.GenerateHierarchy(4);
		
		//trace(hier.GetLevelHierarchy(0).length);
		//trace(hier.GetLevelHierarchy(1).length);
		//trace(hier.GetLevelHierarchy(2).length);
		for (node in hier.GetLevelHierarchy(1))
		{
			trace(node.GetPosition().ToString() + " ___ " + node.GetNeighbours().length);
		}
		
		for (node in hier.GetLevelHierarchy(4))
		{
			trace(node.GetPosition().ToString() + " _____ " + NodeHierarchical.GetAveragePosition(node.GetHierarchicalChildren()).ToString());
			for (child in node.GetHierarchicalChildren())
			{
				trace(child.GetPosition().ToString());
			}
		}
		
		
		/*var minMap:GraphGridMapMinimalist = new GraphGridMapMinimalist(32, 32, true);
		minMap.SetTraversableFalse(7, 1);
		minMap.SetTraversableFalse(4, 2);
		minMap.SetTraversableFalse(5, 2);
		minMap.SetTraversableFalse(2, 10);
		
		var rMap:GraphGridMapMinimalist = minMap.RotateMap();
		
		var invMap:GraphGridMapMinimalist = minMap.InvertMap();
		
		var x:Int = 2;
		
		var possibleLength:Int = minMap.CheckBitsRight(x, 1);
		var belowRightMinMap:Int = minMap.CheckBitsRight(x, 2);
		var jumpPositionBelow:Int = belowRightMinMap + invMap.CheckBitsRight(x + belowRightMinMap, 2) - 1;
		
		var aboveRightMinMap:Int = minMap.CheckBitsRight(x, 0);
		var jumpPositionAbove:Int = aboveRightMinMap + invMap.CheckBitsRight(x + aboveRightMinMap, 0) - 1;
		
		var closestJumpPosition:Int = cast(Math.min(jumpPositionAbove, jumpPositionBelow), Int);
		
		var finalJumpPosition:Int = -1;
		
		if (closestJumpPosition <= possibleLength)
		{
			finalJumpPosition = closestJumpPosition;
		}
		
		trace(finalJumpPosition);*/
		
		/*var num:haxe.Int32 = 0;
		var s = Timer.stamp();
		var r = PathUtility.CTZ(num);
		//trace(StringTools.hex(num));
		trace((Timer.stamp() - s) * 1000000 + " _ " + r);
		trace((num >> 2) & 1);*/
		
		//var graph:GraphGridMapMinimalist = new GraphGridMapMinimalist(64, 64, false);
		//graph.SetTraversableTrue(0, 0);
		//graph.SetTraversableTrue(19, 0);
		//trace(graph.GetTraversable(1, 0));
		//trace(graph.GetTraversable(19, 0));
		//trace(graph.CheckBitsLeft(60, 0));
		
		/*var rotated:GraphGridMapMinimalist = graph.RotateMap();
		for (i in 0...rotated.GetWidth())
		{
			for (j in 0...rotated.GetHeight())
			{
				if(!rotated.GetTraversable(i, j))
				trace(i + " _ " + j);
			}
		}*/
		
		//trace(rotated.GetTraversable(20, 54));
		//trace(rotated.CheckBitsRight(0, 0));
		
		#if debugging
		//DebugLogger.GetInstance().SetLoggingFunction(Print);
		#end
		
		/*var map = LoadMap("resources/DragonAgeMaps/arena2.map");
		var paths = LoadScenarios("resources/DragonAgeScenarios/arena2.map.scen", map, "	");
		
		var pathfinder:IPathfinder = new AStar(function(nodeOne, nodeTwo)
			{
				return Math.sqrt(Math.pow(nodeOne.GetX() - nodeTwo.GetX(), 2) + Math.pow(nodeOne.GetY() - nodeTwo.GetY(), 2));
			});
		var jpsp:IPathfinder = new JPSPlus(map.GenerateGraphGridMapMinimalist(), function(nodeOne, nodeTwo)
			{
				return Math.sqrt(Math.pow(nodeOne.GetX() - nodeTwo.GetX(), 2) + Math.pow(nodeOne.GetY() - nodeTwo.GetY(), 2));
			});
			//cast(jpsp, JPSPlus).AttachPrint(Print);
		
		var jpso:IPathfinder = new JPSO(map, function(nodeOne, nodeTwo)
			{
				return Math.sqrt(Math.pow(nodeOne.GetX() - nodeTwo.GetX(), 2) + Math.pow(nodeOne.GetY() - nodeTwo.GetY(), 2));
			});
		var jpsm:IPathfinder = new JPSM(map, function(nodeOne, nodeTwo)
			{
				return Math.sqrt(Math.pow(nodeOne.GetX() - nodeTwo.GetX(), 2) + Math.pow(nodeOne.GetY() - nodeTwo.GetY(), 2));
			});
			
		var path:Int = 255;*/
		
		//var paths = GeneratePaths(map, 50);
		/*var timerJPS = new DebugRunningTimer();
		for (i in 0...100000)
		{
			trace("starting JPS: " + i);
			timerJPS.Start();
			GetPath(jps, paths[path], map);
			timerJPS.Stop();
		}*/
		
		
		/*var timerAStar = new DebugRunningTimer();
		for (i in 0...1000)
		{
			trace("starting AStar: " + i);
			timerAStar.Start();
			GetPath(pathfinder, paths[path], map);
			timerAStar.Stop();
		}*/

		//trace("timerJPS took: " + (timerJPS.GetCurrentTotalTime() / 100000)/* + " timerAStar took: " + (timerAStar.GetCurrentTotalTime() / 1000)*/);
		/*trace("looking through: " + " _ " + paths[path].start.GetPosition().GetX() + "," + paths[path].start.GetPosition().GetY() + " t: " + paths[path].start.GetTraversable() + " _ " + paths[path].end.GetPosition().GetX() + "," + paths[path].end.GetPosition().GetY() + " t: " + paths[path].end.GetTraversable());
		ComparePath( GetPath(pathfinder, paths[path], map), GetPath(jpso, paths[path], map) , 1);
		ComparePath( GetPath(jpsm, paths[path], map), GetPath(jpsp, paths[path], map) , 1);*/
		
		/*var tests = [0, 128, 255, 379, 507, 627, 750];
		
		for (i in tests)
		{
			trace("A* path: " + i + " _ " + GetTime(1000, pathfinder, paths[i]));
			trace("JPSO: " + i + " _ " + GetTime(1000, jpso, paths[i]));
			trace("JPSM*: " + i + " _ " + GetTime(1000, jpsm, paths[i]));
			trace("JPSP*: " + i + " _ " + GetTime(1000, jpsp, paths[i]));
		}*/
		/*var i = 0;
		for (path in paths)
		{
			trace("looking through: " + i++ + " _ " + path.start.GetPosition().GetX() + "," + path.start.GetPosition().GetY() + " t: " +
			path.start.GetTraversable() + " _ " + path.end.GetPosition().GetX() + "," + path.end.GetPosition().GetY() + " t: " + path.end.GetTraversable());
			ComparePath( GetPath(jps, path, map), GetPath(pathfinder, path, map) , 0.4);
			//GetPath(jps, path, map); // currently using GetPath on just the A* algorithm to determine whether a scenario is viable
		}*/
	}
	
	/*public function PrintPath(pathStruct_:PathResult)
	{
		var path:Array<Node> = pathStruct_.path;
		trace("path length: " + path.length);
		for (i in 0...path.length)
		{
			trace(path[i].GetX() + " _ " + path[i].GetY());
		}
		
		trace("path length of: " + pathStruct_.pathLength);
	}*/
	
	public function GetTime(iterationCount_:Int, pathfinder_:IPathfinder, path_:Path):Float
	{
		var pathParam = new PathplannerParameter();
		pathParam.startNode = path_.start;
		pathParam.startX = path_.start.GetPosition().GetX();
		pathParam.startY = path_.start.GetPosition().GetY();
		pathParam.goalX = path_.end.GetPosition().GetX();
		pathParam.goalY = path_.end.GetPosition().GetY();
		pathParam.goalNode = path_.end;
		
		var timer = new DebugRunningTimer();
		for (i in 0...iterationCount_)
		{
			timer.Start();
			pathfinder_.FindPath(pathParam);
			timer.Stop();
		}
		return timer.GetCurrentTotalTime() / iterationCount_ * 1000;
	}
	
	public function GetPath(pathfinder_:IPathfinder, path_:Path, map_:GraphGridMap):PathResult
	{
		var pathParam = new PathplannerParameter();
		pathParam.startNode = path_.start;
		pathParam.startX = path_.start.GetPosition().GetX();
		pathParam.startY = path_.start.GetPosition().GetY();
		pathParam.goalX = path_.end.GetPosition().GetX();
		pathParam.goalY = path_.end.GetPosition().GetY();
		pathParam.goalNode = path_.end;
		var timer = new DebugRunningTimer();
		timer.Start();
		var path:Array<Position>;
		
		//Timer.measure(function()
		//{
			path = pathfinder_.FindPath(pathParam);
		//} );
		timer.Stop();
		
		/*for (i in 0...100)
		{
			trace("_: " + pathfinder_.GetActionOutput().GetActionList()[i].actionType);
		}*/
		
		var pathplannerName = Type.getClassName(Type.getClass(pathfinder_));
		var actionOutput = pathfinder_.GetActionOutput();
		DebugLogger.Assert(path != null, "The pathplanner: " + pathplannerName + " has produced a null path! action count: " + actionOutput.GetActionList().length);
		
		var pathLength:Float = 0;
		for (i in 0...path.length - 1)
		{
			//trace("path: " + i + " __ " + path[i].ToString());
			pathLength += Math.sqrt(Math.pow(path[i].GetX() - path[i + 1].GetX(), 2) + Math.pow(path[i].GetY() - path[i + 1].GetY(), 2));
		}
		
		return {pathplanner:pathfinder_, path: path, timeTaken: (timer.GetCurrentTotalTime() * 1000), actionCount: actionOutput.GetActionList().length, pathLength: pathLength };
	}
	
	public function ComparePath(pathOne_:PathResult, pathTwo_:PathResult, deviance_:Float)
	{
		
		var pathplannerOneName = Type.getClassName(Type.getClass(pathOne_.pathplanner));
		var pathplannerTwoName = Type.getClassName(Type.getClass(pathTwo_.pathplanner));
		
		// percentage difference between the paths taking path one as the default
		var percent:Float = pathOne_.pathLength / pathTwo_.pathLength;
		
		trace("pathone: " + pathplannerOneName + " ________________________ ");
		trace("pathLength:" + pathOne_.pathLength);
		trace("timeTaken: " + pathOne_.timeTaken);
		trace("actionCount: " + pathOne_.actionCount);
		trace("node length: " + pathOne_.path.length);
		
		trace("pathtwo: " + pathplannerTwoName + " ________________________ ");
		trace("pathLength:" + pathTwo_.pathLength);
		trace("timeTaken: " + pathTwo_.timeTaken);
		trace("actionCount: " + pathTwo_.actionCount);
		trace("node length: " + pathTwo_.path.length);
		
		//DebugLogger.Assert(!((percent - 1) < -deviance_ || (percent - 1) > deviance_), "Deviance of " + pathplannerTwoName + " is too large: " + (percent - 1) + " against: +/-" + deviance_);
	}
	
	public function LoadMap(filePath_:String):GraphGridMap
	{
		var fin = File.read(filePath_, false);
		var map:GraphGridMap = null;
		try
		{
			fin.readLine(); //map type
			var height:Int = Std.parseInt(fin.readLine().split(" ")[1]);
			var width:Int = Std.parseInt(fin.readLine().split(" ")[1]);
			fin.readLine(); //not sure what the 4th line is for in the map files
			
			map = new GraphGridMap(width, height);

			for(y in 0...height)
			{
				var str = fin.readLine();
				
				for (x in 0...width)
				{
					var char = str.charAt(x);
					switch(char)
					{
						case '.':
							map.GetNodeByIndex(x, y).SetTraversable(true);
						case 'G':
							map.GetNodeByIndex(x, y).SetTraversable(true);
						case 'S':
							map.GetNodeByIndex(x, y).SetTraversable(true);
							
						case '@':
							map.GetNodeByIndex(x, y).SetTraversable(false);
						case 'O':
							map.GetNodeByIndex(x, y).SetTraversable(false);
						case 'W':
							map.GetNodeByIndex(x, y).SetTraversable(false);
						case 'T':
							map.GetNodeByIndex(x, y).SetTraversable(false);
							
						default:
							throw "something went wrong: " + char;
							
					}
					
				}
			}
		}
		catch( ex:haxe.io.Eof ) 
		{
			throw ex;
		}

		return map;
	}
	
	public function GeneratePaths(map_:GraphGridMap, amount_:Int):Array<Path>
	{
		var returnArray:Array<Path> = new Array<Path>();
		for (i in 0...amount_)
		{
			returnArray[i] = {
				start: GetRandomTraversableNode(map_),
				end: GetRandomTraversableNode(map_),
				optimalLength:0
			}
		}
		
		return returnArray;
	}
	
	public function GetRandomTraversableNode(map_:GraphGridMap):Node
	{
		var node:Node = null;
        while(node == null)
        {
            var temp:Node = map_.GetNodeByIndex(Std.random(map_.GetWidth()), Std.random(map_.GetHeight()));
            if(temp.GetTraversable() == true)
            {
                node = temp;
            }
        }

        return node;
	}
	
	public function LoadScenarios(filePath_:String, map_:GraphGridMap, splitChar_:String):Array<Path>
	{
		var fin = File.read(filePath_, false);
		
		var segmentArray:Array<Path> = new Array<Path>();
		
		try
		{
			fin.readLine(); //version number

			while (true)
			{
				/* each line in the scenario file is split up into segements seperated by a whitespace which represent different things.
				 * [0] Bucket
				 * [1] map file path
				 * [2] map width
				 * [3] map height
				 * [4] start x coord
				 * [5] start y coord
				 * [6] goal x coord
				 * [7] goal y coord
				 * [8] optimal length
				 */
				var segments = fin.readLine().split(splitChar_);
				if (segments.length == 9) // some scenario files seem to have empty lines
				{
					segmentArray.push( {
						start: map_.GetNodeByIndex(Std.parseInt(segments[4]), Std.parseInt(segments[5])),
						end: map_.GetNodeByIndex(Std.parseInt(segments[6]), Std.parseInt(segments[7])),
						optimalLength: Std.parseFloat(segments[8])
					});
				}
			}
		}
		catch( ex:haxe.io.Eof ) 
		{
		}
		
		return segmentArray;
	}
	
	public static function main()
	{
		
		new Main();
		
	}
	
}