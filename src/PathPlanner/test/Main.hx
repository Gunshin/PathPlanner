package pathPlanner.test;

import haxe.io.Eof;
import haxe.io.Path;
import haxe.Timer;
import pathPlanner.DebugRunningTimer;
import pathPlanner.GraphGridMapMinimalist;
import pathPlanner.PathplannerParameter;

import pathPlanner.IPathfinder;
import pathPlanner.Node;
import pathPlanner.test.Main.PathResult;
import sys.io.File;
import sys.io.FileOutput;

import pathPlanner.GraphGridMap;

typedef Path = { start:Node, end:Node, optimalLength:Float }
typedef PathResult = { pathplanner:IPathfinder, pathLength: Float, timeTaken: Float, actionCount: Int, path: Array<Node> }

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
		/*var num:haxe.Int32 = 61624;
		var s = Timer.stamp();
		var r = PathUtility.CTZ(num);
		//trace(StringTools.hex(num));
		trace((Timer.stamp() - s) * 1000000 + " _ " + r);
		trace((num >> 2) & 1);*/
		
		/*var graph:GraphGridMapMinimalist = new GraphGridMapMinimalist(64, 64, false);
		graph.SetTraversableTrue(32, 0);
		trace(graph.GetTraversable(32, 0));
		
		var rotated:GraphGridMapMinimalist = graph.RotateMap();
		trace(rotated.GetTraversable(0, 32));*/
		
		#if debugging
		//DebugLogger.GetInstance().SetLoggingFunction(Print);
		#end
		
		var map = LoadMap("resources/DragonAgeMaps/arena2.map");
		var paths = LoadScenarios("resources/DragonAgeScenarios/arena2.map.scen", map, "	");
		
		var pathfinder:IPathfinder = new AStar();
		var jps:IPathfinder = new JPS(map);
		
		var path:Int = 0;
		
		//var paths = GeneratePaths(map, 50);
		/*var timerJPS = new DebugRunningTimer();
		for (i in 0...10000)
		{
			trace("starting JPS: " + i);
			timerJPS.Start();
			GetPath(jps, paths[path], map);
			timerJPS.Stop();
		}
		
		
		var timerAStar = new DebugRunningTimer();
		for (i in 0...10000)
		{
			trace("starting AStar: " + i);
			timerAStar.Start();
			GetPath(pathfinder, paths[path], map);
			timerAStar.Stop();
		}
		
		trace("timerJPS took: " + (timerJPS.GetCurrentTotalTime() / 10000) + " timerAStar took: " + (timerAStar.GetCurrentTotalTime() / 10000));*/
		//trace("looking through: " + " _ " + paths[path].start.GetX() + "," + paths[path].start.GetY() + " t: " + paths[path].start.GetTraversable() + " _ " + paths[path].end.GetX() + "," + paths[path].end.GetY() + " t: " + paths[path].end.GetTraversable());
		//ComparePath( GetPath(pathfinder, paths[path], map), GetPath(jps, paths[path], map) , 0.1);
		//ComparePath( GetPath(pathfinder, paths[400], map), GetPath(jps, paths[400], map) , 0.1);
		//ComparePath( GetPath(pathfinder, paths[400], map), GetPath(jps, paths[400], map) , 0.1);
		//ComparePath( GetPath(pathfinder, paths[400], map), GetPath(jps, paths[400], map) , 0.1);
		//trace(DebugLogger.GetInstance().GetActionList().length);
		
		var i = 0;
		for (path in paths)
		{
			trace("looking through: " + i++ + " _ " + path.start.GetX() + "," + path.start.GetY() + " t: " + path.start.GetTraversable() + " _ " + path.end.GetX() + "," + path.end.GetY() + " t: " + path.end.GetTraversable());
			ComparePath( GetPath(jps, path, map), GetPath(pathfinder, path, map) , 0.4);
			//GetPath(jps, path, map); // currently using GetPath on just the A* algorithm to determine whether a scenario is viable
		}
	}
	
	public function PrintPath(pathStruct_:PathResult)
	{
		var path:Array<Node> = pathStruct_.path;
		trace("path length: " + path.length);
		for (i in 0...path.length)
		{
			trace(path[i].GetX() + " _ " + path[i].GetY());
		}
		
		trace("path length of: " + pathStruct_.pathLength);
	}
	
	public function GetPath(pathfinder_:IPathfinder, path_:Path, map_:GraphGridMap):PathResult
	{
		var pathParam = new PathplannerParameter();
		pathParam.startNode = path_.start;
		pathParam.goalNode = path_.end;
		var timer = new DebugRunningTimer();
		timer.Start();
		var path:Array<Node>;
		
		//Timer.measure(function()
		//{
			path = pathfinder_.FindPath(pathParam, 
			function(nodeOne, nodeTwo)
			{
				return Math.sqrt(Math.pow(nodeOne.GetX() - nodeTwo.GetX(), 2) + Math.pow(nodeOne.GetY() - nodeTwo.GetY(), 2));
			}
			);
		//} );
		timer.Stop();
		
		var pathplannerName = Type.getClassName(Type.getClass(pathfinder_));
		DebugLogger.Assert(path == null, "The pathplanner: " + pathplannerName + " has produced a null path! action count: " + DebugLogger.GetInstance().GetActionList().length);
		
		var pathLength:Float = 0;
		for (node in path)
		{
			if(node.GetParent() != null)
			pathLength += Math.sqrt(Math.pow(node.GetX() - node.GetParent().GetX(), 2) + Math.pow(node.GetY() - node.GetParent().GetY(), 2));
		}
		
		return {pathplanner:pathfinder_, path: path, timeTaken: (timer.GetCurrentTotalTime() * 1000000), actionCount: DebugLogger.GetInstance().GetActionList().length, pathLength: pathLength };
	}
	
	public function ComparePath(pathOne_:PathResult, pathTwo_:PathResult, deviance_:Float)
	{
		
		var pathplannerOneName = Type.getClassName(Type.getClass(pathOne_.pathplanner));
		var pathplannerTwoName = Type.getClassName(Type.getClass(pathTwo_.pathplanner));
		
		// percentage difference between the paths taking path one as the default
		var percent:Float = pathOne_.pathLength / pathTwo_.pathLength;
		DebugLogger.Assert((percent - 1) < -deviance_ || (percent - 1) > deviance_, "Deviance of " + pathplannerTwoName + " is too large: " + (percent - 1) + " against: +/-" + deviance_);
		
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
		
		//DebugLogger.Assert(pathOne_.timeTaken > pathTwo_.timeTaken, "found");
		
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
			
			map = new GraphGridMap(width, height, 1, 1);

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