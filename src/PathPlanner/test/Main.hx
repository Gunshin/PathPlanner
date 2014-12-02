package pathPlanner.test;

import haxe.io.Eof;
import pathPlanner.IPathfinder;
import pathPlanner.test.Main.PathResult;
import sys.io.File;
import sys.io.FileOutput;

import pathPlanner.Map;

typedef PathResult = { pathplanner:IPathfinder, pathLength: Float, path: Array<Node> }

/**
 * ...
 * @author Michael Stephens
 */
class Main 
{

	var map:Array<Array<Node>>;
	
	#if debug
	function Print(message_:String)
	{
		trace(message_);
	}
	#end
	
	public function new()
	{
		#if debugging
		//DebugLogger.instance.loggingFunction = Print;
		#end
		
		var pathfinder:IPathfinder = new AStar();
		var jps:IPathfinder = new JPS();
		
		var map = LoadMap("resources/maps/battleground.map");
		var segments = LoadScenarios("resources/scenarios/battleground.map.scen");
		
		var i = 0;
		for (segment in segments)
		{
			trace("looking through: " + i++);
			ComparePath( GetPath(pathfinder, segment, map), GetPath(jps, segment, map) , 0.1);
		}
	}
	
	public function PrintPath(pathStruct_:PathResult)
	{
		var path:Array<Node> = pathStruct_.path;
		trace("path length: " + path.length);
		for (i in 0...path.length)
		{
			trace(path[i].x + " _ " + path[i].y);
		}
		
		trace("path length of: " + pathStruct_.pathLength);
	}
	
	public function GetPath(pathfinder_:IPathfinder, scenario_, map_:Map):PathResult
	{
		var sNode = map_.GetNodeByIndex(scenario_.sx, scenario_.sy);
		var eNode = map_.GetNodeByIndex(scenario_.gx, scenario_.gy);
		
		var path:Array<Node> = pathfinder_.FindPath(sNode, eNode, 
		function(nodeOne, nodeTwo)
		{
			return Math.sqrt(Math.pow(nodeOne.x - nodeTwo.x, 2) + Math.pow(nodeOne.y - nodeTwo.y, 2));
		}
		);
		
		var pathplannerName = Type.getClassName(Type.getClass(pathfinder_));
		DebugLogger.Assert(path == null, "The pathplanner: " + pathplannerName + " has produced a null path!");
		
		var pathLength:Float = 0;
		for (node in path)
		{
			if(node.parent != null)
			pathLength += Math.sqrt(Math.pow(node.x - node.parent.x, 2) + Math.pow(node.y - node.parent.y, 2));
		}
		
		return {pathplanner:pathfinder_, path: path, pathLength: pathLength };
	}
	
	public function ComparePath(pathOne_:PathResult, pathTwo_:PathResult, deviance_:Float)
	{
		
		var pathplannerOneName = Type.getClassName(Type.getClass(pathOne_.pathplanner));
		var pathplannerTwoName = Type.getClassName(Type.getClass(pathTwo_.pathplanner));
		
		// percentage difference between the paths taking path one as the default
		var percent:Float = pathOne_.pathLength / pathTwo_.pathLength;
		DebugLogger.Assert((percent - 1) < -deviance_ || (percent - 1) > deviance_, "Deviance of " + pathplannerTwoName + " is too large: " + (percent - 1) + " against: +/-" + deviance_);
		
		trace("pathone: " + pathplannerOneName + ": pathLength:" + pathOne_.pathLength + " node length: " + pathOne_.path.length +
		" pathtwo: " + pathplannerTwoName + ": pathLength:" + pathTwo_.pathLength + " node length: " + pathTwo_.path.length);
		
	}
	
	public function LoadMap(filePath_:String):Map
	{
		var fin = File.read(filePath_, false);
		var map:Map = null;
		try
		{
			fin.readLine(); //map type
			var height:Int = Std.parseInt(fin.readLine().split(" ")[1]);
			var width:Int = Std.parseInt(fin.readLine().split(" ")[1]);
			fin.readLine(); //not sure what the 4th line is for in the map files
			
			map = new Map(width, height, 1, 1);

			for(y in 0...height)
			{
				var str = fin.readLine();
				
				for (x in 0...width)
				{
					var char = str.charAt(x);
					switch(char)
					{
						case '.':
							map.GetNodeByIndex(x, y).traversable = true;
						case 'G':
							map.GetNodeByIndex(x, y).traversable = true;
						case 'S':
							map.GetNodeByIndex(x, y).traversable = true;
							
						case '@':
							map.GetNodeByIndex(x, y).traversable = false;
						case 'O':
							map.GetNodeByIndex(x, y).traversable = false;
						case 'W':
							map.GetNodeByIndex(x, y).traversable = false;
						case 'T':
							map.GetNodeByIndex(x, y).traversable = false;
							
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
	
	public function LoadScenarios(filePath_:String)
	{
		var fin = File.read(filePath_, false);
		
		var segmentArray = [];
		
		try
		{
			fin.readLine(); //version number

			while (true)
			{
				/* each line in the scenario file is split up into segements seperated by a whitespace which represent different things.
				 * [0] Bucket (not sure)
				 * [1] map file path
				 * [2] map width
				 * [3] map height
				 * [4] start x coord
				 * [5] start y coord
				 * [6] goal x coord
				 * [7] goal y coord
				 * [8] optimal length
				 */
				var segments = fin.readLine().split(" ");
				segmentArray.push({
					filePath: segments[1],
					width: Std.parseInt(segments[2]),
					height: Std.parseInt(segments[3]),
					sx: Std.parseInt(segments[4]),
					sy: Std.parseInt(segments[5]),
					gx: Std.parseInt(segments[6]),
					gy: Std.parseInt(segments[7]),
					optimalLength: Std.parseFloat(segments[8])
				});
				
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