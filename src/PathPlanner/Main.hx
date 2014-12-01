package pathPlanner;

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
		DebugLogger.instance.loggingFunction = Print;
		#end
		
		var pathfinder:IPathfinder = new AStar();
		var jps:IPathfinder = new JPS();
		
		trace("init pathplanners");
		
		//CreateMap(32, 32);
		var map:Map = new Map(16, 16, 1, 1);
		
		var node:Node = map.GetNodeByIndex(3, 3);
		node.traversable = false;
		
		var sNode:Node = map.GetNodeByIndex(0, 0);
		var eNode:Node = map.GetNodeByIndex(14, 9);
		
		trace("createdMap");
		
		var x:Int = 1;
		var y:Int = 1;
		
		var path:Array<Node> = pathfinder.FindPath(sNode, eNode, 
		function(nodeOne, nodeTwo)
		{
			return Math.sqrt(Math.pow(nodeOne.x - nodeTwo.x, 2) + Math.pow(nodeOne.y - nodeTwo.y, 2));
		}
		);
		
		trace("A*");
		for (i in 0...path.length)
		{
			trace(path[i].x + " _ " + path[i].y);
		}
		
		trace("A* completed");
		
		var jpsPath:Array<Node> = jps.FindPath(sNode, eNode, 
		function(nodeOne, nodeTwo)
		{
			return Math.sqrt(Math.pow(nodeOne.x - nodeTwo.x, 2) + Math.pow(nodeOne.y - nodeTwo.y, 2));
		}
		);
		
		trace("JPS " + (jpsPath != null));
		for (i in 0...jpsPath.length)
		{
			trace(jpsPath[i].x + " _ " + jpsPath[i].y);
		}
		
		while (true) {}
	}
	
	function CreateMap(width:Int, height:Int):Void
	{
		
		map = new Array<Array<Node>>();
		
		for (i in 0...height)
		{
			map[i] = new Array<Node>();
			
			for (j in 0...width)
			{
				
				map[i][j] = new Node(j, i, true, new GraphStructureIndirect());
				
			}
		}
		
		for (i in 0...height)
		{
			for (j in 0...width)
			{
				
				for (a in -1...2)
				{
					for (b in -1...2)
					{
						
						if (!(a == 0 && b == 0))
						{
							var neighbourX:Int = j + b;
							var neighbourY:Int = i + a;
							
							if(neighbourX >= 0 && neighbourX < width && neighbourY >= 0 && neighbourY < height)
								map[i][j].AddNeighbour(map[neighbourY][neighbourX]);
						}
					}
				}
				
			}
		}
		
	}
	
	public static function main()
	{
		
		new Main();
		
	}
	
}