package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 */
class Main 
{

	var map:Array<Array<Node>>;
	
	public function new() 
	{
		
		var pathfinder:IPathfinder = new AStar();
		
		CreateMap(32, 32);
		
		var x:Int = 1;
		var y:Int = 1;
		
		var path:Array<Node> = pathfinder.FindPath(map[0][0], map[14][9], 
		function(nodeOne, nodeTwo)
		{
			/*
			 * very slightly overestimate so that we get the most direct looking path.
			 * due to certain paths giving the exact same cost, the nodes which are added first are selected first resulting in a slightly odd looking path.
			 * 
			 * Need to visualise it fully, still looks problematic
			 */
			return 1.01 * Math.sqrt(Math.pow(nodeOne.x - nodeTwo.x, 2) + Math.pow(nodeOne.y - nodeTwo.y, 2));
		}
		);
		
		for (i in 0...path.length)
		{
			trace(path[i].x + " _ " + path[i].y);
		}
		
		var map:Map = new Map(10, 10);
		var node:Node = map.GetNodeByIndex(0, 0);
		
		node.AddNeighbour(map.GetNodeByIndex(0, 9));
		
		node.RemoveNeighbour(map.GetNodeByIndex(0, 9));
		
		while (true){}
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