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
		
		trace("length = " + map[x][y].get_neighbours().length);
		for (i in 0...map[x][y].get_neighbours().length)
		{
			trace(map[x][y].get_neighbours()[i].connectedNode.get_x() + " _ " + map[x][y].get_neighbours()[i].connectedNode.get_y());
		}
		
		var path:Array<Node> = pathfinder.FindPath(map[0][0], map[15][10]);
		
		for (i in 0...path.length)
		{
			trace(path[i].get_x() + " _ " + path[i].get_y());
		}
		
		while (true)
		{
			
		}
		
	}
	
	function CreateMap(width:Int, height:Int):Void
	{
		
		map = new Array<Array<Node>>();
		
		for (i in 0...height)
		{
			map[i] = new Array<Node>();
			
			for (j in 0...width)
			{
				
				map[i][j] = new Node(j, i, true);
				
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