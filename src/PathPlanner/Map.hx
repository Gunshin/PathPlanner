package pathPlanner;

import de.polygonal.ds.HashTable;

/**
 * ...
 * @author Michael Stephens
 * 
 * This graph struture is for assuming uniform rectangular/grid based maps. I needed a distinction between
 * GraphStrutureDirect and GraphStructureIndirect because i originally planned for all the algorithms to be
 * based on using indirect graphs, and subsequently built the A* for that. However, for me to implement 
 * techniques such as JPS for A*, i need a direct graph structure. Therefor, to help keep the algorithms
 * graph independent, i created these.
 * 
 * This class stores a rectangular grid map.
 * Additional neighbours are added to nodes through a hashmap, with each node being a key.
 */
class Map implements IGraphStructure
{
	public var width(get, null):Int = 0;
	public var height(get, null):Int = 0;
	
	var map:Array<Array<Node>>;
	
	var neighbourHashTable:HashTable < Node, Array<DistanceNode> > ;
	
	public function new(width_:Int, height_:Int) 
	{
		width = width_;
		height = height_;
		
		neighbourHashTable = new HashTable < Node, Array<DistanceNode> > (4, 32);
		
		map = new Array<Array<Node>>();
		
		for (i in 0...width)
		{
			map[i] = new Array<Node>();
			for (j in 0...height)
			{
				map[i][j] = new Node(i, j, true, new GraphStructureIndirect());
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
							var neighbourX:Int = i + a;
							var neighbourY:Int = j + b;
							
							if(neighbourX >= 0 && neighbourX < width && neighbourY >= 0 && neighbourY < height)
								map[i][j].AddNeighbour(map[neighbourX][neighbourY]);
						}
					}
				}
				
			}
		}
	}
	
	public function AddNeighbour(node_:Node, neighbour_:Node, ?distance_:Float):Void
	{
		var neighbourDistance:DistanceNode;
		
		if (distance_ != null)
		{
			neighbourDistance = new DistanceNode(neighbour_, node_, distance_);
		}
		else
		{
			neighbourDistance = new DistanceNode(neighbour_, node_);
		}
		
		var neighbours:Array<DistanceNode> = neighbourHashTable.get(node_);
		
		if (neighbours != null)
		{
			neighbours.push(neighbourDistance);
		}
		else
		{
			neighbours = new Array<DistanceNode>();
			neighbours.push(neighbourDistance);
			neighbourHashTable.set(node_, neighbours);
		}
	}
	
	public function RemoveNeighbour(node_:Node, neighbour_:Node):Bool
	{
		var neighbours:Array<DistanceNode> = neighbourHashTable.get(node_);
		
		var indexOfNeighbour = PathUtility.Contains(neighbours, neighbour_);
		var success:Bool = false;
		
		if (indexOfNeighbour >= 0)
		{
			neighbours.remove(neighbours[indexOfNeighbour]);
			success = true;
		}
		
		if (neighbours.length == 0)
		{
			neighbourHashTable.remove(neighbours);
		}
		
		return success;
	}
	
	/*
	 * Returns an array with the format 0 = topLeft, 1 = top, 2 = topRight, 3 = midRight etc etc in a clockwise direction
	 * These elements can be individually null depending on if the node is on the edge of the map
	 * 
	 * Returns null if the node does not exist in the map
	 */
	public function GetDirectNeighbours(node_:Node):Array<DistanceNode>
	{
		var x:Int = cast(node_.x, Int);
		var y:Int = cast(node_.y, Int);
		
		var neighbours:Array<DistanceNode> = null;
		
		// make sure that node is a part of the map
		if (node_ == map[x][y])
		{
			neighbours = new Array<DistanceNode>();
			// do it backwards since i can only assume that the arrays resize based on illegal/overflow access
			// i have the feeling that whoever made the Array resize, does not resize by +1 each time, but im not going to take the risk.
			neighbours[7] = x > 0 ? 							new DistanceNode(map[x - 1][y], node_) : null; 	// mid left
			neighbours[6] = x > 0 && y > 0 ? 					new DistanceNode(map[x - 1][y - 1], node_) : null; // bottom left
			neighbours[5] = y > 0 ? 							new DistanceNode(map[x][y - 1], node_) : null; 	// bottom mid
			neighbours[4] = x < width - 1 && y > 0 ? 			new DistanceNode(map[x + 1][y - 1], node_) : null; // bottom right
			neighbours[3] = x < width - 1 ? 					new DistanceNode(map[x + 1][y], node_) : null; 	// mid right
			neighbours[2] = x < width - 1 && y < height - 1 ? 	new DistanceNode(map[x + 1][y + 1], node_) : null; // top right
			neighbours[1] = y < height - 1 ? 					new DistanceNode(map[x][y + 1], node_) : null; 	// top mid
			neighbours[0] = x > 0 && y < height - 1 ? 			new DistanceNode(map[x - 1][y + 1], node_) : null; // top left
		}
		
		return neighbours;
	}
	
	/*
	 * Returns all neighbours who are NOT directly connected to the node from the point of the map
	 * 
	 * Returns null if the node has no indirect neighbours
	 */
	public function GetIndirectNeighbours(node_:Node):Array<DistanceNode>
	{
		return neighbourHashTable.get(node_);
	}
	
	/*
	 * Returns all neighbours associated with this node in this map. If it exists in the map, the first 8 slots
	 * will be the direct neighbours, and the remainder will be indirect neighbours.
	 * 
	 * Returns null if node does not exist in map
	 */
	public function GetNeighbours(node_:Node):Array<DistanceNode>
	{
		var directNeighbours:Array<DistanceNode> = GetDirectNeighbours(node_);
		var indirectNeighbours:Array<DistanceNode> = GetIndirectNeighbours(node_);
		if (directNeighbours != null)
		{
			return directNeighbours.concat(indirectNeighbours);
		}
		
		return null;
	}
	
	public function GetNodeByIndex(x_:Int, y_:Int):Node
	{
		return x_ >= 0 && y_ >= 0 && x_ < width && y_ < height ? map[x_][y_] : null;
	}
	
	/*
	 * Only in use temporarily as i need to split the public interface for nodes from the pathplanner specific needs.
	 */
	public function ResetForPathplanning():Void
	{
		for (i in 0...width)
		{
			for (j in 0...height)
			{
				map[i][j].parent = null;
				map[i][j].pathCost = 0;
				map[i][j].heuristic = 0;
			}
		}
	}
	
	function get_width():Int
	{
		return width;
	}
	
	function get_height():Int
	{
		return height;
	}
	
}