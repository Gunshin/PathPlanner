package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 * 
 * This class is 'needed' because not all pathplanners rely on the same data structures. Some such as JPS
 * can take either integer indexs or nodes to determine a path from one place to another, but the more 
 * graph independent algorithms such as A* require nodes to start from since they could be using 
 * GraphStructureIndirect or similar
 * 
 */
class PathplannerParameter
{

	public var startX:Int = -1;
	public var startY:Int = -1;
	public var goalX:Int = -1;
	public var goalY:Int = -1;
	public var startNode:Node = null;
	public var goalNode:Node = null;
	
	public function new() 
	{
		
	}
	
}