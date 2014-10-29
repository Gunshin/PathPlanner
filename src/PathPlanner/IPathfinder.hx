package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 */
interface IPathfinder 
{

	public function FindPath(startNode_:Node, endNode_:Node, heuristicFunction: Node -> Node -> Float):Array<Node>;
	
}