package pathPlanner;

#if cs
import cs.Lib;
#end

/**
 * ...
 * @author Michael Stephens
 */
interface IPathfinder 
{

	#if cs
	public function FindPath(startNode_:Node, endNode_:Node, heuristicFunction_:cs.system.Func_3<Node,Node,Float>):Array<Node>;
	#else
	public function FindPath(startNode_:Node, endNode_:Node, heuristicFunction_: Node -> Node -> Float):Array<Node>;
	#end
}