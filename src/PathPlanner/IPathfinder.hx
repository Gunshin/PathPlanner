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
	public function FindPath(param_:PathplannerParameter, heuristicFunction_:cs.system.Func_3<Node,Node,Float>):Array<Node>;
	#else
	public function FindPath(param_:PathplannerParameter, heuristicFunction_: Node -> Node -> Float):Array<Node>;
	#end
}