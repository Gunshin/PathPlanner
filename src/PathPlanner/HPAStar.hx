package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 */
class HPAStar implements IPathfinder 
{

	var graphHier:GraphHierarchical;
	
	@:protected
	var heuristicFunction:
	#if cs
	cs.system.Func_3<Position,Position,Float>;
	#else
	Position -> Position -> Float;
	#end
	
	#if action_output
	var actionOutput:ActionOutput;
	#end
	
	public function new(heuristicFunction_:
		#if cs
		cs.system.Func_3<Position,Position,Float>
		#else
		Position -> Position -> Float
		#end
		,
		graphHier_:GraphHierarchical) 
	{
		graphHier = graphHier_;
	}
	
	public function FindPath(param_:PathplannerParameter):Array<Position>
	{
		return null;
	}
	
	// the only thing that is used within debug logger atm is assert
	//public function AttachDebugLogger(debugLogger_:DebugLogger):Void;
	
	public function GetActionOutput():ActionOutput
	{
		#if action_output
		return actionOutput;
		#else
		throw "PathPlanner library has not been compiled with action output as needed. Recompile with compiler command -D action_output";
		#end
	}
	
}