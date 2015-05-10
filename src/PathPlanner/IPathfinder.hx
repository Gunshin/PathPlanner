package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 */
interface IPathfinder 
{
	public function FindPath(param_:PathplannerParameter):Array<Position>;
	
	// the only thing that is used within debug logger atm is assert
	//public function AttachDebugLogger(debugLogger_:DebugLogger):Void;
	
	public function GetActionOutput():ActionOutput;
}