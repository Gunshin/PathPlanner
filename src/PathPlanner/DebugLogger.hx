package pathPlanner;

#if cs
import cs.Lib;
#end

// secondary node incase SetParent or similar
typedef Action = {actionType:Int, primaryNode:Node, secondaryNode:Node}

/**
 * ...
 * @author ...
 */
@:nativeGen
class DebugLogger 
{
	@:protected
	static var instance:DebugLogger;
	
	public var actionTypeMap:Map<String, Int> = [
		"Expand" => 1,
		"AddToOpen" => 2,
		"AddToClosed" => 3,
		"SetParent" => 4
	];
	
	@:protected
	private var actionList:Array<Action> = new Array<Action>();
	
	#if cs
	var loggingFunction:cs.system.Action_1<String>;
	#else
	var loggingFunction:String -> Void;
	#end
	
	function new()
	{
		
	}
	
	public function GetActionList():Array<Action>
	{
		return actionList;
	}
	
	public function ResetActionList()
	{
		actionList = new Array<Action>();
	}
	
	public function Expand(node:Node)
	{
		actionList.push({actionType:1, primaryNode:node, secondaryNode:null});
	}
	
	public function AddToOpen(node:Node)
	{
		actionList.push({actionType:2, primaryNode:node, secondaryNode:null});
	}
	
	public function AddToClosed(node:Node)
	{
		actionList.push({actionType:3, primaryNode:node, secondaryNode:null});
	}
	
	public function SetParent(node:Node, parent:Node)
	{
		actionList.push({actionType:4, primaryNode:node, secondaryNode:parent});
	}
	
	public function Print(message_:String)
	{
		if(loggingFunction != null)
		#if cs
		loggingFunction.Invoke(message_);
		#else
		loggingFunction(message_);
		#end
	}
	
	public static function Assert(flag_:Bool, message_:String)
	{
		if (flag_)
		{
			instance.Print(message_);
			throw message_;
		}
	}
	
	public static function GetInstance():DebugLogger
	{
		if (instance == null)
		{
			instance = new DebugLogger();
		}
		
		return instance;
	}
	
	#if cs
	public function GetLoggingFunction():cs.system.Action_1<String>
	#else
	public function GetLoggingFunction():String -> Void
	#end
	{
		return loggingFunction;
	}
	
	#if cs
	public function SetLoggingFunction(loggingFunc_:cs.system.Action_1<String>):cs.system.Action_1<String>
	#else
	public function SetLoggingFunction(loggingFunc_:String -> Void):String -> Void
	#end
	{
		return loggingFunction = loggingFunc_;
	}
	
}