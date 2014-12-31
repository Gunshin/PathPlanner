package pathPlanner;
import pathPlanner.DebugLogger.Action;

#if cs
import cs.Lib;
#end

// secondary node incase SetParent or similar
class Action
{
	var actionType:Int;
	var primaryNode:Node;
	var secondaryNode:Node;
	
	public function new(actionType_:Int, primaryNode_:Node, secondaryNode_:Node)
	{
		actionType = actionType_;
		primaryNode = primaryNode_;
		secondaryNode = secondaryNode_;
	}
}

/**
 * ...
 * @author ...
 */
class DebugLogger 
{
	@:protected
	static var instance:DebugLogger;
	
	public var actionTypes:Array<String> = [
	"Expand",
	"AddToOpen",
	"AddToClose",
	"SetParent"
	];
	
	@:protected
	var actionTypeMap:Map<String, Int> = [
		"Expand" => 1,
		"AddToOpen" => 2,
		"AddToClosed" => 3,
		"SetParent" => 4
	];
	
	@:protected
	var actionList:Array<Action> = new Array<Action>();
	
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
		actionList.push(new Action(1, node, null));
	}
	
	public function AddToOpen(node:Node)
	{
		actionList.push(new Action(2, node, null));
	}
	
	public function AddToClosed(node:Node)
	{
		actionList.push(new Action(3, node, null));
	}
	
	public function SetParent(node:Node, parent:Node)
	{
		actionList.push(new Action(4, node, parent));
	}
	
	public function GetActionKeysValue(actionKey:String)
	{
		return actionTypeMap.get(actionKey);
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