package pathPlanner;

#if cs
import cs.Lib;
#end

/**
 * ...
 * @author ...
 */
class DebugLogger 
{
	static var instance:DebugLogger;
	
	public var expandedSet:Array<Node> = new Array<Node>();
	public var openSet:Array<Node> = new Array<Node>();
	public var closedSet:Array<Node> = new Array<Node>();
	
	#if cs
	var loggingFunction:cs.system.Action_1<String>;
	#else
	var loggingFunction:String -> Void;
	#end
	
	function new()
	{
		
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