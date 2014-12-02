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
	public static var instance(get, null):DebugLogger;
	
	public var expandedSet:Array<Node> = new Array<Node>();
	public var openSet:Array<Node> = new Array<Node>();
	public var closedSet:Array<Node> = new Array<Node>();
	
	#if cs
	@:isVar public var loggingFunction(get, set):cs.system.Action_1<String>;
	#else
	@:isVar public var loggingFunction(get, set):String -> Void;
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
	
	public static function get_instance():DebugLogger
	{
		if (instance == null)
		{
			instance = new DebugLogger();
		}
		
		return instance;
	}
	
	#if cs
	public function get_loggingFunction():cs.system.Action_1<String>
	#else
	public function get_loggingFunction():String -> Void
	#end
	{
		return loggingFunction;
	}
	
	#if cs
	public function set_loggingFunction(loggingFunc_:cs.system.Action_1<String>):cs.system.Action_1<String>
	#else
	public function set_loggingFunction(loggingFunc_:String -> Void):String -> Void
	#end
	{
		return loggingFunction = loggingFunc_;
	}
	
}