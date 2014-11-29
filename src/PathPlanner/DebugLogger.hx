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
		#if cs
		loggingFunction.Invoke(message_);
		#else
		loggingFunction(message_);
		#end
	}
	
	public static function get_instance():DebugLogger
	{
		if (instance == null)
		{
			instance = new DebugLogger();
		}
		
		trace("ran get_instance()");
		
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