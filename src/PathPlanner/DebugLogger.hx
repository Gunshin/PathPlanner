package pathPlanner;

#if cs
import cs.Lib;
#end

/**
 * ...
 * @author ...
 * 
 * 
 */
class DebugLogger
{
	/*#if cs
	var loggingFunction:cs.system.Action_1<String>;
	#else
	var loggingFunction:String -> Void;
	#end*/
	
	function new()
	{
	}
	
	/*public function Print(message_:String)
	{
		if(loggingFunction != null)
		#if cs
		loggingFunction.Invoke(message_);
		#else
		loggingFunction(message_);
		#end
	}*/
	
	public static inline function Assert(flag_:Bool, message_:String)
	{
		#if debugging
		if (!flag_)
		{
			throw message_;
		}
		#end
	}
	
	/*#if cs
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
	}*/
	
}