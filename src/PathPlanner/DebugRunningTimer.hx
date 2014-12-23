package pathPlanner;

import haxe.Timer;

/**
 * ...
 * @author ...
 */
class DebugRunningTimer 
{
	
	var currentTotalTime:Float = 0;
	
	var start:Float = 0;
	
	public function new() 
	{
	}
	
	public inline function Start():Void
	{
		start = Timer.stamp();
	}
	
	public inline function Stop():Float
	{
		var timeTaken:Float = Timer.stamp() - start;
		currentTotalTime += timeTaken;
		return timeTaken;
	}
	
	public inline function GetCurrentTotalTime():Float
	{
		return currentTotalTime;
	}
	
	public inline function Reset():Void
	{
		currentTotalTime = 0;
	}
	
}