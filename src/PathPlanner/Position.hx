package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 * 
 * This is just a utility class for storing a position (duh). Was not originally needed, but kinda made sense to use
 * it since more of my pathplanners started to require this data instead of a full node (looking at you heuristic function)
 * 
 */
class Position
{
	@:protected
	var x:Int;
	@:protected
	var y:Int;
	
	public function new(x_:Int, y_:Int)
	{
		x = x_;
		y = y_;
	}
	
	public function GetX():Int
	{
		return x;
	}
	
	public function GetY():Int
	{
		return y;
	}
	
	public function ToString()
	{
		return GetX() + " _ " + GetY();
	}
	
}