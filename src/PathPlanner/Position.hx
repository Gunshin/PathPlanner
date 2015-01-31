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
	
	public static inline function Equal(positionA_:Position, positionB_:Position):Bool
	{
		return positionA_.GetX() == positionB_.GetX() && positionA_.GetY() == positionB_.GetY();
	}
	
	public function ToString():String
	{
		return GetX() + " _ " + GetY();
	}
	
	public static inline function Distance(posA_:Position, posB_:Position):Float
	{
		return Math.sqrt(Math.pow(posA_.GetX() - posB_.GetX(), 2) + Math.pow(posA_.GetY() - posB_.GetY(), 2));
	}
	
}