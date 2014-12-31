package pathPlanner;
import haxe.Int32;
import haxe.io.BytesData.Unsigned_char__;

/**
 * ...
 * @author ...
 */
class GraphGridMapMinimalist
{
	
	@:protected
	var width:Int = 0;
	@:protected
	var height:Int = 0;
	
	// these are for the width divided by the sizeof int (assumed 32bit for now)
	@:protected
	var correctedWidth:Int = 0;
	
	@:protected
	var map:Array<Int32>;

	public function new(width_:Int, height_:Int, traversableDefault_:Bool) 
	{
		width = width_;
		height = height_;
		
		// width may be of non 32 multiplier size
		correctedWidth = Math.ceil(width / 32);
		
		map = new Array<Int>();
		
		SetMap(traversableDefault_);
	}
	
	public function SetMap(traversableDefault_:Bool)
	{
		var gridValue:Int32 = traversableDefault_ ? ~0 : 0;// all bits set as 1 as assumed traversable
		for (j in 0...height)
		{
			for (i in 0...correctedWidth)
			{
				map[i + j * height] = gridValue;
			}
		}
	}
	
	public function RotateMap():GraphGridMapMinimalist
	{
		
		var returnee:GraphGridMapMinimalist = new GraphGridMapMinimalist(width, height, false);
		
		for (i in 0...height)
		{
			for (j in 0...width)
			{
				if (GetTraversable(j, i) == true)
				{
					returnee.SetTraversableTrue(i, j);
				}
				else
				{
					returnee.SetTraversableFalse(i, j);
				}
			}
		}
		
		return returnee;
	}
	
	/*public function GetDirectNeighbours(x_:Int, y_:Int):Unsigned_char__
	{
		if (x_ < 0 || x_ >= width || y_ < 0 || y_ >= height)
		{
			return null;
		}
		
		var returnee:Unsigned_char__ = 0;
		
		returnee |= GetTraversable(x_ - 1, y_ + 1) ? 1 << 7 : 0;
		returnee |= GetTraversable(x_, y_ + 1) ? 1 << 6 : 0;
		returnee |= GetTraversable(x_ + 1, y_ + 1) ? 1 << 5 : 0;
		returnee |= GetTraversable(x_ + 1, y_) ? 1 << 4 : 0;
		returnee |= GetTraversable(x_ + 1, y_ - 1) ? 1 << 3 : 0;
		returnee |= GetTraversable(x_, y_ - 1) ? 1 << 2 : 0;
		returnee |= GetTraversable(x_ - 1, y_ - 1) ? 1 << 1 : 0;
		returnee |= GetTraversable(x_ - 1, y_) ? 1 : 0;
		
		return neighbours;
	}*/
	
	public inline function GetTraversable(x_:Int, y_:Int):Bool
	{
		
		#if debugging
		if (x_ < 0 || x_ >= width || y_ < 0 || y_ >= height)
		{
			throw "operation out of bounds: x: " + x_ + " y: " + y_ + " mapWidth: " + width + " mapHeight: " + height;
		}
		#end
		
		/*var indexX:Int = Std.int(x_ / 32);
		var indexY:Int = y_;
		var bitShift:Int = 32 - x_ % 32;
		
		return ((map[indexX + (indexY * height)] >> bitShift) & 1) == 1;*/
		return ((map[Std.int(x_ / 32) + (y_ * height)] >> 32 - x_ % 32) & 1) == 1;
	}
	
	/*
	 * SetTraversable is seperated to remove branching within the method so to make it as potentially fast as possible
	 */
	public inline function SetTraversableTrue(x_:Int, y_:Int):Void
	{
		
		#if debugging
		if (x_ < 0 || x_ >= width || y_ < 0 || y_ >= height)
		{
			throw "operation out of bounds: x: " + x_ + " y: " + y_ + " mapWidth: " + width + " mapHeight: " + height;
		}
		#end
		
		/*var indexX:Int = Std.int(x_ / 32);
		var indexY:Int = y_;
		var bitShift:Int = 32 - x_ % 32;
		
		trace(indexX + " _ " + bitShift + " _ " + map[indexX + (indexY * height)]);
		
		map[indexX + (indexY * height)] |= 1 << bitShift;*/
		
		map[Std.int(x_ / 32) + (y_ * height)] |= 1 << (32 - (x_ % 32));
		
	}
	
	/*
	 * SetTraversable is seperated to remove branching within the method so to make it as potentially fast as possible
	 */
	public inline function SetTraversableFalse(x_:Int, y_:Int):Void
	{
		
		#if debugging
		if (x_ < 0 || x_ >= width || y_ < 0 || y_ >= height)
		{
			throw "operation out of bounds: x: " + x_ + " y: " + y_ + " mapWidth: " + width + " mapHeight: " + height;
		}
		#end
		
		/*var indexX:Int = Std.int(x_ / 32);
		var indexY:Int = y_;
		var bitShift:Int = 32 - x_ % 32;
		
		trace(indexX + " _ " + bitShift + " _ " + map[indexX + (indexY * height)]);
		
		map[indexX + (indexY * height)] &= ~(1 << bitShift);*/
		
		map[Std.int(x_ / 32) + (y_ * height)] &= ~(1 << (32 - (x_ % 32)));
		
	}
	
	
	
	public inline function GetWidth():Int
	{
		return width;
	}
	
	public inline function GetHeight():Int
	{
		return height;
	}
	
	
	
}