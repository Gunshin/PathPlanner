package pathPlanner;
import haxe.ds.Vector;
import haxe.Int32;

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
	var map:Vector<Int32>;

	public function new(width_:Int, height_:Int, traversableDefault_:Bool) 
	{
		width = width_;
		height = height_;
		
		// width may be of non 32 multiplier size
		correctedWidth = Math.ceil(width / 32);
		
		map = new Vector<Int>(correctedWidth * height);
		
		SetMap(traversableDefault_);
	}
	
	public function SetMap(traversableDefault_:Bool)
	{
		var gridValue:Int32 = traversableDefault_ ? 0 : ~0;// all bits set as 0 are assumed traversable
		for (j in 0...(correctedWidth * height))
		{
			map[j] = gridValue;
			//trace("j: " + j + " _ " + map[j]);
		}
	}
	
	/**
	 * If you imagine that the graph has positive y as upwards, this function rotates the graph clockwise
	 * The positive x axis of the rotated map becomes positive y of the normal map
	 * 
	 * @return
	 */
	public function RotateMap():GraphGridMapMinimalist
	{
		
		var returnee:GraphGridMapMinimalist = new GraphGridMapMinimalist(height, width, true);
		
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
	
	public function InvertMap():GraphGridMapMinimalist
	{
		var returnee:GraphGridMapMinimalist = new GraphGridMapMinimalist(width, height, true);
		
		for (i in 0...height)
		{
			for (j in 0...width)
			{
				if (GetTraversable(j, i) == true)
				{
					returnee.SetTraversableFalse(j, i);
				}
				else
				{
					returnee.SetTraversableTrue(j, i);
				}
			}
		}
		
		return returnee;
	}
	
	/**
	 * On a rotated map, this function acts as though it checks upwards on the normal map. To check upwards from example (2, 4), you need to call this function
	 * with index (4, 2) or (indexY, indexX)
	 * 
	 * @param	indexX_
	 * @param	indexY_
	 * @return The amount of traversable bits to the right of the parameters
	 */
	public function CheckBitsRight(indexX_:Int, indexY_:Int):Int
	{
		
		var indexX:Int = Std.int(indexX_ / 32);
		var indexY:Int = indexY_;
		var bitShift:Int = indexX_ % 32;
		
		var num:Int32 = map[indexX + (indexY * correctedWidth)] << bitShift;
		
		var leadingZerosCount:Int = PathUtility.CLZ(num);
		
		if (leadingZerosCount < 32)
		{
			return leadingZerosCount;
		}
		
		var movingIndexX:Int = indexX + 1;
		if (movingIndexX < correctedWidth)
		{
			num = map[movingIndexX + (indexY * correctedWidth)];
		}
		else
		{
			return width - indexX_;
		}
			
		while (true)
		{
			var returnVal = PathUtility.CLZ(num);
			leadingZerosCount += returnVal;
			if (returnVal < 32 || leadingZerosCount + (indexX * 32) >= width)
			{
				return leadingZerosCount - bitShift;
			}
			
			movingIndexX++;
			if (movingIndexX < correctedWidth)
			{
				num = map[movingIndexX + (indexY * correctedWidth)];
			}
			else
			{
				return width - indexX_;
			}
			
		}
		
	}
	
	/**
	 * On a rotated map, this function acts as though it checks downwards on the normal map. To check downwards from example (2, 4), you need to call this function
	 * with index (4, 2) or (indexY, indexX)
	 * 
	 * @param	indexX_ position x
	 * @param	indexY_ position y
	 * @return The amount of traversable bits to the left of the parameters
	 */
	public function CheckBitsLeft(indexX_:Int, indexY_:Int):Int
	{
		
		var indexX:Int = Std.int(indexX_ / 32);
		var indexY:Int = indexY_;
		var bitShift:Int = 31 - indexX_ % 32;
		
		var num:Int32 = map[indexX + (indexY * correctedWidth)] >> bitShift;
		var trailingZerosCount:Int = PathUtility.CTZ(num);
		if (trailingZerosCount < 32)
		{
			return trailingZerosCount;
		}
		
		var movingIndexX:Int = indexX - 1;
		if (movingIndexX > -1)
		{
			num = map[movingIndexX + (indexY * correctedWidth)];
		}
		else
		{
			return indexX_ + 1; // difference between x = 10 and x = 0 is 11, so add one
		}
		
		while (true)
		{
			var returnVal = PathUtility.CTZ(num);
			trailingZerosCount += returnVal;
			if (returnVal < 32 || trailingZerosCount + ((indexX - movingIndexX) * 32) >= width)
			{
				return trailingZerosCount - bitShift; // acount for the fact that we are starting from the opposite direction
			}
			
			movingIndexX--;
			if (movingIndexX > -1)
			{
				num = map[movingIndexX + (indexY * correctedWidth)];
			}
			else
			{
				return indexX_ + 1;
			}
			
		}
		
	}
	
	/*public function GetBitsRight(indexX_:Int, indexY_:Int):Int32
	{
		
		var indexX:Int = Std.int(indexX_ / 32);
		var indexY:Int = indexY_;
		var bitShift:Int = indexX_ % 32;
		
		var num:Int32 = map[indexX + (indexY * correctedWidth)] << bitShift;
		num |= map[indexX + (indexY * correctedWidth)] << (32 - (x_ % 32));
		
		return 0;
		
	}
	
	public function GetBitsLeft(indexX_:Int, indexY_:Int):Int32
	{
		
		return 0;
		
	}*/
	
	public function GetTraversable(x_:Int, y_:Int):Bool
	{
		
		#if debugging
		DebugLogger.Assert(!(x_ < 0 || x_ >= width || y_ < 0 || y_ >= height), "operation out of bounds: x: " + x_ + " y: " + y_ + " mapWidth: " + width + " mapHeight: " + height);
		#end
		
		return ((map[Std.int(x_ / 32) + (y_ * correctedWidth)] >> 31 - x_ % 32) & 1) == 0;
	}
	
	/*
	 * SetTraversable is seperated to remove branching within the method so to make it as potentially fast as possible
	 */
	public function SetTraversableTrue(x_:Int, y_:Int):Void
	{
		
		#if debugging
		DebugLogger.Assert(!(x_ < 0 || x_ >= width || y_ < 0 || y_ >= height), "operation out of bounds: x: " + x_ + " y: " + y_ + " mapWidth: " + width + " mapHeight: " + height);
		#end
		map[Std.int(x_ / 32) + (y_ * correctedWidth)] &= ~(1 << (31 - (x_ % 32)));
	}
	
	/*
	 * SetTraversable is seperated to remove branching within the method so to make it as potentially fast as possible
	 */
	public function SetTraversableFalse(x_:Int, y_:Int):Void
	{
		
		#if debugging
		DebugLogger.Assert(!(x_ < 0 || x_ >= width || y_ < 0 || y_ >= height), "operation out of bounds: x: " + x_ + " y: " + y_ + " mapWidth: " + width + " mapHeight: " + height);
		#end
		
		map[Std.int(x_ / 32) + (y_ * correctedWidth)] |= 1 << (31 - (x_ % 32));
	}
	
	public function GetWidth():Int
	{
		return width;
	}
	
	public function GetHeight():Int
	{
		return height;
	}
	
}