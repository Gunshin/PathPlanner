package pathPlanner;
import haxe.Int32;

/**
 * ...
 * @author Michael Stephens
 */
class PathUtility 
{

	static inline public function ReconstructPathFromNodes(endNode_:Node):Array<Position>
	{
		var path:Array<Position> = new Array<Position>();
		var currentNode_:Node = endNode_;
		
		while (currentNode_ != null)
		{
			
			path.push(currentNode_.GetPosition());
			currentNode_ = currentNode_.GetParent();
			
		}
		path.reverse();
		return path;
		
	}
	
	static inline public function ReconstructPathFromPositionMap(endPosition_:Position, positionMap_:Array<Position>, positionMapWidth_:Int):Array<Position>
	{
		var path:Array<Position> = new Array<Position>();
		var currentPosition_:Position = endPosition_;
		
		while (currentPosition_ != null)
		{
			
			path.push(currentPosition_);
			currentPosition_ = positionMap_[currentPosition_.GetX() + currentPosition_.GetY() * positionMapWidth_];
			
		}
		path.reverse();
		return path;
		
	}
	
	/*static public function Insert(array_:Array<Node>, node_:Node)
	{
		
		for (i in 0...array_.length)
		{
			if (node_.heuristic > array_[i].heuristic)
			{
				array_.insert(i, node_);
				return;
			}
		}
		
		array_.push(node_);
		
	}*/
	
	/*
	 * returns the index of the node, otherwise if the node is not in the array, it returns -1
	 */
	static public function ContainsNodeHierarchical(array_:Array<NodeHierarchical>, node_:NodeHierarchical):Int
	{
		for (i in 0...array_.length)
		{
			if (node_ == array_[i])
			{
				return i;
			}
		}
		
		return -1;
	}
	
	static public function ContainsNode(array_:Array<DistanceNode>, node_:Node):Int
	{
		for (i in 0...array_.length)
		{
			if (node_ == array_[i].connectedNode)
			{
				return i;
			}
		}
		
		return -1;
	}
	
	/*
	 * Count Trailing Zeros assuming we are looking at data directly in memory assuming Big Endian
	 */
	public static inline function CTZ(number:Int32):Int32
	{
		#if cpp
		var x:Int32 = number;
		untyped __cpp__('unsigned long index');
		untyped __cpp__('unsigned char zero = _BitScanForward(&index, x)'); // special microsoft compiler intrinsic
		return untyped __cpp__('index > 32 || zero == 0 ? 32 : index'); // need the zero byte to know whether the mask (x) is 0 or not
		#else
		var x:Int32 = number;
		if (x == 0)
		{
			return 32;
		}
		
		var n:Int32 = 0;
		if ((x & 0x0000FFFF) == 0)
		{
			n = n + 16;
			x = x >> 16;
		}
		if ((x & 0x000000FF) == 0) 
		{
			n = n + 8;
			x = x >> 8;
		}
		if ((x & 0x0000000F) == 0)
		{
			n = n + 4;
			x = x >> 4;
		}
		if ((x & 0x00000003) == 0)
		{
			n = n + 2;
			x = x >> 2;
		}
		if ((x & 0x00000001) == 0)
		{
			n = n + 1;
		}
		return n;
		#end
	}
	
	/*
	 * Count Trailing Zeros assuming we are looking at data directly in memory assuming Big Endian
	 */
	public static inline function CLZ(number:Int32):Int32
	{
		#if cpp
		var x:Int32 = number;
		untyped __cpp__('unsigned long index');
		untyped __cpp__('unsigned char zero = _BitScanReverse(&index, x)'); // special microsoft compiler intrinsic
		return untyped __cpp__('index > 32 || zero == 0 ? 32 : 31 - index'); // need the zero byte to know whether the mask (x) is 0 or not
		#else
		number |= (number >> 1);
        number |= (number >> 2);
        number |= (number >> 4);
        number |= (number >> 8);
        number |= (number >> 16);
        return(32 - Ones(number));
		#end
	}
	
	static inline function Ones(x:Int32):Int32
	{
        x -= ((x >> 1) & 0x55555555);
        x = (((x >> 2) & 0x33333333) + (x & 0x33333333));
        x = (((x >> 4) + x) & 0x0f0f0f0f);
        x += (x >> 8);
        x += (x >> 16);
        return(x & 0x0000003f);
	}
	
}