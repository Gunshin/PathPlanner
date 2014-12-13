package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 */
class PathUtility 
{

	static public function ReconstructPath(endNode_:Node):Array<Node>
	{
		
		var path:Array<Node> = new Array<Node>();
		var currentNode_:Node = endNode_;
		
		while (currentNode_ != null)
		{
			
			path.push(currentNode_);
			currentNode_ = currentNode_.GetParent();
			
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
	
	static public function Contains(array_:Array<DistanceNode>, node_:Node):Int
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
	
	public static inline function CLZ(number:Int):Int
	{
		#if cpp
		var x:Int = number;
		untyped __cpp__('unsigned long index');
		untyped __cpp__('_BitScanForward(&index, x)'); // special microsoft compiler intrinsic
		return untyped __cpp__('index');
		#else
		var x:Int = number;
		if (x == 0)
		{
			return 32;
		}
		
		var n:Int = 0;
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
	
	public static inline function CTZ(number:Int):Int
	{
		#if cpp
		var n:UInt = number;
		return untyped __cpp__('__lzcnt(n)'); // special microsoft compiler intrinsic
		#else
		number |= (number >> 1);
        number |= (number >> 2);
        number |= (number >> 4);
        number |= (number >> 8);
        number |= (number >> 16);
        return(32 - Ones(number));
		#end
	}
	
	static inline function Ones(x:Int):Int
	{
        x -= ((x >> 1) & 0x55555555);
        x = (((x >> 2) & 0x33333333) + (x & 0x33333333));
        x = (((x >> 4) + x) & 0x0f0f0f0f);
        x += (x >> 8);
        x += (x >> 16);
        return(x & 0x0000003f);
	}
	
}