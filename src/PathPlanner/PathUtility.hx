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
	
}