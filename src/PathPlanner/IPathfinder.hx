package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 */
class IPathfinder 
{

	public function FindPath(startNode_:Node, endNode_:Node):Array<Node> { return null; };
	
	function ReconstructPath(endNode_:Node):Array<Node>
	{
		
		var path:Array<Node> = new Array<Node>();
		var currentNode_:Node = endNode_;
		
		while (currentNode_ != null)
		{
			
			path.push(currentNode_);
			currentNode_ = currentNode_.get_parent();
			
		}
		path.reverse();
		return path;
		
	}
	
	function Insert(array_:Array<Node>, node_:Node)
	{
		
		for (i in 0...array_.length)
		{
			if (node_.get_heuristic() > array_[i].get_heuristic())
			{
				array_.insert(i, node_);
				return;
			}
		}
		
		array_.push(node_);
		
	}
	
	function Contains(array_:Array<Node>, node_:Node):Int
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
	
}