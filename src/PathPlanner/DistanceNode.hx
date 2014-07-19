package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 */
class DistanceNode
{
	
	public var connectedNode:Node;
	public var distanceBetween:Float;
	
	public function new(connectedNode_:Node, ?currentNode_:Node, ?distance_:Float)
	{
		
		connectedNode = connectedNode_;
		
		if (currentNode_ != null)
		{
			distanceBetween = Math.sqrt(Math.pow(connectedNode_.get_x() - currentNode_.get_x(), 2) + Math.pow(connectedNode_.get_y() - currentNode_.get_y(), 2));
		}
		else
		{
			distanceBetween = distance_;
		}
		
	}
	
}