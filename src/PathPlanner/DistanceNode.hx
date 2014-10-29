package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 */
class DistanceNode
{
	
	public var connectedNode:Node;
	public var distanceBetween:Float = 0;
	
	public function new(connectedNode_:Node, ?currentNode_:Node, ?distance_:Float)
	{
		
		connectedNode = connectedNode_;
		
		if (currentNode_ != null)
		{
			distanceBetween = Math.sqrt(Math.pow(connectedNode_.x - currentNode_.x, 2) + Math.pow(connectedNode_.y - currentNode_.y, 2));
		}
		else if(distance_ != null)
		{
			distanceBetween = distance_;
		}
		
	}
	
}