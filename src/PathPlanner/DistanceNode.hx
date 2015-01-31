package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 */
class DistanceNode
{
	
	public var connectedNode:Node;
	public var distanceBetween:Float = 0;
	public var distanceCalculated:Bool = false;
	
	public function new(connectedNode_:Node, ?currentNode_:Node, ?distance_:Float)
	{
		
		connectedNode = connectedNode_;
		
		if(distance_ != null)
		{
			distanceBetween = distance_;
			distanceCalculated = true;
		}
		else if (currentNode_ != null)
		{
			distanceBetween = Position.Distance(connectedNode.GetPosition(), currentNode_.GetPosition());
			distanceCalculated = true;
		}
		
	}
	
	
	
}