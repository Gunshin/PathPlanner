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
			distanceBetween = Math.sqrt(Math.pow(connectedNode_.GetPosition().GetX() - currentNode_.GetPosition().GetX(), 2) + Math.pow(connectedNode_.GetPosition().GetY() - currentNode_.GetPosition().GetY(), 2));
			distanceCalculated = true;
		}
		
	}
	
}