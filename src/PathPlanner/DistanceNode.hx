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
			distanceBetween = Distance(connectedNode, currentNode_);
			distanceCalculated = true;
		}
		
	}
	
	public static inline function Distance(nodeA_:Node, nodeB_:Node):Float
	{
		return Math.sqrt(Math.pow(nodeA_.GetPosition().GetX() - nodeB_.GetPosition().GetX(), 2) + Math.pow(nodeA_.GetPosition().GetY() - nodeB_.GetPosition().GetY(), 2));
	}
	
}