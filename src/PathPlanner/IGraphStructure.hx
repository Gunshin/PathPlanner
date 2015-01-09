package pathPlanner;

/**
 * ...
 * @author Michael Stephens
 */
interface IGraphStructure
{
	public function GetNeighbours(node_:Node):Array<DistanceNode>;
	
	public function AddNeighbour(node_:Node, newNeighbour_:Node, ?distance_:Float):Void;
	
	public function RemoveNeighbour(node_:Node, neighbour_:Node):Bool;
}