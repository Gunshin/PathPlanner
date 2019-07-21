package pathPlanner;

import polygonal.ds.PriorityQueue;

/**
 * ...
 * @author Michael Stephens
 */
class HPAStar implements IPathfinder 
{

	var graphHier:GraphHierarchical;
	
	@:protected
	var heuristicFunction:
	#if cs
	cs.system.Func_3<Position,Position,Float>;
	#else
	Position -> Position -> Float;
	#end
	
	#if action_output
	var actionOutput:ActionOutput;
	#end
	
	public function new(heuristicFunction_:
		#if cs
		cs.system.Func_3<Position,Position,Float>
		#else
		Position -> Position -> Float
		#end
		,
		graphHier_:GraphHierarchical) 
	{
		heuristicFunction = heuristicFunction_;
		graphHier = graphHier_;
	}
	
	public function FindPath(param_:PathplannerParameter):Array<Position>
	{
		
		#if action_output
		actionOutput = new ActionOutput();
		#end
		
		var correctStart:NodeHierarchical = graphHier.GetHierarchicalEquivalent(param_.startNode);
		var correctEnd:NodeHierarchical = graphHier.GetHierarchicalEquivalent(param_.goalNode);
		
		// for now hard coded to the first layer until i sort out the connections
		var startParent:NodeHierarchical = correctStart.GetHierarchicalParent();
		var endParent:NodeHierarchical = correctEnd.GetHierarchicalParent();
		
		var path:Array<Position> = new Array<Position>();
		
		// if the parents of the start and end node are the same, do a quick path between them and return
		if (startParent == endParent)
		{
			trace("returned same parent");
			var nodes:Array<Node> = AStarPath(param_.startNode, param_.goalNode);
			for (node in nodes)
			{
				path.push(node.GetPosition());
			}
			
			return path;
		}
		
		var hierNodePath:Array<Node> = AStarPath(startParent, endParent); // lets find a general path
		#if action_output // simple way to record the path
		for (i in 0...hierNodePath.length - 1)
		{
			actionOutput.AddAction("AbstractionPath:1", hierNodePath[i].GetPosition(), hierNodePath[i + 1].GetPosition());
		}
		#end
		
		if (hierNodePath == null)// if this path is null, we need to catch it. can happen.
		{
			trace("returned null");
			return null;
		}
		
		var hierHierPath:Array<NodeHierarchical> = CastToNodeHierarchical(hierNodePath);
		
		var currentConcreteNode:Node = correctStart;
		// add concrete start to path since it is ignored due to the way we add the concrete nodes
		path.push(currentConcreteNode.GetPosition());
		
		for (i in 0...hierHierPath.length - 1)
		{
			
			// each parent (non-concrete node) has a child connector/neighbour node to the next adjacent node on the same level.
			// its a cache of which child node leads to a specific neighbour node
			// what we want to do is find a path from the child in our current node, to the neighbour connector of the next in the path to our node.
			// eg. find which child node of our neighbour connects to us, and find a path to it. continue until done.
			
			var concreteTargets:Array<NodeHierarchical> = hierHierPath[i + 1].GetConnectionChildren(hierHierPath[i]); //get connections to current node
			
			var localPath:Array<Node> = AStarPath(currentConcreteNode, concreteTargets[0]); // get path to first node
			
			
			
			if (localPath == null) // just incase no local path can be found (i think its impossible to be the case)
			{
				throw "something went wrong: " + currentConcreteNode.GetPosition().ToString() + " _ " + concreteTargets[0].GetPosition().ToString();
			}
			
			// since we have a path, add it!
			// start at one in, since the first node of the new path is actually already on the end of the path
			for (i in 1...localPath.length)
			{
				path.push(localPath[i].GetPosition());
			}
			
			currentConcreteNode = concreteTargets[0]; // set current node to next place
		}
		
		// now that we reached the final abstract node, we need to find the path to the end node
		{
			var nodes:Array<Node> = AStarPath(currentConcreteNode, correctEnd);
			// start at one in, since the first node of the new path is actually already on the end of the path
			for (i in 1...nodes.length)
			{
				path.push(nodes[i].GetPosition());
			}
		}
		
		return path;
	}
	
	function AStarPath(startNode_:Node, endNode_:Node):Array<Node>
	{
		
		startNode_.SetParent(null);
		
		var open:PriorityQueue<Node> = new PriorityQueue<Node>(16, true);
		var closed:PriorityQueue<Node> = new PriorityQueue<Node>(16, true);
		
		open.enqueue(startNode_);
		#if action_output
		actionOutput.AddAction("AddToOpen", startNode_.GetPosition(), null);
		#end
		while (!open.isEmpty())
		{
			
			var currentNode:Node = open.dequeue();
			closed.enqueue(currentNode);
			#if action_output
			actionOutput.AddAction("AddToClosed", currentNode.GetPosition(), null);
			#end
			
			var neighbours:Array<DistanceNode>;
			
			if (currentNode == endNode_)
			{				
				return PathUtility.ReconstructNodePathFromNodes(endNode_);
			}
			else if((neighbours = currentNode.GetNeighbours()).length > 0)
			{
				for (i in 0...neighbours.length)
				{
					if (neighbours[i] != null && neighbours[i].connectedNode.GetTraversable() == true) // if node is traversable, expand it
					{
						//improveTimer.Start();
						Improve(currentNode, neighbours[i], endNode_, open, closed);
						//improveTimer.Stop();
					}
					
				}
			}
			
		}
		
		trace("no path found");
		return null;// no path is found
	}
	
	// Procedure Improve as listed on page 70 of Heuristic Search: Theory and Applications by Stefan Edelkamp and Stefan Schrodl
	function Improve(
	currentNode_:Node,
	successorNode_:DistanceNode,
	endNode_:Node,
	open_:PriorityQueue<Node>,
	closed_:PriorityQueue<Node>
	):Void
	{
		#if action_output
		actionOutput.AddAction("Expand", successorNode_.connectedNode.GetPosition(), null);
		#end
		if (open_.contains(successorNode_.connectedNode))
		{
			if (currentNode_.GetPathCost() + successorNode_.distanceBetween < successorNode_.connectedNode.GetPathCost())
			{
				successorNode_.connectedNode.SetParent(currentNode_);
				#if action_output
				actionOutput.AddAction("SetParent", successorNode_.connectedNode.GetPosition(), currentNode_.GetPosition());
				#end
				
				successorNode_.connectedNode.SetPathCost(currentNode_.GetPathCost() + successorNode_.distanceBetween);
				#if cs
				successorNode_.connectedNode.heuristic = heuristicFunction.Invoke(successorNode_.connectedNode.GetPosition(), endNode_.GetPosition());
				#else
				successorNode_.connectedNode.heuristic = heuristicFunction(successorNode_.connectedNode.GetPosition(), endNode_.GetPosition()); // the heuristic should not change, but under the assumption
				#end
				// that a function may change it, we will forced it to update everytime.
				
				// since it already belongs in the queue, we need to reset its priority
				open_.reprioritize(successorNode_.connectedNode, successorNode_.connectedNode.GetPathCost() + successorNode_.connectedNode.heuristic);
			}
		}
		else if (closed_.contains(successorNode_.connectedNode))
		{
			// if it is in the closed set, but we have found a better route to this neighbour, update it with the better route.
			if (currentNode_.GetPathCost() + successorNode_.distanceBetween < successorNode_.connectedNode.GetPathCost())
			{
				successorNode_.connectedNode.SetParent(currentNode_);
				#if action_output
				actionOutput.AddAction("SetParent", successorNode_.connectedNode.GetPosition(), currentNode_.GetPosition());
				#end
				
				successorNode_.connectedNode.SetPathCost(currentNode_.GetPathCost() + successorNode_.distanceBetween);
				#if cs
				successorNode_.connectedNode.heuristic = heuristicFunction.Invoke(successorNode_.connectedNode.GetPosition(), endNode_.GetPosition());
				#else
				successorNode_.connectedNode.heuristic = heuristicFunction(successorNode_.connectedNode.GetPosition(), endNode_.GetPosition()); // the heuristic should not change, but under the assumption
				#end
				// that a function may change it, we will forced it to update everytime.
				
				closed_.remove(successorNode_.connectedNode); // remove it from the closed list so it can be explored again to update values
				successorNode_.connectedNode.priority = successorNode_.connectedNode.GetPathCost() + successorNode_.connectedNode.heuristic;
				open_.enqueue(successorNode_.connectedNode); // add to open list so we can allow exploration
				#if action_output
				actionOutput.AddAction("AddToOpen", successorNode_.connectedNode.GetPosition(), null);
				#end
			}
		}
		else
		{
			// if the neighbour is not in the open set, add it.
			successorNode_.connectedNode.SetParent(currentNode_);
			#if action_output
			actionOutput.AddAction("SetParent", successorNode_.connectedNode.GetPosition(), currentNode_.GetPosition());
			#end
			
			successorNode_.connectedNode.SetPathCost(currentNode_.GetPathCost() + successorNode_.distanceBetween);
			#if cs
			successorNode_.connectedNode.heuristic = heuristicFunction.Invoke(successorNode_.connectedNode.GetPosition(), endNode_.GetPosition());
			#else
			successorNode_.connectedNode.heuristic = heuristicFunction(successorNode_.connectedNode.GetPosition(), endNode_.GetPosition()); // set the heuristic for the node
			#end
			
			// since we are adding it to the queue for the first time, we need to set its priority
			successorNode_.connectedNode.priority = successorNode_.connectedNode.GetPathCost() + successorNode_.connectedNode.heuristic;
			open_.enqueue(successorNode_.connectedNode);
			#if action_output
			actionOutput.AddAction("AddToOpen", successorNode_.connectedNode.GetPosition(), null);
			#end
		}
	}
	
	// the only thing that is used within debug logger atm is assert
	//public function AttachDebugLogger(debugLogger_:DebugLogger):Void;
	
	public function GetActionOutput():ActionOutput
	{
		#if action_output
		return actionOutput;
		#else
		throw "PathPlanner library has not been compiled with action output as needed. Recompile with compiler command -D action_output";
		#end
	}
	
	public static function CastToNodeHierarchical(nodeArray_:Array<Node>):Array<NodeHierarchical>
	{
		var array:Array<NodeHierarchical> = new Array<NodeHierarchical>();
		for (node in nodeArray_)
		{
			array.push(cast(node, NodeHierarchical));
		}
		return array;
	}
	
}