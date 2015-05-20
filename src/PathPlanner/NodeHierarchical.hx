package pathPlanner;

import de.polygonal.ds.HashTable;

/**
 * ...
 * @author Michael Stephens
 */
class NodeHierarchical extends Node
{
	
	@:protected
	var level:Int = -1; // this is the current level of this particular hierarchical node. 0 = concrete level
	
	@:protected
	var hierarchicalChildren:Array<NodeHierarchical> = new Array<NodeHierarchical>(); // any children under this node in the level directly below
	
	@:protected
	var hierarchicalParent:NodeHierarchical = null; // the node in the next layer up that this node is bound to
	
	// This is here so that we can document how to get from this node to the neighbour.
	// The key is the same level neighbour node
	// The value array is a collection of child nodes that have neighbours of whom their parents are this nodes neighbour.
	@:protected
	var neighbourChildConnections:HashTable<NodeHierarchical, Array<NodeHierarchical>> = new HashTable<NodeHierarchical, Array<NodeHierarchical>>(4, 16);
	
	
	public function new(level_:Int) 
	{
		level = level_;
		
		super(new Position(0, 0), true, new GraphStructureIndirect());
		
	}
	
	
	/*
	 * return true if the child was added successfully.
	 * 
	 * may return false if the child is not in the layer directly below the level of this node.
	 */
	public function AddHierarchicalChild(child_:NodeHierarchical):Bool
	{
		
		if (child_.GetLevel() != GetLevel() - 1) // level below?
		{
			return false;
		}
		
		hierarchicalChildren.push(child_);
		
		SetPosition(GetAveragePosition(hierarchicalChildren));
		
		return true;
	}
	
	/*
	 * return true if the parent was added successfully.
	 * 
	 * may return false if the parent is not in the layer directly above the level of this node.
	 */
	public function AddHierarchicalParent(parent_:NodeHierarchical):Bool
	{
		
		if (parent_.GetLevel() != GetLevel() + 1) // level above?
		{
			return false;
		}
		
		hierarchicalParent = parent_;
		
		return true;
	}
	
	public function HasHierarchicalChild(child_:NodeHierarchical):Bool
	{
		return PathUtility.ContainsNodeHierarchical(hierarchicalChildren, child_) != -1;
	}
	
	public function HasHierarchicalParent(parent_:NodeHierarchical):Bool
	{
		return hierarchicalParent == parent_;
	}
	
	public function GetHierarchicalChildren():Array<NodeHierarchical>
	{
		return hierarchicalChildren;
	}
	
	public function GetHierarchicalParent():NodeHierarchical
	{
		return hierarchicalParent;
	}
	
	public function GenerateSameLevelConnections():Void
	{
		
		for (child in hierarchicalChildren)
		{
			DebugLogger.Assert(child != null, "child is null");
			// this is the base class method, but since we are using hierarchical nodes, we can guarantee they are all hierarchical nodes aswell
			var childNeighbours:Array<DistanceNode> = child.GetNeighbours(); 
			
			for (neighbour in childNeighbours)
			{
				DebugLogger.Assert(neighbour.connectedNode != null, "connectedNode is null");
				var nodeHier:NodeHierarchical = cast(neighbour.connectedNode, NodeHierarchical);
				DebugLogger.Assert(nodeHier != null, "casted node is null");
				
				// if the parent of the childs neighbour is not us, that means we have a border connection
				if (nodeHier.GetHierarchicalParent() != this)
				{
					AddSameLevelNeighbour(child, nodeHier.GetHierarchicalParent()); // add the node to our neighbour list
				}
				
			}
			
		}
		
	}
	
	/*
	 * 
	 * This function adds same level nodes to the neighbour list.
	 * 
	 * @ourChildConnector_: The child of this node that connects this node to the same level neighbour
	 * 
	 * @return: can return false if ourChildConnector_ is not a child of this node, or the levels are incorrect
	 * 
	 */
	public function AddSameLevelNeighbour(ourChildConnector_:NodeHierarchical, sameLevelNeighbour_:NodeHierarchical):Bool
	{
		
		if (PathUtility.ContainsNodeHierarchical(hierarchicalChildren, ourChildConnector_) == -1 || ourChildConnector_.GetLevel() != level - 1)
		{
			return false;
		}
		
		AddNeighbour(sameLevelNeighbour_); // this may not add the neighbour if it already exists in the list
		
		var childConnectors:Array<NodeHierarchical> = neighbourChildConnections.get(sameLevelNeighbour_);

		if (childConnectors != null)
		{
			childConnectors.push(ourChildConnector_);
		}
		else
		{
			childConnectors = new Array<NodeHierarchical>();
			childConnectors.push(ourChildConnector_);
			neighbourChildConnections.set(sameLevelNeighbour_, childConnectors);
		}
		
		return true;
		
	}
	
	public function GetLevel():Int
	{
		return level;
	}
	
	public function GetConnectionChildren(neighbour_:NodeHierarchical):Array<NodeHierarchical>
	{
		
		return neighbourChildConnections.get(neighbour_);
		
	}
	
	public static function GetAveragePosition(nodes_:Array<NodeHierarchical>):Position
	{
		var x:Int = 0;
		var y:Int = 0;
		
		for (node in nodes_)
		{
			x += node.GetPosition().GetX();
			y += node.GetPosition().GetY();
		}
		var finalX:Int = Math.round(x / nodes_.length);
		var finalY:Int = Math.round(y / nodes_.length);
		return new Position(finalX, finalY);
	}
}