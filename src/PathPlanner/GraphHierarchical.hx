package pathPlanner;

import de.polygonal.ds.HashTable;

/**
 * ...
 * @author Michael Stephens
 */
class GraphHierarchical 
{
	
	var hierarchyLists:Array<Array<NodeHierarchical>> = new Array<Array<NodeHierarchical>>(); // this is the container of all hierarchical nodes in the generated map

	public function new() 
	{
		
	}
	
	public function GenerateFromGridGraph(gridGraph_:GraphGridMap):Void
	{
		
		hierarchyLists[0] = new Array<NodeHierarchical>();
		
		// we are using this hash table to store a reference between the old normal node from the GraphGridMap, with the corresponding new NodeHierarchical
		// useful for setting up neighbours
		var neighbourHashTable:HashTable <Node, NodeHierarchical> = new HashTable <Node, NodeHierarchical>(4, gridGraph_.GetWidth() * gridGraph_.GetHeight());
		
		// first we create and add all new nodehierarchicals
		for (i in 0...gridGraph_.GetHeight())
		{
			
			for (j in 0...gridGraph_.GetWidth())
			{
				
				var node:Node = gridGraph_.GetNodeByIndex(j, i);
				
				if (!node.GetTraversable()) // if this node is not traversable, lets ignore it
				{
					continue;
				}
				
				var hierNode:NodeHierarchical = new NodeHierarchical(0); // concrete layer
				hierNode.SetPosition(new Position(j, i));
				
				hierarchyLists[0].push(hierNode);
				
				// add to the hashmap
				neighbourHashTable.set(node, hierNode);
			}
			
		}
		
		// we then set up the connections after the nodes have been created
		for (i in 0...gridGraph_.GetHeight())
		{
			
			for (j in 0...gridGraph_.GetWidth())
			{
				
				var node:Node = gridGraph_.GetNodeByIndex(j, i);
				
				if (!node.GetTraversable()) // if this node is not traversable, lets ignore it
				{
					continue;
				}
				
				var currentHierNode:NodeHierarchical = neighbourHashTable.get(node);
				
				// grab the neighbours and add them
				var neighbours:Array<Node> = gridGraph_.GetRawNeighbours(node);
				for (neighbour in neighbours)
				{
					
					if (neighbour == null)
					{
						continue;
					}
					
					// make sure this neighbour is traversable
					if (neighbour.GetTraversable())
					{
						
						// guaranteed to not be null since every traversable node was added to the hash table
						var neighbourHierNode:NodeHierarchical = neighbourHashTable.get(neighbour);
						
						currentHierNode.AddNeighbour(neighbourHierNode);
						
					}
					
				}
				
			}
			
		}
		
	}
	
	/*
	 * 
	 * currently this generates the hierarchy by just grabbing what ever happens to be at the end
	 * of a list, grabbing its neighbours and adding them all to a single node above it. The nodes
	 * can therefor have any number of child nodes between 1 and 9.
	 * 
	 */
	public function GenerateHierarchy(levelCount_:Int)
	{
		
		// lets guarantee that the non-concrete layers all are removed
		
		for (i in 0...levelCount_)
		{
			
			var hierarchyCopy:Array<NodeHierarchical> = hierarchyLists[i].copy();
			hierarchyLists[i + 1] = new Array<NodeHierarchical>();
			
			// lets build parents for every node in the list
			while (hierarchyCopy.length > 0)
			{
				
				var hierNode:NodeHierarchical = hierarchyCopy.pop(); //just grab the last node of the list. doesnt particularly matter which order we do this.
				// this also removed it from the temporary list.
				
				var hierNodeParent:NodeHierarchical = new NodeHierarchical(i + 1);// level above
				hierarchyLists[i + 1].push(hierNodeParent);
				
				//set the node we currently have to be a child of the new parent
				hierNode.AddHierarchicalParent(hierNodeParent);
				hierNodeParent.AddHierarchicalChild(hierNode);
				
				//now the same for the neighbours of the current node
				var neighbours:Array<DistanceNode> = hierNode.GetNeighbours();
				for (distNode in neighbours)
				{
					var hierNeighbour:NodeHierarchical = cast(distNode.connectedNode, NodeHierarchical);
					// remove from list since it now has a parent
					if (hierarchyCopy.remove(hierNeighbour)) // this neighbour was succesfully removed from the list meaing it had no parent
					{
						hierNeighbour.AddHierarchicalParent(hierNodeParent);
						hierNodeParent.AddHierarchicalChild(hierNeighbour);
					}
					
				}
				
			}
			
			// now that the parents have been built, we need to iterate over them and build the neighbour structures for them
			
			for (parent in hierarchyLists[i + 1])
			{
				parent.GenerateSameLevelConnections();
			}
			
		}
	}
	
	public function GetLevelHierarchy(level_:Int):Array<NodeHierarchical>
	{
		
		return hierarchyLists[level_];
		
	}
	
}