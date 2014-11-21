PathPlanner

*****************

Unfortunately due to a problem with the keyword 'untyped' with hxcs (possibly related to this current bug as of 21/11/2014 https://github.com/HaxeFoundation/haxe/issues/3516) 
i have to include my own 1 line modification of polygonal-ds so that it compiles to cs. polygonal-ds can be found here: https://github.com/polygonal/ds

*****************

This repo is the start of a collection of algorithms written in haxe so that i can port them to whichever language i need them in.

Currently working algorithms:
- A*

Working on:
- A* with JPS


Usage:

To use this library you need to build the DLL's first.

<CREATE THE BUILDS AND EXPLAIN HOW TO BUILD>


Usage In Applications:

To use the DLL's in your applications, there are a couple of things you need to be aware of.

The algorithms support 2 different graphs. An indirect graph (GraphStructureIndirect) where the nodes are not assumed to be neatly aligned with each other (like connecting cities to each other on a map),
and a direct graph (Map) which has a uniform rectangular grid of nodes. The direct graph also supports indirect links through a hashmap, however, the main difference between them is that the nodes in a 
direct graph have 8 neighbours each (unless they are on the edge), 2 horizontal, 2 vertical and 4 diagonal.

In your application, you will need to decide which to use. You could use both, but i am not sure how the 2 different graph structures would work together.

To use the Map, simply create a map object. 
The line below will create a square grid of width = 10 and height = 10. To get the individual nodes, just call map.GetNodeByIndex().

var map:Map = new Map(10, 10);


For adding indirect neighbours, regardless of whether you are using the Map or GraphStructureIndirect, simply call as shown below:

node.AddNeighbour(map.GetNodeByIndex(0, 9));
node.RemoveNeighbour(map.GetNodeByIndex(0, 9));


To use the algorithms, simply create one of the pathplanner objects, and call FindMap. FindMap takes a startNode, endNode, and an anonymous Heuristic function that generates a value based on the currentNode and endNode.


Examples of these can be found in Main. 