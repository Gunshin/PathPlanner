package pathPlanner;

// secondary node incase SetParent or similar
class Action
{
	public var actionType:Int;
	public var primaryNode:Node;
	public var secondaryNode:Node;
	
	public function new(actionType_:Int, primaryNode_:Node, secondaryNode_:Node)
	{
		actionType = actionType_;
		primaryNode = primaryNode_;
		secondaryNode = secondaryNode_;
	}
}

/**
 * ...
 * @author ...
 */
class ActionOutput 
{
	
	public var actionTypes:Array<String> = [
	"Expand",
	"AddToOpen",
	"AddToClose",
	"SetParent"
	];
	
	@:protected
	var actionTypeMap:Map<String, Int> = [
		"Expand" => 1,
		"AddToOpen" => 2,
		"AddToClosed" => 3,
		"SetParent" => 4
	];
	
	@:protected
	var actionList:Array<Action> = new Array<Action>();

	public function new() 
	{
		
	}
	
	public function GetActionList():Array<Action>
	{
		return actionList;
	}
	
	public function ResetActionList()
	{
		actionList = new Array<Action>();
	}
	
	public function Expand(node:Node)
	{
		actionList.push(new Action(1, node, null));
	}
	
	public function AddToOpen(node:Node)
	{
		actionList.push(new Action(2, node, null));
	}
	
	public function AddToClosed(node:Node)
	{
		actionList.push(new Action(3, node, null));
	}
	
	public function SetParent(node:Node, parent:Node)
	{
		actionList.push(new Action(4, node, parent));
	}
	
	public function GetActionKeysValue(actionKey:String)
	{
		return actionTypeMap.get(actionKey);
	}
	
}