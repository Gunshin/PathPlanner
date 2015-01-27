package pathPlanner;

// secondary node incase SetParent or similar
class Action
{
	public var actionType:String;
	public var primaryNode:Node;
	public var secondaryNode:Node;
	
	public function new(actionType_:String, primaryNode_:Node, secondaryNode_:Node)
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

	public function AddAction(action_:String, nodeOne_:Node, nodeTwo_:Node)
	{
		actionList.push(new Action(action_, nodeOne_, nodeTwo_));
	}
}