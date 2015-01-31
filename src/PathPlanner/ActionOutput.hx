package pathPlanner;

// secondary node incase SetParent or similar
class Action<T>
{
	public var actionType:String;
	public var primary:T;
	public var secondary:T;
	
	public function new(actionType_:String, primary_:T, secondary_:T)
	{
		actionType = actionType_;
		primary = primary_;
		secondary = secondary_;
	}
}

/**
 * ...
 * @author ...
 */
class ActionOutput<T>
{
	@:protected
	var actionList:Array<Action<T>> = new Array<Action<T>>();

	public function new() 
	{
		
	}
	
	public function GetActionList():Array<Action<T>>
	{
		return actionList;
	}
	
	public function ResetActionList()
	{
		actionList = new Array<Action<T>>();
	}

	public function AddAction(action_:String, primary_:T, secondary_:T)
	{
		actionList.push(new Action<T>(action_, primary_, secondary_));
	}
}