package pathPlanner;

// secondary node incase SetParent or similar
class Action
{
	public var actionType:String;
	public var primary:Position;
	public var secondary:Position;
	
	public function new(actionType_:String, primary_:Position, secondary_:Position)
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

	public function AddAction(action_:String, primary_:Position, secondary_:Position)
	{
		actionList.push(new Action(action_, primary_, secondary_));
	}
	
	public function GetActionTypes():Array<String>
	{
		var typeList:Array<String> = new Array<String>();
		
		for (i in actionList)
		{
			var contained:Bool = false;
			
			for (t in typeList)
			{
				if (i.actionType == t)
				{
					contained = true;
				}
			}
			
			if (!contained)
			{
				typeList.push(i.actionType);
			}
		}
		
		return typeList;
	}
	
	public function GetCountOfActionType(actionType_:String):Int
	{
		var count:Int = 0;
		for (i in actionList)
		{
			if (actionType_ == i.actionType)
			{
				count++;
			}
		}
		return count;
	}
}