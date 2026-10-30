function actorID=getExecutionActorID(actionID)
%GETEXECUTIONACTORID Map an execution action to its one physical actor.
plan=evalin('base','executionPlan');
idx=mod(double(actionID),numel(plan))+1;
actorID=double(actionID);
if strcmp(plan(idx).action,'buffer_to_pallet')
    prev=find([plan(1:idx-1).boxID]==plan(idx).boxID & ...
        strcmp({plan(1:idx-1).action},'conveyor_to_buffer'),1,'last');
    assert(~isempty(prev),'No stored actor exists for buffered box %d.',plan(idx).boxID);
    actorID=double(prev-1);
end
end
