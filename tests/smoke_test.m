function tests = smoke_test
%SMOKE_TEST Basic reproducibility checks for the public repository.
tests = functiontests(localfunctions);
end

function testConstructiveDemo(testCase)
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root);
result = run_demo(Method="constructive",NumCartons=6, ...
    PlanTrajectory=false,OpenModel=false);
verifyEqual(testCase,sum([result.layoutPlan.isPlaced]),6);
verifyGreaterThan(testCase,result.envelopeUtilization,0);
verifyTrue(testCase,all(isfinite([result.layoutPlan.pos])));
end
