function sim3dPosition = worldToSim3DPosition(worldPosition)
%WORLDTOSIM3DPOSITION Convert Robotics/Simulink world XYZ to Sim3D XYZ.
%
% The MathWorks 3D scene in this example uses the opposite Y axis from the
% Robotics System Toolbox world frame used by the layout and IK code.

worldPosition = double(worldPosition(:)).';
sim3dPosition = [worldPosition(1), -worldPosition(2), worldPosition(3)];
end
