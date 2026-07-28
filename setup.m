clear; 
clc; 
close all;

projectRoot = 'C:\Users\SOUD_BROWN\Desktop\3DoF ROBOT ARM\final\robotic_arm_3_dof_pick_and_place_mechanism_1_snapshot_1';
urdfFile    = fullfile(projectRoot, 'urdf', ...
              'robotic_arm_3_dof_pick_and_place_mechanism_1_snapshot_1.urdf');
modelName   = 'RoboticArm3DOF_PickPlace';
endEffector = 'piece_5';

cd(projectRoot);
assert(isfile(urdfFile), ['URDF not found: ' urdfFile]);

if ~bdIsLoaded(modelName)
    load_system(fullfile(projectRoot, [modelName '.slx']));
end
open_system(modelName);
blk = @(x) [modelName '/' x];

knownDeadEnds = {'PS1', 'PS_Converter_object'};
for i = 1:numel(knownDeadEnds)
    hits = find_system(modelName, 'SearchDepth', 1, 'Name', knownDeadEnds{i});
    if ~isempty(hits)
        delete_block(hits{1});
        fprintf('Removed leftover block: %s\n', knownDeadEnds{i});
    end
end

staleCandidates = {'JointSource', 'PS2', 'gripMode_src', 'PS_Converter_mode'};
for i = 1:numel(staleCandidates)
    name = staleCandidates{i};
    hits = find_system(modelName, 'SearchDepth', 1, 'Name', name);
    if isempty(hits)
        continue
    end
    h = hits{1};
    lines = get_param(h, 'LineHandles');
    allLineHandles = [lines.Inport(:); lines.Outport(:); lines.Enable(:); lines.Trigger(:)];
    isConnected = any(allLineHandles ~= -1);
    if isConnected
        warning('Block "%s" has connected lines - leaving it in place.', name);
    else
        delete_block(h);
        fprintf('Removed unused block: %s\n', name);
    end
end
save_system(modelName);

robot = importrobot(urdfFile);
robot.DataFormat = 'row';
jointNames = {};
for i = 1:numel(robot.Bodies)
    if ~strcmp(robot.Bodies{i}.Joint.Type, 'fixed')
        jointNames{end+1} = robot.Bodies{i}.Joint.Name;
    end
end
numJ = numel(jointNames);
fprintf('Joints found: %d -> %s\n', numJ, strjoin(jointNames, ', '));
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0 0 0 1 1 1];  
qHome = robot.homeConfiguration;
jointMin = deg2rad(-90 * ones(1, numJ));
jointMax = deg2rad( 90 * ones(1, numJ));

TABLE_Z     = 0.00;
CLEARANCE   = 0.12;
SAFE_RADIUS = 0.12;

pickAbove  = [0.35  0.15  0.25];
pickPos    = [0.35  0.15  0.12];
placeAbove = [0.35 -0.15  0.25];
placePos   = [0.35 -0.15  0.12];

pickPos(3)  = max(pickPos(3),  TABLE_Z + CLEARANCE);
placePos(3) = max(placePos(3), TABLE_Z + CLEARANCE);
if norm(pickPos(1:2)) < SAFE_RADIUS,  pickPos(1)  = SAFE_RADIUS; end
if norm(placePos(1:2)) < SAFE_RADIUS, placePos(1) = SAFE_RADIUS; end

[qAbovePick, ~] = ik(endEffector, trvec2tform(pickAbove),  weights, qHome);
[qPick,      ~] = ik(endEffector, trvec2tform(pickPos),    weights, qAbovePick);
qGrip = qPick;

idx2 = find(strcmp(jointNames, 'revolute_2'), 1);
[qPlaceIK, ~] = ik(endEffector, trvec2tform(placePos), weights, qGrip);

qLift       = qGrip;
qAbovePlace = qGrip;
qPlace      = qGrip;
qPlace(idx2) = qPlaceIK(idx2);
qRelease    = qPlace;

[qRetreat, ~] = ik(endEffector, trvec2tform(placeAbove), weights, qPlace);
qBackHome = qHome;

wrapRow = @(q) min(max(wrapToPi(q), jointMin), jointMax);
qHome        = wrapRow(qHome);
qAbovePick   = wrapRow(qAbovePick);
qPick        = wrapRow(qPick);
qGrip        = wrapRow(qGrip);
qLift        = wrapRow(qLift);
qAbovePlace  = wrapRow(qAbovePlace);
qPlace       = wrapRow(qPlace);
qRelease     = wrapRow(qRelease);
qRetreat     = wrapRow(qRetreat);
qBackHome    = wrapRow(qBackHome);

names = {'Home','AbovePick','Pick','Grip','Lift', ...
         'AbovePlace','Place','Release','Retreat','Home'};
Q = [qHome; qAbovePick; qPick; qGrip; qLift; ...
     qAbovePlace; qPlace; qRelease; qRetreat; qBackHome];

dtMove = 2.0;
dtGrip = 1.2;
dts    = [0, dtMove, dtMove, dtGrip, dtMove, dtMove, dtMove, dtGrip, dtMove, dtMove];
timePoints = cumsum(dts)';
numSamples = 250;
segDurationsRow = diff(timePoints)';
segDurations    = repmat(segDurationsRow, numJ, 1);
[qTraj, ~, ~, tSamples] = trapveltraj(Q', numSamples, 'EndTime', segDurations);

t = tSamples(:);
jointAngles = timeseries(qTraj', t);
assignin('base', 'jointAngles', jointAngles);

tGrip    = timePoints(strcmp(names, 'Grip'));
tRelease = timePoints(strcmp(names, 'Release'));
idxPickRow  = find(strcmp(names, 'Pick'),  1);
idxPlaceRow = find(strcmp(names, 'Place'), 1);

TeePickActual  = getTransform(robot, Q(idxPickRow,:),  endEffector);
TeePlaceActual = getTransform(robot, Q(idxPlaceRow,:), endEffector);
pickPosActual  = tform2trvec(TeePickActual);
placePosActual = tform2trvec(TeePlaceActual);

objectPos = zeros(numel(t), 3);
for i = 1:numel(t)
    if t(i) < tGrip
        objectPos(i,:) = pickPosActual;
    elseif t(i) <= tRelease
        Tee = getTransform(robot, qTraj(:,i)', endEffector);
        objectPos(i,:) = tform2trvec(Tee);
    else
        objectPos(i,:) = placePosActual;
    end
end
objectPose = timeseries(objectPos, t);
assignin('base', 'objectPose', objectPose);
if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'jointAngles_src'))
    add_block('simulink/Sources/From Workspace', blk('jointAngles_src'), ...
        'VariableName', 'jointAngles', 'Position', [30 40 130 70]);
end

demuxHits = find_system(modelName, 'SearchDepth', 1, 'BlockType', 'Demux');
if isempty(demuxHits)
    add_block('simulink/Signal Routing/Demux', blk('Demux'), ...
        'Outputs', num2str(numJ), 'Position', [180 30 200 90]);
    demuxName = 'Demux';
else
    demuxName = get_param(demuxHits{1}, 'Name');
end
try, add_line(modelName, 'jointAngles_src/1', [demuxName '/1'], 'autorouting', 'on'); catch, end

for k = 1:numJ
    ps = sprintf('PS_Converter_%d', k);
    if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', ps))
        add_block('nesl_utility/Simulink-PS Converter', blk(ps), ...
            'Position', [280 20+70*(k-1) 360 50+70*(k-1)]);
        set_param(blk(ps), 'InputSignalUnit', 'rad');
    end
    try, add_line(modelName, sprintf('%s/%d', demuxName, k), sprintf('%s/1', ps), 'autorouting', 'on'); catch, end
    jointBlk = blk(jointNames{k});
    try
        set_param(jointBlk, 'MotionActuationMode', 'InputMotion');
        set_param(jointBlk, 'TorqueActuationMode', 'AutomaticallyComputed');
    catch ME
        warning(['Could not set Actuation on %s (%s). Open it by hand -> ' ...
            'Actuation tab -> Torque = "Automatically Computed", ' ...
            'Motion = "Provided by Input".'], jointNames{k}, ME.message);
    end
end

if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'objectPose_src'))
    add_block('simulink/Sources/From Workspace', blk('objectPose_src'), ...
        'VariableName', 'objectPose', 'Position', [500 400 600 430]);
end

demux1Hits = find_system(modelName, 'SearchDepth', 1, 'Name', 'Demux1');
if isempty(demux1Hits)
    add_block('simulink/Signal Routing/Demux', blk('Demux1'), ...
        'Outputs', '3', 'Position', [620 400 640 460]);
end
try, add_line(modelName, 'objectPose_src/1', 'Demux1/1', 'autorouting', 'on'); catch, end

objPS = {'PS_Converter_object_X', 'PS_Converter_object_Y', 'PS_Converter_object_Z'};
for k = 1:3
    ps = objPS{k};
    if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', ps))
        add_block('nesl_utility/Simulink-PS Converter', blk(ps), ...
            'Position', [680 380+50*(k-1) 760 410+50*(k-1)]);
        set_param(blk(ps), 'InputSignalUnit', 'm');
    end

    try
        set_param(blk(ps), 'FilteringAndDerivatives', 'filter', ...
            'SimscapeFilterOrder', '2', 'InputFilterTimeConstant', '0.01');
    catch ME
        warning(['Could not set Input Handling filtering on %s (%s). ' ...
            'Open it by hand -> Input Handling tab -> "Filter input, ' ...
            'derivatives calculated", order=2, time constant=0.01.'], ps, ME.message);
    end
    try, add_line(modelName, sprintf('Demux1/%d', k), sprintf('%s/1', ps), 'autorouting', 'on'); catch, end
end

save_system(modelName);
disp('Model wiring verified/updated.');

set_param(modelName, 'Solver', 'ode23t');
set_param(modelName, 'StopTime', num2str(t(end)));
save_system(modelName);
disp('Running simulation...');
sim(modelName);