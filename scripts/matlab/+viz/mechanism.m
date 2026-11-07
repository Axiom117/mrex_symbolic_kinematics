function result = mechanism(dslYaml, configYaml)
%MECHANISM  Validate & visualize a full modular mechanism assembly.
%   MECHANISM(DSLYAML) parses a mechanism-assembly DSL file from
%   specs/dsl/examples/*.yaml, delegates DSL→IR expansion to ir.Expander
%   (pure symbolic pipeline), substitutes joint values from config, then
%   draws:
%     - each body as imported STL geometry when available (colored by type)
%     - each frame/port as an RGB coordinate triad (X=red, Y=green, Z=blue)
%     - each mate as a diagnostic segment between the paired port origins
%       (zero-length when aligned; a visible gap reveals mis-mated ports)
%     - each 1-DOF joint axis as a highlighted segment
%
%   MECHANISM(DSLYAML, CONFIGYAML) loads per-instance joint variable
%   values (revolute q, prismatic dx/dy/dz) from a config YAML keyed
%   by instance name -> variable.  These values are substituted into the
%   symbolic poses at render time (via ir.Expander.evaluateNumeric).
%   Unlisted joint variables default to 0 (zero pose).  Geometric module
%   parameters (cubeLength, tipDistance, ...) come from
%   <module_library>/config/dimensions.yaml keyed by module_type.
%
%   Pipeline (A.4.0 pure symbolic):
%     DSL → ir.Expander (symbolic) → evaluateNumeric(config) → render
%
%   RESULT = MECHANISM(...) returns a struct with the mechanism
%   name, the computed global pose map (frame name -> 4x4), and the list of
%   any frames that could not be placed, useful for headless checking.
%
%   Example:
%     viz.mechanism('../../specs/dsl/examples/open-chain-2r/robot_description.yaml', ...
%                  '../../specs/dsl/examples/open-chain-2r/joint_config.yaml')
%
%   See also: +ir/Expander, +ir/EdgeGraph, +core/PosePropagator

    if nargin < 1 || isempty(dslYaml)
        error('viz:mechanism:usage', ...
            'Usage: viz.mechanism(dslYaml[, configYaml])');
    end
    if nargin < 2; configYaml = ''; end

    % ---- path setup ----
    here = fileparts(fileparts(mfilename('fullpath')));
    repoRoot = fileparts(fileparts(here));

    % ---- DSL → IR expansion ----
    expander = ir.Expander(dslYaml);

    % ---- numeric evaluation for rendering (substitute joint values from config) ----
    poses = expander.evaluateNumeric(configYaml);

    % ---- read expanded state into local variables ----
    mechName = expander.MechName;
    inst     = expander.Instances;
    connInfo = expander.ConnectionInfo;
    libDir   = expander.LibDir;
    nInst    = numel(inst);
    nConns   = numel(connInfo);

    % --- characteristic scale ---
    maxr = 1; ks = keys(poses);
    % compute the maximum distance from the origin to all frames
    for k = 1:numel(ks); P = poses(ks{k}); maxr = max(maxr, norm(P(1:3, 4))); end
    % L, the characteristic length, is used to control the size of the triads and joint axes in the visualization
    L = max(4, 0.20 * maxr);

    % --- figure ---
    fig = core.VizHelpers.newFigure(sprintf('Mechanism: %s', mechName));
    ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    view(ax, 135, 25); xlabel(ax, 'X (mm)'); ylabel(ax, 'Y (mm)'); zlabel(ax, 'Z (mm)');
    title(ax, sprintf('%s  —  %d instances, %d connections  (X=red Y=green Z=blue)', ...
        mechName, nInst, nConns), 'Interpreter', 'none');
    
    % draw the world frame triad at the origin
    core.VizHelpers.triad(ax, eye(4), L * 1.4, 2.5, '-');
    text(ax, 0, 0, 0, '  world', 'FontWeight', 'bold', 'Color', [.2 .2 .2]);

    % ---- render the mechanism frame (delegated to shared renderFrame) ----
    result.mechanism = mechName;
    result.poses = poses;
    unplaced = renderFrame(ax, poses, inst, connInfo, libDir, repoRoot, ...
        mechName, nInst, nConns, L, expander.JointValues);
    result.unplaced = unplaced;

    % ---- dropdown menu: select which instance's frames to show ----
    instNames = {inst.name};
    menuStr = ['<全部显示>', instNames];
    fig.UserData = struct('ax', ax, 'instNames', {instNames}, 'nInst', nInst);
    uicontrol('Style', 'popupmenu', ...
        'String', menuStr, ...
        'Value', 1, ...
        'Position', [20 20 180 25], ...
        'Callback', @(src, ~) toggleInstanceFrames(src, fig), ...
        'Parent', fig);

    rotate3d(ax, 'on');
end

%% local functions

function toggleInstanceFrames(src, fig)
%TOGGLEINSTANCEFRAMES  Dropdown callback: show frames for selected instance only.
%   idx=0 (first menu item) shows all; idx>0 shows only that instance's frames.
    ud = fig.UserData;
    idx = src.Value - 1;  % 0 = show all
    for ii = 1:ud.nInst
        h = findobj(ud.ax, 'Tag', ud.instNames{ii});
        if idx == 0 || idx == ii
            set(h, 'Visible', 'on');
        else
            set(h, 'Visible', 'off');
        end
    end
end




