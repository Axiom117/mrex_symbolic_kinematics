function animate(dslYaml, jointTrajectory, varNames, fps, varargin)
%ANIMATE  Animate a modular mechanism through a sequence of joint configurations.
%   Layered wrapper (A.5.2.0) — the renderer holds no kinematics:
%       expander = ir.Expander(dslYaml);            % static scene + FK engine
%       seq      = solver.buildPoseSequence(...);   % compute layer: Q → 位姿序列
%       scene    = viz.sceneSpec(expander);         % static scene snapshot
%       viz.play(seq, scene, ...);                  % pure renderer
%   See viz.play / solver.buildPoseSequence / viz.sceneSpec for the split API.
%
%   viz.animate(DSLYAML, JOINTTRAJECTORY, VARNAMES) renders one frame per row
%   of JOINTTRAJECTORY (nFrames×nJoints), updating the figure in-place.
%   VARNAMES is a cell array of canonical joint variable names (e.g.
%   {'joint_AB.q', 'joint_BD.q', ...}) matching the columns of JOINTTRAJECTORY.
%
%   viz.animate(DSLYAML, JOINTTRAJECTORY, VARNAMES, FPS) sets the playback speed
%   (default 10 fps).
%
%   viz.animate(..., 'Anchor', MODE) selects the display reference frame:
%     'root'  (default) — tool-rooted FK poses rendered as-is (root pinned).
%     'world' — re-express every pose in the world frame via
%               E(q) = T_bind * inv(T_tip<-ground_ref(q)), so that for L3
%               world-closed-loop mechanisms (e.g. mrex-3t1r_001) the Manipulator
%               ground frames stay fixed at their calibrated world bindings
%               while the tool visibly travels.  Requires frames with
%               semantic_tag == 'ground'.  T_bind is taken from the zero-pose
%               FK — the same value as solver.ClosureSolver's auto-calibrated
%               world binding (see test_closure_solver_m_rex_3t1r check [5]).
%               Extra ground frames are reported as a branch-consistency
%               diagnostic (max gap printed at setup).
%
%   viz.animate(..., 'ShowTrail', TF) toggles the tip world-path overlay
%   (full path as a faint dashed line, traversed segment highlighted, current
%   tip marked).  Only meaningful in 'world' anchor mode; default true.
%
%   viz.animate(..., 'View', [AZ EL]) sets the fixed camera orientation
%   (degrees, default [135 25]); 'FixedView', TF (default true) freezes the
%   axes limits/camera during playback so the view does not follow the
%   mechanism (see viz.play).
%
%   Playback controls (buttons at bottom of figure):
%     ▶ Play / ⏸ Pause    — toggle playback
%     ⏮ Reset             — jump to first frame
%     Speed slider         — adjust fps (1–60)
%     Frame slider         — scrub to any frame
%
%   Example — animate a closed-loop sweep of the parallelogram:
%       nFrames = 60;
%       theta = linspace(0, pi/3, nFrames);
%       Q = [theta; -theta; theta; -theta]';  % nFrames×4
%       varNames = {'joint_AB.q','joint_BD.q','joint_CD.q','joint_AC.q'};
%       viz.animate('../../specs/dsl/cases/single-closed-loop/robot_description.yaml', Q, varNames);
%
%   See also: solver.buildPoseSequence, viz.play, viz.sceneSpec, +ir/Expander

    if nargin < 1 || isempty(dslYaml)
        error('viz:animate:usage', ...
            'Usage: viz.animate(dslYaml, jointTrajectory, varNames[, fps][, Name, Value...])');
    end
    if nargin < 2; jointTrajectory = []; end
    if nargin < 3; varNames = {}; end
    if nargin < 4 || isempty(fps); fps = 10; end

    % ---- parse name-value options ----
    opts = struct('Anchor', 'root', 'ShowTrail', true, ...
        'View', [135 25], 'FixedView', true);
    if ~isempty(varargin)
        assert(mod(numel(varargin), 2) == 0, ...
            'viz:animate:badOptions', 'Name-value options must come in pairs.');
        for oi = 1:2:numel(varargin)
            oName = varargin{oi};
            oVal  = varargin{oi + 1};
            switch lower(oName)
                case 'anchor'
                    assert(ischar(oVal) || (isstring(oVal) && isscalar(oVal)), ...
                        'viz:animate:badAnchor', 'Anchor must be a char value.');
                    assert(any(strcmp(oVal, {'root', 'world'})), ...
                        'viz:animate:badAnchor', ...
                        'Anchor must be ''root'' or ''world'', got ''%s''.', char(oVal));
                    opts.Anchor = char(oVal);
                case 'showtrail'
                    assert(isscalar(oVal) && (islogical(oVal) || isnumeric(oVal)), ...
                        'viz:animate:badShowTrail', 'ShowTrail must be a logical scalar.');
                    opts.ShowTrail = logical(oVal);
                case 'view'
                    assert(isnumeric(oVal) && numel(oVal) == 2 ...
                        && all(isfinite(oVal(:))), 'viz:animate:badView', ...
                        'View must be a finite [azimuth elevation] pair (degrees).');
                    opts.View = double(oVal(:).');
                case 'fixedview'
                    assert(isscalar(oVal) && (islogical(oVal) || isnumeric(oVal)), ...
                        'viz:animate:badFixedView', ...
                        'FixedView must be a logical scalar.');
                    opts.FixedView = logical(oVal);
                otherwise
                    error('viz:animate:badOption', 'Unknown option ''%s''.', oName);
            end
        end
    end

    % ---- validate trajectory (before the expensive pipeline build) ----
    if isempty(jointTrajectory) || isempty(varNames)
        error('viz:animate:noTrajectory', ...
            'jointTrajectory and varNames are required for animation.');
    end

    % ---- build the symbolic pipeline (one-time; same FK engine as the solvers) ----
    fprintf('Building symbolic pipeline (one-time) ... ');
    expander = ir.Expander(dslYaml);
    fprintf('OK (%d instances, %d joints)\n', ...
        numel(expander.Instances), expander.JointVarMap.Count);
    expander.compilePoseFunctions();

    % ---- compute layer: joint trajectory → per-frame poses (solver product) ----
    seq = solver.buildPoseSequence(expander, jointTrajectory, varNames, ...
        'Anchor', opts.Anchor);

    % ---- render layer: static scene snapshot + pure playback ----
    scene = viz.sceneSpec(expander);
    viz.play(seq, scene, 'Fps', fps, 'ShowTrail', opts.ShowTrail, ...
        'View', opts.View, 'FixedView', opts.FixedView);
end




