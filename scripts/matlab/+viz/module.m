function result = module(moduleYaml, configYaml)
%MODULE  Validate & visualize one L1 module definition.
%   MODULE(MODULEYAML) parses a module definition from
%   specs/modules/*.yaml, builds its internal frame graph (body center
%   frames + ports), evaluates all fixedTransform / joint edges, and draws:
%     - each body as imported STEP geometry when available
%     - each frame/port as an RGB coordinate triad (X=red, Y=green, Z=blue)
%
%   MODULE(MODULEYAML, CONFIGYAML) also injects numeric parameter
%   values (e.g. cubeLength, tipDistance) and joint variable values from a
%   config YAML keyed by module_type, so symbolic translations like
%   'cubeLength/2' and the revolute angle 'q' resolve to numbers.
%
%   RESULT = MODULE(...) returns a struct with the computed global
%   pose of every frame (4x4), useful for headless checking.
%
%   Example:
%     viz.module('../../specs/modules/frame.yaml', 'module_viz_config.yaml')
%
%   Rotation conventions follow specs/modeling-conventions.md:
%     - align{a,b}: rule 's -> d' means child-frame axis 's' equals parent
%       axis 'd'; R_child_in_parent = DST*SRC' (third axis by right-hand rule).
%     - rpy = [Rx,Ry,Rz] intrinsic Z-Y-X: R = Rz*Ry*Rx.
%     - axis_angle = Rodrigues(omega, q).

    if nargin < 1 || isempty(moduleYaml)
        error('viz:module:usage', ...
            'Usage: viz.module(moduleYaml[, configYaml])');
    end
    if nargin < 2; configYaml = ''; end

    % resolve a relative path against the current script's directory
    here = fileparts(fileparts(mfilename('fullpath')));
    moduleYaml = core.PathUtils.resolve(moduleYaml, here);

    m = core.readYaml(moduleYaml);

    % -- validate module definition ---
    assert(isfield(m, 'module_type'), 'Not a module definition: %s', moduleYaml);

    % --- parameter injection from config (keyed by module_type) ---
    params = struct();
    if ~isempty(configYaml)
        configYaml = core.PathUtils.resolve(configYaml, here);
        if exist(configYaml, 'file')
            cfg = core.readYaml(configYaml);

            mtf = m.module_type;

            % check if the config has a field for this module_type and it's a struct
            if isfield(cfg, mtf) && isstruct(cfg.(mtf))
                % inject the parameters from the config into the params struct
                params = cfg.(mtf);
            end
        end
    end

    % --- build edges & node poses ---
    bodies = core.CommonUtils.asList(core.CommonUtils.field(m, 'bodies', {}));
    frames = core.CommonUtils.asList(core.CommonUtils.field(m, 'frames', {}));
    fts    = core.CommonUtils.asList(core.CommonUtils.field(m, 'fixed_transforms', {}));
    jts    = core.CommonUtils.asList(core.CommonUtils.field(m, 'joints', {}));

    % --- sanity check ---
    assert(~isempty(bodies), 'Module %s declares no bodies.', m.module_type);

    % shared pose-graph accumulator (see ARCHITECTURE.md for layer design)
    g = ir.EdgeGraph();

    % -- build edges from fixed transforms ---
    for k = 1:numel(fts)
        t = fts{k};
        tr = core.CommonUtils.evalVec(t.translation, params);
        R = core.RigidBodyMath.rot(t.rotation, params);
        T = core.RigidBodyMath.T(R, tr);
        g.addFixedTransform(t.from_frame, t.to_frame, T);
    end

    % -- build edges from joints ---
    for k = 1:numel(jts)
        j = jts{k};
        ax = core.CommonUtils.evalVec(j.axis, params);
        qv = core.CommonUtils.field(params, j.variable, 0);
        kind = core.CommonUtils.field(j, 'kind', 'revolute');
        g.addJoint(j.from_frame, j.to_frame, ax, qv, kind);
    end

    % -- pose computing using forward kinematics --
    rootBody = bodies{1}.name;
    g.addRoot(rootBody);
    poses = g.propagate();

    moduleDir = fileparts(moduleYaml);
    repoRoot = fileparts(fileparts(here));

    % --- characteristic scale ---
    maxr = 1;

    % get the name of each pose
    ks = keys(poses);

    for k = 1:numel(ks)
        % get the pose matrix for the current key
        P = poses(ks{k});

        % compute the maximum distance from the origin to all frames
        maxr = max(maxr, norm(P(1:3, 4)));
    end

    % adjust the maximum distance based on the cube length parameter, if provided
    if isfield(params, 'cubeLength'); maxr = max(maxr, params.cubeLength/2); end
    L = max(4, 0.30 * maxr);          % triad axis length

    % --- figure ---
    fig = core.VizHelpers.newFigure(sprintf('Module: %s', m.module_type));
    ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    view(ax, 135, 25); xlabel(ax, 'X (mm)'); ylabel(ax, 'Y (mm)'); zlabel(ax, 'Z (mm)');

    % world frame
    core.VizHelpers.triad(ax, eye(4), L*1.3, 2.5, '-');
    text(ax, 0, 0, 0, '  world', 'FontWeight', 'bold', 'Color', [.2 .2 .2]);

    % bodies as imported geometry when available
    for k = 1:numel(bodies)
        b = bodies{k};
        if ~isKey(poses, b.name); continue; end
        Tb = poses(b.name);
        geomSpec = core.CommonUtils.field(b, 'geometry', '');

        geomPath = core.PathUtils.resolveGeometryPath(geomSpec, moduleDir, repoRoot);
        if isempty(geomSpec)
            % No body geometry: keep triads/labels only so pose checks still work.
        elseif isempty(geomPath)
            warning('viz:module:geometryMissing', ...
                'Body "%s" geometry file not found: %s', b.name, b.geometry);
        else
            geom = core.VizHelpers.importGeometry(geomPath);
            if ~isempty(geom)
                core.VizHelpers.patchGeometry(ax, Tb, geom, [0.30 0.55 0.85], 0.12);
            end
        end
        core.VizHelpers.triad(ax, Tb, L, 1.5, '-');
        text(ax, Tb(1,4), Tb(2,4), Tb(3,4), ...
            ['  ' b.name], 'FontAngle', 'italic', 'Color', [0.1 0.2 0.5]);
    end

    % port / frame triads
    result.module_type = m.module_type;
    result.frames = struct();
    fprintf('\n=== %s (%d DOF, %s) ===\n', m.module_type, ...
        core.CommonUtils.field(m, 'dof', 0), core.CommonUtils.field(m, 'extraction_status', 'n/a'));

    % get the names of all bodies for later checking of unreached bodies, where @(b) b.name is an anonymous function that extracts the 'name' field from each body struct, and 'UniformOutput', false allows the output to be a cell array of names
    bodyNames = cellfun(@(b) b.name, bodies, 'UniformOutput', false);

    for k = 1:numel(frames)
        f = frames{k};

        if ~isKey(poses, f.name)
            fprintf('  [UNPLACED] %-16s (not reachable from root)\n', f.name);
            continue;
        end

        T = poses(f.name);
        isPort = isfield(f, 'exposed') && isequal(f.exposed, true);
        
        if isPort
            color = [0 0 0]; lw = 2.0; mk = 'PORT';
        else
            color = [0.4 0.4 0.4]; lw = 1.0; mk = 'frame';
        end

        core.VizHelpers.triad(ax, T, L, lw, core.CommonUtils.tern(isPort, '-', '--'));
        plot3(ax, T(1,4), T(2,4), T(3,4), 'o', 'MarkerSize', 5, ...
            'MarkerFaceColor', color, 'MarkerEdgeColor', color);
        text(ax, T(1,4), T(2,4), T(3,4), ['  ' f.name], 'Color', color, ...
            'FontWeight', core.CommonUtils.tern(isPort, 'bold', 'normal'));
        result.frames.(f.name) = T;
        fprintf('  %-7s %-16s pos=[% 7.2f % 7.2f % 7.2f]  +Z=[% .2f % .2f % .2f]\n', ...
            mk, f.name, T(1,4), T(2,4), T(3,4), T(1,3), T(2,3), T(3,3));
    end

    % unreached bodies note
    for k = 1:numel(bodyNames)
        if ~isKey(poses, bodyNames{k})
            fprintf('  [UNPLACED BODY] %s\n', bodyNames{k});
        end
    end

    title(ax, sprintf('%s  —  %d DOF, %s  (X=red Y=green Z=blue)', ...
        m.module_type, core.CommonUtils.field(m, 'dof', 0), ...
        core.CommonUtils.field(m, 'extraction_status', 'n/a')), 'Interpreter', 'none');
    rotate3d(ax, 'on');
end
