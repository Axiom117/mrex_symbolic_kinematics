function scene = sceneSpec(expander)
%SCENESPEC  Static scene description for pose-sequence rendering.
%   SCENE = viz.sceneSpec(EXPANDER)
%
%   Extracts the configuration-independent data needed by viz.play /
%   renderFrame from an expanded mechanism, and pre-loads body geometry
%   into a cache (avoids per-frame disk I/O):
%
%     SCENE.MechName       char
%     SCENE.Instances      struct array — instance list (bodies/frames/joints)
%     SCENE.ConnectionInfo struct array — connection list (mate diagnostics)
%     SCENE.LibDir         char — module library directory
%     SCENE.RepoRoot       char — repository root (geometry path resolution)
%     SCENE.NInst / .NConns double
%     SCENE.L            double — characteristic scale (from the zero pose)
%     SCENE.GeomCache    containers.Map — geometry path → imported geometry
%
%   The scene is pure static data: viz.play pairs it with a PoseSequence
%   (solver.buildPoseSequence) to render the mechanism's motion.
%
%   See also: viz.play, viz.animate, solver.buildPoseSequence, +ir/Expander

    arguments
        expander (1,1) ir.Expander
    end

    here = fileparts(fileparts(mfilename('fullpath')));
    repoRoot = fileparts(fileparts(here));

    inst     = expander.Instances;
    connInfo = expander.ConnectionInfo;
    nInst    = numel(inst);
    nConns   = numel(connInfo);

    % ---- characteristic scale (from the zero pose) ----
    zeroPoses = expander.evaluateNumericDirect(containers.Map);
    maxr = 1; ks = keys(zeroPoses);
    for k = 1:numel(ks); P = zeroPoses(ks{k}); maxr = max(maxr, norm(P(1:3,4))); end
    L = max(4, 0.20 * maxr);

    % ---- pre-load all geometry (avoid per-frame disk I/O) ----
    geomCache = containers.Map('KeyType', 'char', 'ValueType', 'any');
    for i = 1:nInst
        for k = 1:numel(inst(i).bodies)
            gPath = inst(i).bodies{k}.geometry;
            if ~isempty(gPath) && ~isKey(geomCache, gPath)
                geomAbs = core.PathUtils.resolveGeometryPath(gPath, expander.LibDir, repoRoot);
                if ~isempty(geomAbs)
                    g = core.VizHelpers.importGeometry(geomAbs);
                    if ~isempty(g)
                        geomCache(gPath) = g;
                    end
                end
            end
        end
    end
    fprintf('  Geometry cache: %d unique STL files loaded\n', geomCache.Count);

    scene = struct( ...
        'MechName',       expander.MechName, ...
        'Instances',      {inst}, ...
        'ConnectionInfo', {connInfo}, ...
        'LibDir',         expander.LibDir, ...
        'RepoRoot',       repoRoot, ...
        'NInst',          nInst, ...
        'NConns',         nConns, ...
        'L',              L, ...
        'GeomCache',      {geomCache});
end
