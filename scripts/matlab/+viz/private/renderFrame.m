function unplaced = renderFrame(ax, poses, inst, connInfo, libDir, repoRoot, ...
    mechName, nInst, nConns, L, jointValues, showDetails, geomCache)
%RENDERFRAME  Render one frame of a mechanism assembly onto axes AX.
%   Shared rendering logic used by both viz.mechanism (static, with triads/mates)
%   and viz.animate (per-frame redraw, bodies only).  Clears previous dynamic
%   content then redraws.
%
%   UNPLACED = renderFrame(AX, POSES, INST, CONNINFO, LIBDIR, REPOROOT, ...
%                          MECHNAME, NINST, NCONNS, L, JOINTVALUES)
%   UNPLACED = renderFrame(..., SHOWDETAILS)
%   UNPLACED = renderFrame(..., SHOWDETAILS, GEOMCACHE)
%     GEOMCACHE : containers.Map — geometry path → pre-loaded geom struct.
%                 When non-empty, bypasses disk I/O for STL import.
%
%   See also: +viz/mechanism, +viz/animate

    if ~exist('showDetails', 'var')
        showDetails = true;
    end
    if ~exist('geomCache', 'var')
        geomCache = [];
    end

    % ---- clear previous dynamic content (keep world triad / axis labels) ----
    delete(findobj(ax, 'Tag', 'vizDynamic'));

    unplaced = {};

    if showDetails
        fprintf('\n=== mechanism: %s (%d instances, %d connections) ===\n', ...
            mechName, nInst, nConns);
    end

    % ---- bodies per instance ----
    for i = 1:nInst
        col = core.VizHelpers.typeColor(inst(i).type);

        if showDetails
            fprintf('\n-- instance %s [%s] --\n', inst(i).name, inst(i).type);
        end

        % snapshot before body patches (for tagging in animation mode)
        preBodyKids = allchild(ax);

        % pass 1: body geometry patches (always drawn)
        for k = 1:numel(inst(i).bodies)
            b = inst(i).bodies{k};
            if ~isKey(poses, b.node)
                unplaced{end+1} = b.node; %#ok<AGROW>
                continue;
            end
            Tb = poses(b.node);
            geomPath = core.PathUtils.resolveGeometryPath(b.geometry, libDir, repoRoot);
            drewGeom = false;
            if ~isempty(b.geometry) && ~isempty(geomPath)
                % use cached geometry when available (animation mode);
                % otherwise import from disk (static viz mode)
                if ~isempty(geomCache) && isKey(geomCache, b.geometry)
                    geom = geomCache(b.geometry);
                else
                    geom = core.VizHelpers.importGeometry(geomPath);
                end
                if ~isempty(geom)
                    core.VizHelpers.patchGeometry(ax, Tb, geom, col, 0.8);
                    drewGeom = true;
                end
            end
            if ~drewGeom
                % fallback for bodies without (resolvable) geometry — e.g. the
                % Manipulator drive stages: translucent box so the body stays
                % visible in both static and animation rendering.
                hs = 0.30 * L;   % half-size [mm], scaled by characteristic length
                V = hs * [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; ...
                          -1 -1  1; 1 -1  1; 1 1  1; -1 1  1];
                F = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
                Vw = (Tb(1:3, 1:3) * V.' + Tb(1:3, 4)).';
                patch(ax, 'Vertices', Vw, 'Faces', F, ...
                    'FaceColor', col, 'FaceAlpha', 0.35, ...
                    'EdgeColor', col * 0.7, 'EdgeAlpha', 0.9);
            end
        end

        % tag body patches for clearing on next frame
        postBodyKids = allchild(ax);
        newBodyKids = postBodyKids(~ismember(postBodyKids, preBodyKids));
        for hk = 1:numel(newBodyKids)
            set(newBodyKids(hk), 'Tag', 'vizDynamic');
        end

        if ~showDetails
            continue;  % skip triads, frames, joints, mates in animation mode
        end

        % ---- detail rendering below (static viz only) ----

        preKids = allchild(ax);

        % pass 2: body triads
        for k = 1:numel(inst(i).bodies)
            b = inst(i).bodies{k};
            if ~isKey(poses, b.node); continue; end
            core.VizHelpers.triad(ax, poses(b.node), L, 1.2, '-');
        end

        % frames: triads + markers
        for k = 1:numel(inst(i).frames)
            f = inst(i).frames{k};
            if ~isKey(poses, f.node)
                fprintf('  [UNPLACED] %-22s\n', f.node);
                unplaced{end+1} = f.node; %#ok<AGROW>
                continue;
            end
            T = poses(f.node);
            if f.exposed
                lw = 2.0; sty = '-'; mk = 'PORT';
            else
                lw = 1.0; sty = '--'; mk = 'frame';
            end
            core.VizHelpers.triad(ax, T, L, lw, sty);
            core.VizHelpers.frameMarker(ax, T, f.node, f.exposed);
            fprintf('  %-7s %-22s pos=[% 7.2f % 7.2f % 7.2f]  +Z=[% .2f % .2f % .2f]\n', ...
                mk, f.node, T(1,4), T(2,4), T(3,4), T(1,3), T(2,3), T(3,3));
        end

        % joint axes
        for k = 1:numel(inst(i).joints)
            j = inst(i).joints{k};
            if ~isKey(poses, j.node); continue; end
            jKey = [inst(i).name '.' j.var];
            if ~isempty(jointValues) && isKey(jointValues, jKey)
                jVal = jointValues(jKey);
            else
                jVal = 0;
            end
            core.VizHelpers.jointAxis(ax, poses(j.node), j.axis, L, j.kind, ...
                sprintf('%s.%s=%.3g', inst(i).name, j.var, jVal));
        end

        % tag all new frame-related children
        postKids = allchild(ax);
        newKids = postKids(~ismember(postKids, preKids));
        set(newKids, 'Tag', inst(i).name);
    end

    if showDetails
        % tag remaining dynamic content
        hAll = allchild(ax);
        for hi = 1:numel(hAll)
            if isempty(get(hAll(hi), 'Tag'))
                set(hAll(hi), 'Tag', 'vizDynamic');
            end
        end

        % ---- mate diagnostics ----
        fprintf('\n-- mate checks --\n');
        for c = 1:numel(connInfo)
            ci = connInfo(c);
            if ~isKey(poses, ci.socketNode) || ~isKey(poses, ci.plugNode)
                fprintf('  [UNPLACED MATE] %s\n', ci.label); continue;
            end
            Ps = poses(ci.socketNode); Pp = poses(ci.plugNode);
            gap = norm(Ps(1:3,4) - Pp(1:3,4));
            zdot = dot(Ps(1:3,3), Pp(1:3,3));
            if ci.closed; lc = [0.95 0.55 0.10]; lw = 3.0; else; lc = [0.2 0.2 0.2]; lw = 1.5; end
            hLine = line(ax, [Ps(1,4) Pp(1,4)], [Ps(2,4) Pp(2,4)], [Ps(3,4) Pp(3,4)], ...
                'Color', lc, 'LineWidth', lw, 'LineStyle', '--');
            set(hLine, 'Tag', 'vizDynamic');
            fprintf('  %-40s gap=%.3e  Zdot=% .4f%s\n', ci.label, gap, zdot, ...
                core.CommonUtils.tern(ci.closed, '  [closed]', ''));
        end

        if ~isempty(unplaced)
            fprintf('\n  [WARNING] %d node(s) not placed (disconnected component?).\n', ...
                numel(unplaced));
        end
    end
end
