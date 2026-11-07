classdef VizHelpers
    %VIZHELPERS  Geometry import & MATLAB figure-drawing helpers.
    %   All methods are static.  Covers light-themed figure creation, STL/STEP
    %   import dispatch, patch rendering with feature-edge overlay, and
    %   coordinate-triad drawing (RGB axes with arrowheads).

    methods (Static)

        %% create a figure locked to the light theme (white canvas)
        %   R2025a+ figures follow the desktop theme by default, which turns
        %   axes and labels dark in dark mode. Forcing Theme='light' keeps the
        %   hard-coded triad/label colours readable on the white background.
        %   Additional name-value pairs are forwarded to FIGURE.
        function fig = newFigure(name, varargin)
            fig = figure('Name', name, 'Color', 'w', varargin{:});
            if isprop(fig, 'Theme')
                fig.Theme = 'light';
            end
        end

        %% import geometry into surface patch data, preferring the referenced file
        function geom = importGeometry(geomPath)
            geom = [];
            [~, ~, ext] = fileparts(geomPath);
            if strcmpi(ext, '.stl')
                geom = core.VizHelpers.importStlGeometry(geomPath);
                return;
            end
            warning('viz_common:geometryUnsupported', ...
                'Unsupported geometry format for visualization: %s', geomPath);
        end

        %% import an STL mesh into Vertices/Faces patch data
        function geom = importStlGeometry(stlPath)
            geom = [];
            if exist('stlread', 'file') ~= 2
                warning('viz_common:stlImportUnavailable', ...
                    'Skipping STL geometry %s because stlread is unavailable in this MATLAB environment.', stlPath);
                return;
            end
            try
                mesh = stlread(stlPath);
                if isa(mesh, 'triangulation')
                    faces = mesh.ConnectivityList;
                    vertices = mesh.Points;
                elseif isstruct(mesh) && isfield(mesh, 'ConnectivityList') && isfield(mesh, 'Points')
                    faces = mesh.ConnectivityList;
                    vertices = mesh.Points;
                elseif isstruct(mesh) && isfield(mesh, 'faces') && isfield(mesh, 'vertices')
                    faces = mesh.faces;
                    vertices = mesh.vertices;
                else
                    error('Unsupported stlread output type: %s', class(mesh));
                end
                geom = struct('Vertices', vertices, 'Faces', faces);
            catch ME
                warning('viz_common:stlImportFailed', ...
                    'Failed to import STL geometry %s: %s', stlPath, ME.message);
            end
        end

        %% draw imported geometry at the origin of the given transformation matrix T
        function patchGeometry(ax, T, geom, color, alpha)
            Vw = (T(1:3,1:3) * geom.Vertices.' + T(1:3,4)).';
            patch(ax, 'Vertices', Vw, 'Faces', geom.Faces, 'FaceColor', color, ...
                'FaceAlpha', alpha, 'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
                'AmbientStrength', 0.35, 'DiffuseStrength', 0.75, 'SpecularStrength', 0.05);
            core.VizHelpers.drawFeatureEdges(ax, Vw, geom.Faces, [0.75 0.75 0.75], 0.75);
        end

        %% draw only sharp feature edges so STL triangle mesh lines stay hidden
        function drawFeatureEdges(ax, vertices, faces, color, lineWidth)
            if size(faces, 2) ~= 3
                return;
            end
            try
                tr = triangulation(faces, vertices);
                edgePairs = featureEdges(tr, deg2rad(20));
            catch
                edgePairs = [];
            end
            if isempty(edgePairs)
                return;
            end
            X = [vertices(edgePairs(:,1),1) vertices(edgePairs(:,2),1) nan(size(edgePairs,1),1)].';
            Y = [vertices(edgePairs(:,1),2) vertices(edgePairs(:,2),2) nan(size(edgePairs,1),1)].';
            Z = [vertices(edgePairs(:,1),3) vertices(edgePairs(:,2),3) nan(size(edgePairs,1),1)].';
            line(ax, X(:), Y(:), Z(:), 'Color', color, 'LineWidth', lineWidth);
        end

        %% color per module type (RGB)
        function c = typeColor(type)
            switch type
                case 'Frame';        c = [0.30 0.55 0.85];
                case 'Joint';        c = [0.85 0.45 0.25];
                case 'ToolPipette'; c = [0.35 0.70 0.40];
                case 'Manipulator';  c = [0.60 0.40 0.75];
                case 'Adaptor_45';   c = [0.75 0.65 0.25];
                case 'Pin';          c = [0.50 0.50 0.50];
                otherwise;           c = [0.45 0.45 0.45];
            end
        end

        %% draw a frame origin marker (filled circle + label)
        %   isExposed=true  → black, bold; false → gray, normal weight
        function frameMarker(ax, T, label, isExposed)
            if isExposed
                fc = [0 0 0]; fw = 'bold';
            else
                fc = [0.45 0.45 0.45]; fw = 'normal';
            end
            plot3(ax, T(1,4), T(2,4), T(3,4), 'o', 'MarkerSize', 5, ...
                'MarkerFaceColor', fc, 'MarkerEdgeColor', fc);
            text(ax, T(1,4), T(2,4), T(3,4), ['  ' label], 'Color', fc, ...
                'FontWeight', fw, 'Interpreter', 'none', 'FontSize', 8);
        end

        %% draw a joint axis line with label
        %   kind: 'revolute' → red; 'prismatic' → blue
        function jointAxis(ax, T, axisDir, L, kind, label)
            o = T(1:3, 4); d = T(1:3, 1:3) * axisDir(:);
            if strcmpi(kind, 'prismatic')
                jc = [0.1 0.1 0.9];
            else
                jc = [0.9 0.1 0.1];
            end
            p1 = o - d * L * 1.3; p2 = o + d * L * 1.3;
            line(ax, [p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], ...
                'Color', jc, 'LineWidth', 2.5, 'LineStyle', ':');
            text(ax, p2(1), p2(2), p2(3), ['  ' label], 'Color', jc, ...
                'FontSize', 8, 'Interpreter', 'none');
        end

        %% draw a local triad (coordinate frame) at the origin of T
        %   X=red, Y=green, Z=blue, with arrowheads
        function triad(ax, T, L, lw, ls)
            o = T(1:3,4);
            cols = {[0.85 0 0], [0 0.65 0], [0 0 0.9]};
            for a = 1:3
                d = T(1:3, a);
                p1 = o + L * d;
                plot3(ax, [o(1) p1(1)], [o(2) p1(2)], [o(3) p1(3)], ...
                    'Color', cols{a}, 'LineWidth', lw, 'LineStyle', ls);
                headL = L * 0.15;
                headW = L * 0.05;
                if headL > 0
                    if abs(d(3)) < 0.9
                        u = cross([0; 0; 1], d);
                    else
                        u = cross([1; 0; 0], d);
                    end
                    u = u / norm(u);
                    v = cross(d, u);
                    pb = p1 - headL * d;
                    p1b1 = pb + headW * u;
                    p1b2 = pb - headW * u;
                    p1b3 = pb + headW * v;
                    p1b4 = pb - headW * v;
                    hX = [p1(1) p1b1(1) NaN p1(1) p1b2(1) NaN p1(1) p1b3(1) NaN p1(1) p1b4(1)];
                    hY = [p1(2) p1b1(2) NaN p1(2) p1b2(2) NaN p1(2) p1b3(2) NaN p1(2) p1b4(2)];
                    hZ = [p1(3) p1b1(3) NaN p1(3) p1b2(3) NaN p1(3) p1b3(3) NaN p1(3) p1b4(3)];
                    line(ax, hX, hY, hZ, 'Color', cols{a}, 'LineWidth', lw, 'LineStyle', '-');
                end
            end
        end

    end
end
