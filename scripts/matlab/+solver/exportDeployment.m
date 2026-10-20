function out = exportDeployment(pkgDir, varargin)
%EXPORTDEPLOYMENT  Package a generated model package for consumer deployment.
%   OUT = solver.exportDeployment(PKGDIR)
%   OUT = solver.exportDeployment(PKGDIR, 'OutputDir', DIR)
%
%   Reads an existing package written by solver.exportKinematics and writes
%   a self-contained deployment layout next to it (or under OutputDir):
%
%       +<mechanism>/                 namespaced numeric kernels
%           closure_residual.m
%           closure_residual_jacobian.m
%           fk_tool.m
%           fk_all.m
%           fk_all_matrix.m
%       +smk/KinematicsRuntime.m      self-contained generic runtime class
%       IK_<mechanism>.m              top-level IK wrapper (when supported)
%       FK_<mechanism>.m              top-level FK wrapper
%       runtime_manifest.json         machine-readable deployment manifest
%
%   Consumers then only need:
%       addpath(<packageDir>);
%       rt = smk.KinematicsRuntime(<packageDir>);
%       T  = rt.fk(q, Urel);
%       [q, ok] = rt.ik(T, Urel, q0);
%       % or the generated wrappers:
%       [q, ok] = IK_<mechanism>(T, Urel, q0);
%       T       = FK_<mechanism>(q, Urel);
%
%   The namespaced +<mechanism> package avoids the global function-name
%   collisions of the raw generic kernels when several models are on the
%   MATLAB path.
%
%   This function uses base MATLAB only (no Symbolic Math Toolbox, no
%   Expander/ClosureSolver).
%
%   Output OUT:
%     .OutputDir : char
%     .Files     : cell of absolute paths written
%     .Manifest  : struct serialized to runtime_manifest.json
%
%   See also: solver.exportKinematics, solver.KinematicsRuntime

    outputDir = '';
    if ~isempty(varargin)
        assert(mod(numel(varargin), 2) == 0, ...
            'solver:exportDeployment:badOptions', ...
            'Name-value options must come in pairs.');
        for oi = 1:2:numel(varargin)
            switch lower(char(varargin{oi}))
                case 'outputdir'
                    outputDir = char(varargin{oi + 1});
                otherwise
                    error('solver:exportDeployment:badOption', ...
                        'Unknown option ''%s''.', char(varargin{oi}));
            end
        end
    end

    pkgDir = char(pkgDir);
    assert(exist(pkgDir, 'dir') == 7, ...
        'solver:exportDeployment:packageNotFound', ...
        'Package directory not found: %s', pkgDir);
    metaFile = fullfile(pkgDir, 'kinematic_metadata.json');
    assert(exist(metaFile, 'file') == 2, ...
        'solver:exportDeployment:metadataNotFound', ...
        '%s not found.', metaFile);
    md = jsondecode(fileread(metaFile));

    if isempty(outputDir)
        outputDir = pkgDir;
    elseif ~core.PathUtils.isAbsolutePath(outputDir)
        outputDir = fullfile(pwd, outputDir);
    end
    if exist(outputDir, 'dir') ~= 7
        mkdir(outputDir);
    end

    pkgName = matlab.lang.makeValidName(char(md.mechanism));
    kernelDir = fullfile(outputDir, ['+' pkgName]);
    runtimeDir = fullfile(outputDir, '+smk');
    if exist(kernelDir, 'dir') ~= 7
        mkdir(kernelDir);
    end
    if exist(runtimeDir, 'dir') ~= 7
        mkdir(runtimeDir);
    end

    files = {};
    kernelNames = {'closure_residual.m', 'closure_residual_jacobian.m', ...
        'fk_tool.m', 'fk_all.m', 'fk_all_matrix.m'};
    for k = 1:numel(kernelNames)
        src = fullfile(pkgDir, kernelNames{k});
        if exist(src, 'file') ~= 2
            continue;
        end
        dst = fullfile(kernelDir, kernelNames{k});
        if strcmp(kernelNames{k}, 'fk_all.m')
            % Make the generated wrapper call its namespaced sibling.
            txt = fileread(src);
            txt = strrep(txt, 'raw = fk_all_matrix(q);', ...
                sprintf('raw = %s.fk_all_matrix(q);', pkgName));
            fid = fopen(dst, 'w', 'n', 'UTF-8');
            assert(fid > 0, 'solver:exportDeployment:writeFailed', ...
                'Cannot write %s.', dst);
            fwrite(fid, txt, 'char');
            fclose(fid);
        else
            copyfile(src, dst);
        end
        files{end+1} = dst; %#ok<AGROW>
    end

    runtimeSrc = fullfile(fileparts(mfilename('fullpath')), ...
        'KinematicsRuntime.m');
    runtimeDst = fullfile(runtimeDir, 'KinematicsRuntime.m');
    copyfile(runtimeSrc, runtimeDst);
    files{end+1} = runtimeDst;

    hasIK = isfield(md, 'runtime') && isfield(md.runtime, 'supports') && ...
        any(strcmp(cellstr(md.runtime.supports), 'ik'));
    [ikFile, fkFile] = localWriteWrappers(outputDir, pkgName, hasIK);
    if ~isempty(ikFile)
        files{end+1} = ikFile; %#ok<AGROW>
    end
    files{end+1} = fkFile; %#ok<AGROW>

    % A consumer pointed at OutputDir needs the metadata contract there too.
    metaDst = fullfile(outputDir, 'kinematic_metadata.json');
    if ~strcmp(metaDst, metaFile)
        copyfile(metaFile, metaDst);
        files{end+1} = metaDst; %#ok<AGROW>
    end

    manifest = struct();
    manifest.package_format = 'smk-kinematic-runtime';
    manifest.package_version = '0.1.0';
    manifest.mechanism = char(md.mechanism);
    manifest.source_package = pkgDir;
    manifest.kernel_package = pkgName;
    manifest.runtime_class = 'smk.KinematicsRuntime';
    manifest.api_version = '0.1.0';
    manifest.units = md.units;
    if isfield(md.inputs, 'actuated_indices')
        manifest.actuated_indices = double(md.inputs.actuated_indices);
        manifest.passive_indices = double(md.inputs.passive_indices);
    else
        manifest.actuated_indices = [];
        manifest.passive_indices = [];
    end
    if isfield(md.inputs, 'lower') && ~isempty(md.inputs.lower)
        manifest.lower = double(md.inputs.lower(:).');
        manifest.upper = double(md.inputs.upper(:).');
    else
        manifest.lower = [];
        manifest.upper = [];
    end
    if isfield(md, 'runtime') && isfield(md.runtime, 'supports') && ...
            ~isempty(md.runtime.supports)
        manifest.functions = cellstr(md.runtime.supports);
    else
        manifest.functions = {'fkPackage', 'fk', 'ikPackage', 'ik', ...
            'fkDrivers', 'ikDrivers', 'ikTrajectory'};
    end
    if hasIK
        manifest.wrapper_ik = ['IK_' pkgName];
    else
        manifest.wrapper_ik = '';
    end
    manifest.wrapper_fk = ['FK_' pkgName];
    manifest.wrapper_contract = ['T is the fixed-world tool pose (mm / rad); ' ...
        'Urel = T_tip<-tool; q is in metadata.inputs.names order (mm / rad).'];

    manifestFile = fullfile(outputDir, 'runtime_manifest.json');
    fid = fopen(manifestFile, 'w', 'n', 'UTF-8');
    assert(fid > 0, 'solver:exportDeployment:writeFailed', ...
        'Cannot write %s.', manifestFile);
    fwrite(fid, jsonencode(manifest, 'PrettyPrint', true), 'char');
    fclose(fid);
    files{end+1} = manifestFile;

    out = struct();
    out.OutputDir = outputDir;
    out.Files = files;
    out.Manifest = manifest;
end

%% ---- local functions ----

function [ikFile, fkFile] = localWriteWrappers(outputDir, pkgName, hasIK)
%LOCALWRITEWRAPPERS  Emit model-specific top-level IK/FK functions.
    ikName = ['IK_' pkgName];
    fkName = ['FK_' pkgName];
    ikFile = '';
    fkFile = fullfile(outputDir, [fkName '.m']);

    if hasIK
        ikFile = fullfile(outputDir, [ikName '.m']);
        ikTxt = [ ...
            'function [q, ok, info] = ' ikName '(T, Urel, q0)' newline ...
            '%' ikName '  Generated deployment wrapper (do not edit).' newline ...
            '%   T    : fixed-world tool pose, 4x4 (mm / rad).' newline ...
            '%   Urel : T_tip<-tool, 4x4; [] uses the package default.' newline ...
            '%   q    : metadata.inputs.names order (mm / rad).' newline ...
            'persistent rt' newline ...
            'if isempty(rt) || ~isvalid(rt)' newline ...
            '    rt = smk.KinematicsRuntime(fileparts(mfilename(''fullpath'')));' newline ...
            'end' newline ...
            'if nargin < 2 || isempty(Urel); Urel = rt.Urel0; end' newline ...
            'if nargin < 3; q0 = []; end' newline ...
            '[q, ok, info] = rt.ik(T, Urel, q0);' newline ...
            'end' newline];
        localWriteText(ikFile, ikTxt);
    end

    fkTxt = [ ...
        'function T = ' fkName '(q, Urel)' newline ...
        '%' fkName '  Generated deployment wrapper (do not edit).' newline ...
        '%   q    : metadata.inputs.names order (mm / rad).' newline ...
        '%   Urel : T_tip<-tool, 4x4; [] uses the package default.' newline ...
        '%   T    : fixed-world tool pose, 4x4 (mm / rad).' newline ...
        'persistent rt' newline ...
        'if isempty(rt) || ~isvalid(rt)' newline ...
        '    rt = smk.KinematicsRuntime(fileparts(mfilename(''fullpath'')));' newline ...
        'end' newline ...
        'if nargin < 2 || isempty(Urel); Urel = rt.Urel0; end' newline ...
        'T = rt.fk(q, Urel);' newline ...
        'end' newline];
    localWriteText(fkFile, fkTxt);
end

function localWriteText(filePath, txt)
    fid = fopen(filePath, 'w', 'n', 'UTF-8');
    assert(fid > 0, 'solver:exportDeployment:writeFailed', ...
        'Cannot write %s.', filePath);
    fwrite(fid, txt, 'char');
    fclose(fid);
end
