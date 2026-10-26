%% test_execution_config_open_chain_2r.m
% End-to-end test: DSL → Expander → SymbolRegistry → ExecutionConfig.
%
% Verifies the A.3.3 pipeline:
%   1. Expander collects SymbolRegistry (observable joints + task frames)
%   2. ExecutionConfig loads and validates execution-config YAML
%   3. ExecutionConfig cross-validates against SymbolRegistry
%   4. Variable partition + solving-direction detection
%
% Requires: Symbolic Math Toolbox
%
% Usage:
%   addpath(genpath('../../scripts/matlab'));
%   run('test_execution_config_open_chain_2r.m');

fprintf('=== Execution Config Pipeline Test: open-chain-2r ===\n\n');

%% Paths
dslFile       = '../../specs/dsl/cases/open-chain-2r/robot_description.yaml';
execCfgFile   = '../../specs/dsl/cases/open-chain-2r/execution-config.yaml';

%% 1. Build symbolic pipeline and collect SymbolRegistry
fprintf('1. Building symbolic pipeline ... ');
try
    eSym = ir.Expander(dslFile);
catch ME
    if contains(ME.message, 'Unrecognized function or variable') && contains(ME.message, 'sym')
        error('Symbolic Math Toolbox required. Install it to run this test.');
    end
    rethrow(ME);
end
fprintf('OK (%d instances, %d edges)\n', ...
    numel(eSym.Instances), eSym.EdgeGraph_.numEdges);

%% 2. Verify SymbolRegistry contents
fprintf('\n2. Checking SymbolRegistry ... ');
reg = eSym.SymbolRegistry;
assert(~isempty(reg), 'SymbolRegistry must not be empty.');
fprintf('%d entries\n', numel(reg));

% Print registry contents
for i = 1:numel(reg)
    fprintf('   [%d] %-40s type=%-10s scope=%-10s module=%-12s instance=%s\n', ...
        i, reg(i).name, reg(i).type, reg(i).scope, reg(i).module_type, reg(i).instance);
end

% Verify joint entries
regNames = {reg.name};
jointIdx = find(strcmp({reg.type}, 'joint'));
fprintf('\n   Joint variables: ');
for i = 1:numel(jointIdx)
    fprintf('%s ', reg(jointIdx(i)).name);
end
fprintf('\n');
assert(numel(jointIdx) >= 2, 'Expected at least 2 joint variables.');
assert(ismember('joint1.q', regNames), 'joint1.q missing from SymbolRegistry.');
assert(ismember('joint2.q', regNames), 'joint2.q missing from SymbolRegistry.');

% Verify task frame entries
taskIdx = find(strcmp({reg.type}, 'task'));
fprintf('   Task frames: ');
for i = 1:numel(taskIdx)
    fprintf('%s ', reg(taskIdx(i)).name);
end
fprintf('\n');
assert(numel(taskIdx) >= 1, 'Expected at least 1 task frame.');

% Verify task frame symHandles are populated (not empty)
for i = 1:numel(taskIdx)
    sh = reg(taskIdx(i)).symHandle;
    assert(~isempty(sh), 'Task frame "%s" symHandle is empty.', reg(taskIdx(i)).name);
    assert(isa(sh, 'sym'), 'Task frame "%s" symHandle must be sym type.', reg(taskIdx(i)).name);
end

fprintf('   All task frame symHandles populated ✓\n');

%% 3. Load and validate ExecutionConfig
fprintf('\n3. Loading ExecutionConfig ... ');
cfg = ir.ExecutionConfig(execCfgFile, eSym.SymbolRegistry, eSym.EdgeGraph_);
fprintf('OK\n');
fprintf('   Mode:       %s\n', cfg.Mode);
fprintf('   EndFrame:   %s\n', cfg.EndFrame);
fprintf('   Direction:  %s\n', cfg.getSolvingDirection());

%% 4. Verify ExecutionConfig properties
fprintf('\n4. Verifying ExecutionConfig ... ');
assert(strcmp(cfg.Mode, 'open_loop'), 'Mode must be open_loop.');
assert(strcmp(cfg.getSolvingDirection(), 'FK'), 'Direction must be FK.');
assert(strcmp(cfg.EndFrame, 'frame2.frame_hyper_cube'), ...
    'EndFrame should be frame2.frame_hyper_cube.');

% Verify known/unknown partition
[knownV, unknownV] = cfg.partitionVariables();
assert(numel(knownV) == 2, 'Expected 2 known variables, got %d.', numel(knownV));
assert(numel(unknownV) == 1, 'Expected 1 unknown variable, got %d.', numel(unknownV));

knownNames = {knownV.name};
assert(ismember('joint1.q', knownNames), 'joint1.q should be in known.');
assert(ismember('joint2.q', knownNames), 'joint2.q should be in known.');
assert(strcmp(unknownV(1).name, 'frame2.frame_hyper_cube'), ...
    'Unknown should be frame2.frame_hyper_cube.');

% Verify actuated joints
actJoints = cfg.ActuatedJoints;
assert(numel(actJoints) == 2, 'Expected 2 actuated joints.');
fprintf('OK\n');

%% 5. Test ExecutionConfig validation: invalid refs should error
fprintf('\n5. Testing ExecutionConfig validation ... ');

% Test: unknown ref that doesn't exist in symbolRegistry
% Write a minimal bad config to a temp YAML file
tmpFile = [tempname '.yaml'];
fid = fopen(tmpFile, 'w');
fprintf(fid, 'mode: open_loop\n');
fprintf(fid, 'endFrame: nonexistent.frame\n');
fprintf(fid, 'actuated_joints:\n');
fprintf(fid, '  - joint1.q\n');
fprintf(fid, 'known:\n');
fprintf(fid, '  - joint1.q\n');
fprintf(fid, 'unknown:\n');
fprintf(fid, '  - nonexistent.frame\n');
fclose(fid);

try
    ir.ExecutionConfig(tmpFile, eSym.SymbolRegistry);
    error('Expected error for bad ref, but none thrown.');
catch ME
    assert(contains(ME.message, 'not found'), ...
        'Expected "not found" error, got: %s', ME.message);
    fprintf('OK (correctly rejected bad ref)\n');
end
delete(tmpFile);

%% 6. Summary
fprintf('\n=== All A.3.3 Execution Config tests passed ===\n');
fprintf('   ✓ SymbolRegistry collection (joints + task frames)\n');
fprintf('   ✓ ExecutionConfig loading and validation\n');
fprintf('   ✓ Variable partitioning (known/unknown)\n');
fprintf('   ✓ Solving direction detection (FK)\n');
fprintf('   ✓ Validation rejects invalid refs\n');
