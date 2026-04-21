% Runner for export_raw_ball_top_lut.m
% Edit the parameters below, then run this script in MATLAB.

addpath("D:\RCJ\26\vision\fisheye_project\Scaramuzza_OCamCalib_v3.0_win\super800600cam_data")

load('Omni_Calib_Results.mat');
ocamModel = calib_data.ocam_model;

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end
addpath(scriptDir);

projectRoot = fileparts(scriptDir);

% Input parameters
calibMatPath = fullfile(scriptDir, 'Omni_Calib_Results.mat');
cameraHeightM = 0.19;
ballDiameterM = 0.042;
forwardAxis = 'camera_x+';
leftAxis = 'camera_y-';

% Leave empty to auto-generate:
%   <projectRoot>/config/raw_ball_top_lut_YYYYMMDD_HHMMSS.xml
outputPath = '';

% Generic loader kept here for reference:
% calibStruct = load(calibMatPath);
% if isfield(calibStruct, 'calib_data') && isfield(calibStruct.calib_data, 'ocam_model')
%     ocamModel = calibStruct.calib_data.ocam_model;
% elseif isfield(calibStruct, 'ocam_model')
%     ocamModel = calibStruct.ocam_model;
% else
%     error(['MAT file does not contain calib_data.ocam_model or ocam_model:\n  %s'], ...
%         calibMatPath);
% end



fprintf('Loaded ocam_model from:\n');
fprintf('  load(''Omni_Calib_Results.mat'') -> calib_data.ocam_model\n');
fprintf('Export parameters:\n');
fprintf('  cameraHeightM : %.6f m\n', cameraHeightM);
fprintf('  ballDiameterM : %.6f m\n', ballDiameterM);
fprintf('  forwardAxis   : %s\n', forwardAxis);
fprintf('  leftAxis      : %s\n', leftAxis);
if isempty(outputPath)
    fprintf('  outputPath    : <auto>\n');
else
    fprintf('  outputPath    : %s\n', outputPath);
end

generatedPath = export_raw_ball_top_lut( ...
    ocamModel, ...
    cameraHeightM, ...
    ballDiameterM, ...
    forwardAxis, ...
    leftAxis, ...
    outputPath);

fprintf('Generated LUT XML:\n');
fprintf('  %s\n', generatedPath);

checkerPath = fullfile(projectRoot, 'build', 'rcj_localization', 'orange_ball_lut_checker');
if exist(checkerPath, 'file') == 2
    fprintf('Optional C++ checker:\n');
    fprintf('  %s --lut "%s"\n', checkerPath, generatedPath);
end
