function outputPath = export_raw_ball_top_lut( ...
    ocamModel, cameraHeightM, ballDiameterM, forwardAxis, leftAxis, outputPath)
%EXPORT_RAW_BALL_TOP_LUT Export a raw fisheye LUT from pixel to ball-center ground coordinates.
%   outputPath = export_raw_ball_top_lut(ocamModel, cameraHeightM, ...
%       ballDiameterM, forwardAxis, leftAxis, outputPath)
%
%   ocamModel:
%       Loaded ocam_model struct.
%
%   cameraHeightM:
%       Camera optical center height above the ground plane in meters.
%
%   ballDiameterM:
%       Ball diameter in meters.
%
%   forwardAxis / leftAxis:
%       One of: 'camera_x+', 'camera_x-', 'camera_y+', 'camera_y-'
%       The two axes must be orthogonal and must not reuse the same source axis.
%
%   outputPath:
%       Optional output XML path. If empty, a timestamped file is written
%       into the same directory as this script as raw_ball_top_lut_*.xml.

if nargin < 6
    outputPath = '';
end

if nargin < 5 || isempty(leftAxis)
    leftAxis = 'camera_y+';
end
if nargin < 4 || isempty(forwardAxis)
    forwardAxis = 'camera_x+';
end

ocamModel = validateAndNormalizeOcamModel(ocamModel);
forwardMapping = parseAxisMapping(forwardAxis);
leftMapping = parseAxisMapping(leftAxis);
validateAxisPair(forwardMapping, leftMapping);

validateattributes(cameraHeightM, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'}, mfilename, 'cameraHeightM', 2);
validateattributes(ballDiameterM, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'}, mfilename, 'ballDiameterM', 3);

planeDistanceM = cameraHeightM - ballDiameterM;
if planeDistanceM <= 0
    error('cameraHeightM must be larger than ballDiameterM to place the ball top plane below the camera.');
end

sourceHeight = ocamModel.height;
sourceWidth = ocamModel.width;

[rowGrid, colGrid] = ndgrid(1:sourceHeight, 1:sourceWidth);
pixelPoints = [rowGrid(:)'; colGrid(:)'];
rayVectors = cam2worldLocal(pixelPoints, ocamModel);

centerRay = cam2worldLocal([ocamModel.xc; ocamModel.yc], ocamModel);
centerRaySign = sign(centerRay(3));
if centerRaySign == 0
    error('Center ray z component is zero; cannot infer ball-top plane direction.');
end

planeZ = centerRaySign * planeDistanceM;
scale = planeZ ./ rayVectors(3, :);
validMaskVec = ...
    isfinite(scale) & ...
    isfinite(rayVectors(1, :)) & isfinite(rayVectors(2, :)) & isfinite(rayVectors(3, :)) & ...
    abs(rayVectors(3, :)) > 1e-9 & ...
    sign(rayVectors(3, :)) == centerRaySign & ...
    scale > 0;

cameraX = rayVectors(1, :) .* scale;
cameraY = rayVectors(2, :) .* scale;

groundForward = mapAxisValues(forwardMapping, cameraX, cameraY);
groundLeft = mapAxisValues(leftMapping, cameraX, cameraY);

groundX = reshape(single(groundForward), sourceHeight, sourceWidth);
groundY = reshape(single(groundLeft), sourceHeight, sourceWidth);
validMask = reshape(uint8(validMaskVec), sourceHeight, sourceWidth);

groundX(~logical(validMask)) = single(0);
groundY(~logical(validMask)) = single(0);

if isempty(outputPath)
    scriptDir = fileparts(mfilename('fullpath'));
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    outputPath = fullfile(scriptDir, sprintf('raw_ball_top_lut_%s.xml', timestamp));
end

exportMeta = struct( ...
    'source_width', sourceWidth, ...
    'source_height', sourceHeight, ...
    'ocam_xc', ocamModel.xc, ...
    'ocam_yc', ocamModel.yc, ...
    'camera_height_m', cameraHeightM, ...
    'ball_diameter_m', ballDiameterM, ...
    'forward_axis', forwardAxis, ...
    'left_axis', leftAxis, ...
    'plane_distance_m', planeDistanceM, ...
    'plane_z', planeZ, ...
    'center_ray_z_sign', centerRaySign);

writeOpenCvXml(outputPath, groundX, groundY, validMask, exportMeta);
verifyBallDetectorCompatibleXml(outputPath, groundX, groundY, validMask, exportMeta);

fprintf('Saved raw ball LUT:\n  %s\n', outputPath);
fprintf('Source size: %dx%d\n', sourceWidth, sourceHeight);
fprintf('Forward axis: %s, Left axis: %s\n', forwardAxis, leftAxis);
fprintf('Camera height: %.6f m, Ball diameter: %.6f m\n', cameraHeightM, ballDiameterM);

end

function ocamModel = validateAndNormalizeOcamModel(ocamModel)
if ~isstruct(ocamModel)
    error('ocamModel must be a loaded ocam_model struct.');
end

requiredFields = {'ss', 'xc', 'yc', 'width', 'height'};
for idx = 1:numel(requiredFields)
    fieldName = requiredFields{idx};
    if ~isfield(ocamModel, fieldName)
        error('ocam_model is missing required field ''%s''.', fieldName);
    end
end

if ~isfield(ocamModel, 'c') || isempty(ocamModel.c)
    ocamModel.c = 1;
end
if ~isfield(ocamModel, 'd') || isempty(ocamModel.d)
    ocamModel.d = 0;
end
if ~isfield(ocamModel, 'e') || isempty(ocamModel.e)
    ocamModel.e = 0;
end

ocamModel.ss = double(ocamModel.ss(:));
ocamModel.xc = double(ocamModel.xc);
ocamModel.yc = double(ocamModel.yc);
ocamModel.width = double(ocamModel.width);
ocamModel.height = double(ocamModel.height);
ocamModel.c = double(ocamModel.c);
ocamModel.d = double(ocamModel.d);
ocamModel.e = double(ocamModel.e);
end

function mapping = parseAxisMapping(axisName)
switch lower(strtrim(axisName))
    case 'camera_x+'
        mapping.axis = 'x';
        mapping.sign = 1.0;
    case 'camera_x-'
        mapping.axis = 'x';
        mapping.sign = -1.0;
    case 'camera_y+'
        mapping.axis = 'y';
        mapping.sign = 1.0;
    case 'camera_y-'
        mapping.axis = 'y';
        mapping.sign = -1.0;
    otherwise
        error('Unsupported axis mapping ''%s''.', axisName);
end
end

function validateAxisPair(forwardMapping, leftMapping)
if strcmp(forwardMapping.axis, leftMapping.axis)
    error('forwardAxis and leftAxis must be orthogonal and cannot reuse the same camera axis.');
end
end

function values = mapAxisValues(mapping, cameraX, cameraY)
if mapping.axis == 'x'
    values = mapping.sign .* cameraX;
else
    values = mapping.sign .* cameraY;
end
end

function rays = cam2worldLocal(pixelPoints, ocamModel)
nPoints = size(pixelPoints, 2);
A = [ocamModel.c, ocamModel.d; ocamModel.e, 1];
T = [ocamModel.xc; ocamModel.yc] * ones(1, nPoints);
normalizedPixels = A \ (pixelPoints - T);
rho = sqrt(normalizedPixels(1, :).^2 + normalizedPixels(2, :).^2);
z = polyval(ocamModel.ss(end:-1:1), rho);
rays = [normalizedPixels(1, :); normalizedPixels(2, :); z];
norms = sqrt(sum(rays.^2, 1));
norms(norms < eps) = 1;
rays = rays ./ norms;
end

function writeOpenCvXml(filename, groundX, groundY, validMask, meta)
fid = fopen(filename, 'w');
if fid < 0
    error('Cannot open output file for writing:\n  %s', filename);
end
cleanupObj = onCleanup(@() fclose(fid));

fprintf(fid, '<?xml version="1.0"?>\n');
fprintf(fid, '<opencv_storage>\n');
fprintf(fid, '<source_width>%d</source_width>\n', meta.source_width);
fprintf(fid, '<source_height>%d</source_height>\n', meta.source_height);
fprintf(fid, '<ocam_xc>%.12g</ocam_xc>\n', meta.ocam_xc);
fprintf(fid, '<ocam_yc>%.12g</ocam_yc>\n', meta.ocam_yc);
fprintf(fid, '<camera_height_m>%.12g</camera_height_m>\n', meta.camera_height_m);
fprintf(fid, '<ball_diameter_m>%.12g</ball_diameter_m>\n', meta.ball_diameter_m);
fprintf(fid, '<plane_distance_m>%.12g</plane_distance_m>\n', meta.plane_distance_m);
fprintf(fid, '<plane_z>%.12g</plane_z>\n', meta.plane_z);
fprintf(fid, '<center_ray_z_sign>%d</center_ray_z_sign>\n', round(meta.center_ray_z_sign));
fprintf(fid, '<forward_axis>%s</forward_axis>\n', meta.forward_axis);
fprintf(fid, '<left_axis>%s</left_axis>\n', meta.left_axis);

writeOpenCvMatrix(fid, 'ground_x_m', groundX);
writeOpenCvMatrix(fid, 'ground_y_m', groundY);
writeOpenCvMatrix(fid, 'valid_mask', validMask);
fprintf(fid, '</opencv_storage>\n');
end

function writeOpenCvMatrix(fid, name, matrix)
[rows, cols] = size(matrix);
data = reshape(matrix.', 1, []);

if isa(matrix, 'single')
    dt = 'f';
elseif isa(matrix, 'uint8')
    dt = 'u';
else
    error('Unsupported matrix type ''%s'' for OpenCV XML export.', class(matrix));
end

fprintf(fid, '<%s type_id="opencv-matrix">\n', name);
fprintf(fid, '  <rows>%d</rows>\n', rows);
fprintf(fid, '  <cols>%d</cols>\n', cols);
fprintf(fid, '  <dt>%s</dt>\n', dt);
fprintf(fid, '  <data>\n');

valuesPerLine = 8;
for idx = 1:numel(data)
    if mod(idx - 1, valuesPerLine) == 0
        fprintf(fid, '    ');
    end

    if isa(matrix, 'single')
        fprintf(fid, '%.9g', data(idx));
    else
        fprintf(fid, '%d', data(idx));
    end

    if idx < numel(data)
        fprintf(fid, ' ');
    end

    if mod(idx, valuesPerLine) == 0 || idx == numel(data)
        fprintf(fid, '\n');
    end
end

fprintf(fid, '  </data>\n');
fprintf(fid, '</%s>\n', name);
end

function verifyBallDetectorCompatibleXml(filename, expectedGroundX, expectedGroundY, expectedValidMask, meta)
% Verifies the XML contract that orange_ball_detector_node.cpp reads via OpenCV FileStorage.
xmlData = readBallDetectorLutXml(filename);

if xmlData.source_width <= 0 || xmlData.source_height <= 0
    error('XML verification failed: source dimensions must be positive.');
end

expectedSize = size(expectedGroundX);
if ~isequal(size(expectedGroundY), expectedSize) || ~isequal(size(expectedValidMask), expectedSize)
    error('XML verification failed: in-memory LUT matrices do not share the same size.');
end

if xmlData.source_width ~= expectedSize(2) || xmlData.source_height ~= expectedSize(1)
    error(['XML verification failed: source_width/source_height do not match exported matrix size. ' ...
        'Detector expects frame.cols == source_width and frame.rows == source_height.']);
end

if ~isequal(size(xmlData.ground_x_m), expectedSize) || ...
        ~isequal(size(xmlData.ground_y_m), expectedSize) || ...
        ~isequal(size(xmlData.valid_mask), expectedSize)
    error('XML verification failed: round-tripped LUT matrix sizes do not match the exported matrices.');
end

if ~strcmp(xmlData.ground_x_dt, 'f') || ~strcmp(xmlData.ground_y_dt, 'f')
    error('XML verification failed: ground_x_m and ground_y_m must be stored as OpenCV dt="f".');
end

if ~strcmp(xmlData.valid_mask_dt, 'u')
    error('XML verification failed: valid_mask must be stored as OpenCV dt="u".');
end

if ~isequal(xmlData.valid_mask, expectedValidMask)
    error('XML verification failed: valid_mask changed after XML round-trip.');
end

groundXTol = 1e-5;
groundYTol = 1e-5;
maxGroundXErr = max(abs(double(xmlData.ground_x_m(:)) - double(expectedGroundX(:))));
maxGroundYErr = max(abs(double(xmlData.ground_y_m(:)) - double(expectedGroundY(:))));
if maxGroundXErr > groundXTol || maxGroundYErr > groundYTol
    error(['XML verification failed: LUT values changed after XML round-trip. ' ...
        'maxGroundXErr=%.9g, maxGroundYErr=%.9g'], maxGroundXErr, maxGroundYErr);
end

scalarTol = 1e-9;
assertScalarClose(xmlData.ocam_xc, meta.ocam_xc, scalarTol, 'ocam_xc');
assertScalarClose(xmlData.ocam_yc, meta.ocam_yc, scalarTol, 'ocam_yc');
assertScalarClose(xmlData.camera_height_m, meta.camera_height_m, scalarTol, 'camera_height_m');
assertScalarClose(xmlData.ball_diameter_m, meta.ball_diameter_m, scalarTol, 'ball_diameter_m');

fprintf('Verified ball-detector XML contract:\n');
fprintf('  %s\n', filename);
fprintf('  Matrices: ground_x_m(CV_32FC1), ground_y_m(CV_32FC1), valid_mask(CV_8UC1)\n');
fprintf('  Source size: %dx%d\n', xmlData.source_width, xmlData.source_height);
fprintf('  Max round-trip error: ground_x=%.9g m, ground_y=%.9g m\n', maxGroundXErr, maxGroundYErr);
end

function xmlData = readBallDetectorLutXml(filename)
doc = xmlread(filename);
root = doc.getDocumentElement();

xmlData = struct();
xmlData.source_width = readScalarNode(root, 'source_width');
xmlData.source_height = readScalarNode(root, 'source_height');
xmlData.ocam_xc = readScalarNode(root, 'ocam_xc');
xmlData.ocam_yc = readScalarNode(root, 'ocam_yc');
xmlData.camera_height_m = readScalarNode(root, 'camera_height_m');
xmlData.ball_diameter_m = readScalarNode(root, 'ball_diameter_m');

[xmlData.ground_x_m, xmlData.ground_x_dt] = readOpenCvMatrixNode(root, 'ground_x_m');
[xmlData.ground_y_m, xmlData.ground_y_dt] = readOpenCvMatrixNode(root, 'ground_y_m');
[xmlData.valid_mask, xmlData.valid_mask_dt] = readOpenCvMatrixNode(root, 'valid_mask');
end

function [matrix, dt] = readOpenCvMatrixNode(root, tagName)
matrixNode = getSingleNode(root, tagName);
rows = round(readScalarChildNode(matrixNode, 'rows'));
cols = round(readScalarChildNode(matrixNode, 'cols'));
dt = strtrim(readStringChildNode(matrixNode, 'dt'));
dataText = readStringChildNode(matrixNode, 'data');

values = sscanf(dataText, '%f').';
expectedCount = rows * cols;
if numel(values) ~= expectedCount
    error('XML verification failed: node ''%s'' contains %d values, expected %d.', ...
        tagName, numel(values), expectedCount);
end

matrix = reshape(values, cols, rows).';
switch dt
    case 'f'
        matrix = single(matrix);
    case 'u'
        matrix = uint8(matrix);
    otherwise
        error('XML verification failed: unsupported OpenCV dt ''%s'' in node ''%s''.', dt, tagName);
end
end

function value = readScalarNode(root, tagName)
value = str2double(readStringNode(root, tagName));
if ~isfinite(value)
    error('XML verification failed: node ''%s'' is not a finite scalar.', tagName);
end
end

function value = readScalarChildNode(parentNode, tagName)
value = str2double(readStringChildNode(parentNode, tagName));
if ~isfinite(value)
    error('XML verification failed: child node ''%s'' is not a finite scalar.', tagName);
end
end

function text = readStringNode(root, tagName)
node = getSingleNode(root, tagName);
text = strtrim(char(node.getTextContent()));
end

function text = readStringChildNode(parentNode, tagName)
node = getSingleNode(parentNode, tagName);
text = strtrim(char(node.getTextContent()));
end

function node = getSingleNode(parentNode, tagName)
nodeList = parentNode.getElementsByTagName(tagName);
if nodeList.getLength() ~= 1
    error('XML verification failed: expected exactly one node named ''%s'', found %d.', ...
        tagName, nodeList.getLength());
end
node = nodeList.item(0);
end

function assertScalarClose(actualValue, expectedValue, tolerance, fieldName)
if abs(actualValue - expectedValue) > tolerance
    error('XML verification failed: %s mismatch. actual=%.12g expected=%.12g', ...
        fieldName, actualValue, expectedValue);
end
end
