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
[areaPriorDistanceM, areaPriorExpectedAreaPx] = buildBallAreaPriorCurve( ...
    ocamModel, cameraHeightM, ballDiameterM, forwardMapping, leftMapping, ...
    centerRaySign, groundX, groundY, validMask);

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
    'center_ray_z_sign', centerRaySign, ...
    'area_prior_sample_count', numel(areaPriorDistanceM));

writeOpenCvXml(outputPath, groundX, groundY, validMask, ...
    areaPriorDistanceM, areaPriorExpectedAreaPx, exportMeta);
verifyBallDetectorCompatibleXml(outputPath, groundX, groundY, validMask, ...
    areaPriorDistanceM, areaPriorExpectedAreaPx, exportMeta);

fprintf('Saved raw ball LUT:\n  %s\n', outputPath);
fprintf('Source size: %dx%d\n', sourceWidth, sourceHeight);
fprintf('Forward axis: %s, Left axis: %s\n', forwardAxis, leftAxis);
fprintf('Camera height: %.6f m, Ball diameter: %.6f m\n', cameraHeightM, ballDiameterM);
fprintf('Area prior samples: %d, distance range [%.6f, %.6f] m, expected area range [%.6f, %.6f] px\n', ...
    numel(areaPriorDistanceM), double(areaPriorDistanceM(1)), double(areaPriorDistanceM(end)), ...
    double(areaPriorExpectedAreaPx(end)), double(areaPriorExpectedAreaPx(1)));

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

function [distanceSamplesM, expectedAreaSamplesPx] = buildBallAreaPriorCurve( ...
    ocamModel, cameraHeightM, ballDiameterM, forwardMapping, leftMapping, ...
    centerRaySign, groundX, groundY, validMask)
ballRadiusM = ballDiameterM * 0.5;
validDistances = hypot(double(groundX(logical(validMask))), double(groundY(logical(validMask))));
validDistances = validDistances(isfinite(validDistances) & validDistances > 0);
if numel(validDistances) < 2
    error('Cannot build area prior because the LUT valid region does not contain enough positive distances.');
end

areaPriorMinDistanceM = 0.05;
areaPriorMaxDistanceM = 3.0;
if areaPriorMaxDistanceM <= areaPriorMinDistanceM
    error('Area-prior sampling range must satisfy max > min.');
end

distanceSamplesM = linspace(areaPriorMinDistanceM, areaPriorMaxDistanceM, 256).';
ballCenterZ = centerRaySign * (cameraHeightM - ballRadiusM);
[thetaSamples, rhoSamples] = buildThetaRhoLookup(ocamModel);

expectedAreaSamplesPx = nan(size(distanceSamplesM), 'single');
for idx = 1:numel(distanceSamplesM)
    [cameraX, cameraY] = groundToCameraAxes( ...
        distanceSamplesM(idx), 0.0, forwardMapping, leftMapping);
    sphereCenter = [cameraX; cameraY; ballCenterZ];
    areaPx = estimateSphereProjectedAreaPx( ...
        sphereCenter, ballRadiusM, ocamModel, thetaSamples, rhoSamples);
    if isfinite(areaPx) && areaPx > 0
        expectedAreaSamplesPx(idx) = single(areaPx);
    end
end

validSamples = isfinite(expectedAreaSamplesPx) & expectedAreaSamplesPx > 0;
distanceSamplesM = single(distanceSamplesM(validSamples));
expectedAreaSamplesPx = single(expectedAreaSamplesPx(validSamples));
if numel(distanceSamplesM) < 2
    error('Cannot build area prior because fewer than two valid projected-area samples were generated.');
end

for idx = 2:numel(expectedAreaSamplesPx)
    expectedAreaSamplesPx(idx) = min(expectedAreaSamplesPx(idx), expectedAreaSamplesPx(idx - 1));
end
end

function [cameraX, cameraY] = groundToCameraAxes(forwardValue, leftValue, forwardMapping, leftMapping)
cameraX = 0.0;
cameraY = 0.0;

if forwardMapping.axis == 'x'
    cameraX = forwardMapping.sign * forwardValue;
else
    cameraY = forwardMapping.sign * forwardValue;
end

if leftMapping.axis == 'x'
    cameraX = leftMapping.sign * leftValue;
else
    cameraY = leftMapping.sign * leftValue;
end
end

function areaPx = estimateSphereProjectedAreaPx( ...
    sphereCenter, sphereRadiusM, ocamModel, thetaSamples, rhoSamples)
centerDistance = norm(sphereCenter);
if ~isfinite(centerDistance) || centerDistance <= sphereRadiusM
    areaPx = nan;
    return;
end

centerDirection = sphereCenter / centerDistance;
circleCenter = centerDirection * ((centerDistance^2 - sphereRadiusM^2) / centerDistance);
circleRadius = sphereRadiusM * sqrt(max(0.0, 1.0 - (sphereRadiusM^2 / centerDistance^2)));

referenceAxis = [0; 0; 1];
if abs(dot(centerDirection, referenceAxis)) > 0.95
    referenceAxis = [0; 1; 0];
end
uAxis = cross(centerDirection, referenceAxis);
uNorm = norm(uAxis);
if uNorm < eps
    areaPx = nan;
    return;
end
uAxis = uAxis / uNorm;
vAxis = cross(centerDirection, uAxis);
vAxis = vAxis / max(norm(vAxis), eps);

angles = linspace(0, 2 * pi, 145);
angles(end) = [];
boundaryPoints = circleCenter + circleRadius * (uAxis * cos(angles) + vAxis * sin(angles));
imagePoints = world2camLocal(boundaryPoints, ocamModel, thetaSamples, rhoSamples);
if any(~isfinite(imagePoints(:)))
    areaPx = nan;
    return;
end

areaPx = polyarea(imagePoints(1, :), imagePoints(2, :));
end

function [thetaSamples, rhoSamples] = buildThetaRhoLookup(ocamModel)
maxRho = computeMaxNormalizedImageRadius(ocamModel) * 1.05;
rhoSamples = linspace(0, maxRho, 4096).';
zSamples = polyval(ocamModel.ss(end:-1:1), rhoSamples);
thetaSamples = atan2(zSamples, rhoSamples);

validSamples = isfinite(thetaSamples) & isfinite(rhoSamples);
thetaSamples = thetaSamples(validSamples);
rhoSamples = rhoSamples(validSamples);

[thetaSamples, sortIdx] = sort(thetaSamples);
rhoSamples = rhoSamples(sortIdx);
[thetaSamples, uniqueIdx] = unique(thetaSamples, 'stable');
rhoSamples = rhoSamples(uniqueIdx);
if numel(thetaSamples) < 2
    error('Failed to build a valid theta-to-rho lookup for the ocam model.');
end
end

function maxRho = computeMaxNormalizedImageRadius(ocamModel)
pixelCorners = [ ...
    1, 1, ocamModel.height, ocamModel.height; ...
    1, ocamModel.width, 1, ocamModel.width];
A = [ocamModel.c, ocamModel.d; ocamModel.e, 1];
T = [ocamModel.xc; ocamModel.yc];
normalizedCorners = A \ (pixelCorners - T);
cornerRho = hypot(normalizedCorners(1, :), normalizedCorners(2, :));
maxRho = max(cornerRho);
if ~isfinite(maxRho) || maxRho <= 0
    error('Failed to infer a valid normalized image radius from the ocam model.');
end
end

function imagePoints = world2camLocal(points3D, ocamModel, thetaSamples, rhoSamples)
xyNorm = hypot(points3D(1, :), points3D(2, :));
theta = atan2(points3D(3, :), xyNorm);
rho = interp1(thetaSamples, rhoSamples, theta, 'linear', nan);

xNorm = nan(size(theta));
yNorm = nan(size(theta));
centerMask = xyNorm <= eps & isfinite(rho);
xNorm(centerMask) = 0;
yNorm(centerMask) = 0;

offAxisMask = xyNorm > eps & isfinite(rho);
xNorm(offAxisMask) = points3D(1, offAxisMask) ./ xyNorm(offAxisMask) .* rho(offAxisMask);
yNorm(offAxisMask) = points3D(2, offAxisMask) ./ xyNorm(offAxisMask) .* rho(offAxisMask);

imagePoints = [ ...
    xNorm * ocamModel.c + yNorm * ocamModel.d + ocamModel.xc; ...
    xNorm * ocamModel.e + yNorm + ocamModel.yc];
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

function writeOpenCvXml( ...
    filename, groundX, groundY, validMask, areaPriorDistanceM, areaPriorExpectedAreaPx, meta)
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
writeOpenCvMatrix(fid, 'area_prior_distance_m', areaPriorDistanceM);
writeOpenCvMatrix(fid, 'area_prior_expected_area_px', areaPriorExpectedAreaPx);
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

function verifyBallDetectorCompatibleXml( ...
    filename, expectedGroundX, expectedGroundY, expectedValidMask, ...
    expectedAreaPriorDistanceM, expectedAreaPriorExpectedAreaPx, meta)
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
if xmlData.area_prior_distance_m_total ~= numel(expectedAreaPriorDistanceM) || ...
        xmlData.area_prior_expected_area_px_total ~= numel(expectedAreaPriorExpectedAreaPx)
    error('XML verification failed: round-tripped area-prior vector sizes do not match the exported vectors.');
end

if ~strcmp(xmlData.ground_x_dt, 'f') || ~strcmp(xmlData.ground_y_dt, 'f')
    error('XML verification failed: ground_x_m and ground_y_m must be stored as OpenCV dt="f".');
end

if ~strcmp(xmlData.valid_mask_dt, 'u')
    error('XML verification failed: valid_mask must be stored as OpenCV dt="u".');
end
if ~strcmp(xmlData.area_prior_distance_dt, 'f') || ~strcmp(xmlData.area_prior_expected_area_dt, 'f')
    error('XML verification failed: area_prior_distance_m and area_prior_expected_area_px must be stored as OpenCV dt="f".');
end

if ~isequal(xmlData.valid_mask, expectedValidMask)
    error('XML verification failed: valid_mask changed after XML round-trip.');
end
if any(double(xmlData.area_prior_distance_m(:)) ~= sort(double(xmlData.area_prior_distance_m(:))))
    error('XML verification failed: area_prior_distance_m must be strictly increasing.');
end
if any(diff(double(xmlData.area_prior_distance_m(:))) <= 0)
    error('XML verification failed: area_prior_distance_m must be strictly increasing.');
end
if any(diff(double(xmlData.area_prior_expected_area_px(:))) > 1e-6)
    error('XML verification failed: area_prior_expected_area_px must be monotonic non-increasing.');
end
if any(~isfinite(double(xmlData.area_prior_expected_area_px(:)))) || ...
        any(double(xmlData.area_prior_expected_area_px(:)) <= 0)
    error('XML verification failed: area_prior_expected_area_px must stay positive and finite.');
end

groundXTol = 1e-5;
groundYTol = 1e-5;
areaPriorDistanceTol = 1e-5;
areaPriorExpectedAreaTol = 1e-4;
maxGroundXErr = max(abs(double(xmlData.ground_x_m(:)) - double(expectedGroundX(:))));
maxGroundYErr = max(abs(double(xmlData.ground_y_m(:)) - double(expectedGroundY(:))));
maxAreaPriorDistanceErr = max(abs(double(xmlData.area_prior_distance_m(:)) - double(expectedAreaPriorDistanceM(:))));
maxAreaPriorExpectedAreaErr = max(abs(double(xmlData.area_prior_expected_area_px(:)) - double(expectedAreaPriorExpectedAreaPx(:))));
if maxGroundXErr > groundXTol || maxGroundYErr > groundYTol
    error(['XML verification failed: LUT values changed after XML round-trip. ' ...
        'maxGroundXErr=%.9g, maxGroundYErr=%.9g'], maxGroundXErr, maxGroundYErr);
end
if maxAreaPriorDistanceErr > areaPriorDistanceTol || ...
        maxAreaPriorExpectedAreaErr > areaPriorExpectedAreaTol
    error(['XML verification failed: area-prior values changed after XML round-trip. ' ...
        'maxAreaPriorDistanceErr=%.9g, maxAreaPriorExpectedAreaErr=%.9g'], ...
        maxAreaPriorDistanceErr, maxAreaPriorExpectedAreaErr);
end

scalarTol = 1e-9;
assertScalarClose(xmlData.ocam_xc, meta.ocam_xc, scalarTol, 'ocam_xc');
assertScalarClose(xmlData.ocam_yc, meta.ocam_yc, scalarTol, 'ocam_yc');
assertScalarClose(xmlData.camera_height_m, meta.camera_height_m, scalarTol, 'camera_height_m');
assertScalarClose(xmlData.ball_diameter_m, meta.ball_diameter_m, scalarTol, 'ball_diameter_m');

fprintf('Verified ball-detector XML contract:\n');
fprintf('  %s\n', filename);
fprintf('  Matrices: ground_x_m(CV_32FC1), ground_y_m(CV_32FC1), valid_mask(CV_8UC1), ');
fprintf('area_prior_distance_m(CV_32FC1), area_prior_expected_area_px(CV_32FC1)\n');
fprintf('  Source size: %dx%d\n', xmlData.source_width, xmlData.source_height);
fprintf('  Max round-trip error: ground_x=%.9g m, ground_y=%.9g m\n', maxGroundXErr, maxGroundYErr);
fprintf('  Area prior round-trip error: distance=%.9g m, expected_area=%.9g px\n', ...
    maxAreaPriorDistanceErr, maxAreaPriorExpectedAreaErr);
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
[xmlData.area_prior_distance_m, xmlData.area_prior_distance_dt] = ...
    readOpenCvMatrixNode(root, 'area_prior_distance_m');
[xmlData.area_prior_expected_area_px, xmlData.area_prior_expected_area_dt] = ...
    readOpenCvMatrixNode(root, 'area_prior_expected_area_px');
xmlData.area_prior_distance_m_total = numel(xmlData.area_prior_distance_m);
xmlData.area_prior_expected_area_px_total = numel(xmlData.area_prior_expected_area_px);
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
