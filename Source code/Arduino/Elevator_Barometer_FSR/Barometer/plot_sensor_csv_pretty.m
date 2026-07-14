%% plot_sensor_csv_pretty.m
% Pretty plotting for ESP32 / BMP390 floor sensor CSV
%
% Save this file as plot_sensor_csv_pretty.m
% Do NOT name it plot.m, because that breaks MATLAB's built-in plot().

clear; clc; close all;

%% Load CSV

[file, path] = uigetfile("*.csv", "Select sensor CSV");

if isequal(file, 0)
    error("No file selected.");
end

filename = fullfile(path, file);
T = readtable(filename);

%% Convert numeric columns safely

numericVars = [
    "time_ms"
    "temperature_C"
    "pressure_Pa"
    "pressure_hPa"
    "relative_height_m"
    "estimated_floor"
    "offset_from_estimated_floor_m"
    "actual_floor"
    "correct"
    "calibrated_floor"
];

for k = 1:numel(numericVars)
    v = numericVars(k);

    if ismember(v, T.Properties.VariableNames)
        if ~isnumeric(T.(v))
            T.(v) = str2double(string(T.(v)));
        end
    end
end

%% Basic cleanup

T = T(~isnan(T.time_ms), :);
T = sortrows(T, "time_ms");

T.time_s = (T.time_ms - T.time_ms(1)) / 1000;
T.time_min = T.time_s / 60;

%% Select time range to plot

% Set this to false if you want fixed values instead of a popup.
askForTimeRange = true;

% Fixed values, used if askForTimeRange = false
startMin = 0;
endMin = max(T.time_min);

if askForTimeRange
    answer = inputdlg( ...
        {"Start time [min]:", "End time [min]:"}, ...
        "Select plot time range", ...
        [1 35], ...
        {"0", sprintf("%.2f", max(T.time_min))});

    if isempty(answer)
        error("No time range selected.");
    end

    startMin = str2double(answer{1});
    endMin   = str2double(answer{2});
end

if isnan(startMin) || isnan(endMin)
    error("Start time and end time must be numbers.");
end

if endMin <= startMin
    error("End time must be larger than start time.");
end

inTimeRange = T.time_min >= startMin & T.time_min <= endMin;

if ~any(inTimeRange)
    error("No data found in selected time range %.2f to %.2f min.", startMin, endMin);
end

%% Ignore negative sensor values

ignoreNegativeVars = [
    "temperature_C"
    "pressure_Pa"
    "pressure_hPa"
    "relative_height_m"
    "offset_from_estimated_floor_m"
];

for k = 1:numel(ignoreNegativeVars)
    v = ignoreNegativeVars(k);

    if ismember(v, T.Properties.VariableNames)
        T.(v)(T.(v) < 0) = NaN;
    end
end

%% Row types

if ismember("type", T.Properties.VariableNames)
    rowType = lower(string(T.type));
else
    rowType = strings(height(T), 1);
    rowType(:) = "read";
end

isRead = rowType == "read";
isTest = rowType == "test";
isCalibration = rowType == "calibration";

isMeasurementAll = isRead | isTest;
isMeasurement = isMeasurementAll & inTimeRange;

hasActualAll = ismember("actual_floor", T.Properties.VariableNames) & ~isnan(T.actual_floor);
hasActual = hasActualAll & inTimeRange;

hasCalFloorAll = ismember("calibrated_floor", T.Properties.VariableNames) & ~isnan(T.calibrated_floor);

calRowsAll = isCalibration & hasCalFloorAll;
calRows = calRowsAll & inTimeRange;

testRows = (isTest | hasActualAll) & inTimeRange;

%% Plot settings

maxGapMin = 0.5;
markerSize = 55;

blue   = [0.0000 0.4470 0.7410];
orange = [0.8500 0.3250 0.0980];
purple = [0.4940 0.1840 0.5560];
green  = [0.4660 0.6740 0.1880];
dark   = [0.15 0.15 0.15];

rangeText = sprintf("%.2f to %.2f min", startMin, endMin);

%% Main pretty dashboard

fig = figure("Name", "Sensor CSV Overview", "Color", "w");
fig.Position = [100 100 1200 850];

tl = tiledlayout(4, 1);
tl.TileSpacing = "compact";
tl.Padding = "compact";

title(tl, "Sensor CSV Overview: " + string(file) + " | " + rangeText, ...
    "FontSize", 18, ...
    "FontWeight", "bold", ...
    "Interpreter", "none");

%% Pressure

nexttile;

plotWithGaps(T.time_min(isMeasurement), ...
             T.pressure_hPa(isMeasurement), ...
             maxGapMin, ...
             "Color", blue, ...
             "LineWidth", 1.8, ...
             "DisplayName", "Measurement");

hold on;

validCalPressure = calRows & ~isnan(T.pressure_hPa);
if any(validCalPressure)
    scatter(T.time_min(validCalPressure), ...
            T.pressure_hPa(validCalPressure), ...
            markerSize + 25, ...
            orange, ...
            "diamond", ...
            "filled", ...
            "DisplayName", "Calibration");
end

validTestPressure = testRows & ~isnan(T.pressure_hPa);
if any(validTestPressure)
    scatter(T.time_min(validTestPressure), ...
            T.pressure_hPa(validTestPressure), ...
            markerSize, ...
            purple, ...
            "filled", ...
            "DisplayName", "Test / actual floor");
end

grid on;
box on;
xlim([startMin endMin]);
ylabel("Pressure [hPa]");
title("Pressure over time");
legend("Location", "best");

applyNiceYLim(T.pressure_hPa(isMeasurement));

%% Temperature

nexttile;

plotWithGaps(T.time_min(isMeasurement), ...
             T.temperature_C(isMeasurement), ...
             maxGapMin, ...
             "Color", green, ...
             "LineWidth", 1.8, ...
             "DisplayName", "Measurement");

hold on;

validCalTemp = calRows & ~isnan(T.temperature_C);
if any(validCalTemp)
    scatter(T.time_min(validCalTemp), ...
            T.temperature_C(validCalTemp), ...
            markerSize + 25, ...
            orange, ...
            "diamond", ...
            "filled", ...
            "DisplayName", "Calibration");
end

validTestTemp = testRows & ~isnan(T.temperature_C);
if any(validTestTemp)
    scatter(T.time_min(validTestTemp), ...
            T.temperature_C(validTestTemp), ...
            markerSize, ...
            purple, ...
            "filled", ...
            "DisplayName", "Test / actual floor");
end

grid on;
box on;
xlim([startMin endMin]);
ylabel("Temperature [°C]");
title("Temperature over time");
legend("Location", "best");

applyNiceYLim(T.temperature_C(isMeasurement));

%% Relative height

nexttile;

plotWithGaps(T.time_min(isMeasurement), ...
             T.relative_height_m(isMeasurement), ...
             maxGapMin, ...
             "Color", blue, ...
             "LineWidth", 1.8, ...
             "DisplayName", "Measured height");

hold on;

validCalHeight = calRows & ~isnan(T.relative_height_m);
if any(validCalHeight)
    scatter(T.time_min(validCalHeight), ...
            T.relative_height_m(validCalHeight), ...
            markerSize + 25, ...
            orange, ...
            "diamond", ...
            "filled", ...
            "DisplayName", "Calibration");

    for i = find(validCalHeight)'
        text(T.time_min(i), ...
             T.relative_height_m(i), ...
             " floor " + string(T.calibrated_floor(i)), ...
             "FontSize", 9, ...
             "Color", dark, ...
             "VerticalAlignment", "bottom", ...
             "HorizontalAlignment", "left");
    end
end

validTestHeight = testRows & ~isnan(T.relative_height_m);
if any(validTestHeight)
    scatter(T.time_min(validTestHeight), ...
            T.relative_height_m(validTestHeight), ...
            markerSize, ...
            purple, ...
            "filled", ...
            "DisplayName", "Test / actual floor");
end

grid on;
box on;
xlim([startMin endMin]);
ylabel("Height [m]");
title("Relative height");
legend("Location", "best");

applyNiceYLim(T.relative_height_m(isMeasurement));

%% Estimated floor and actual floor

nexttile;

stairsWithGaps(T.time_min(isMeasurement), ...
               T.estimated_floor(isMeasurement), ...
               maxGapMin, ...
               "Color", blue, ...
               "LineWidth", 2.0, ...
               "DisplayName", "Estimated floor");

hold on;

validActualFloor = hasActual & ~isnan(T.actual_floor);
if any(validActualFloor)
    scatter(T.time_min(validActualFloor), ...
            T.actual_floor(validActualFloor), ...
            markerSize + 20, ...
            purple, ...
            "filled", ...
            "DisplayName", "Actual floor");
end

validCalFloor = calRows & ~isnan(T.calibrated_floor);
if any(validCalFloor)
    scatter(T.time_min(validCalFloor), ...
            T.calibrated_floor(validCalFloor), ...
            markerSize + 25, ...
            orange, ...
            "diamond", ...
            "filled", ...
            "DisplayName", "Calibrated floor");
end

grid on;
box on;
xlim([startMin endMin]);
xlabel("Time [min]");
ylabel("Floor");
title("Estimated floor over time");
legend("Location", "best");

allFloors = [
    T.estimated_floor(isMeasurement);
    T.actual_floor(validActualFloor);
    T.calibrated_floor(validCalFloor)
];

allFloors = unique(allFloors(~isnan(allFloors)));

if ~isempty(allFloors)
    yticks(sort(allFloors));
    ylim([min(allFloors)-0.5, max(allFloors)+0.5]);
end

%% Second figure: offset detail

fig2 = figure("Name", "Offset Detail", "Color", "w");
fig2.Position = [150 150 1100 450];

plotWithGaps(T.time_min(isMeasurement), ...
             T.offset_from_estimated_floor_m(isMeasurement), ...
             maxGapMin, ...
             "Color", blue, ...
             "LineWidth", 1.8, ...
             "DisplayName", "Offset");

hold on;

yline(0, "--", ...
      "Perfect floor reference", ...
      "Color", dark, ...
      "LineWidth", 1.1, ...
      "DisplayName", "Zero offset");

validTestOffset = testRows & ~isnan(T.offset_from_estimated_floor_m);
if any(validTestOffset)
    scatter(T.time_min(validTestOffset), ...
            T.offset_from_estimated_floor_m(validTestOffset), ...
            markerSize, ...
            purple, ...
            "filled", ...
            "DisplayName", "Test / actual floor");
end

validCalOffset = calRows;
if any(validCalOffset)
    scatter(T.time_min(validCalOffset), ...
            zeros(sum(validCalOffset), 1), ...
            markerSize + 25, ...
            orange, ...
            "diamond", ...
            "filled", ...
            "DisplayName", "Calibration");

    for i = find(validCalOffset)'
        text(T.time_min(i), ...
             0, ...
             " floor " + string(T.calibrated_floor(i)), ...
             "FontSize", 9, ...
             "Color", dark, ...
             "VerticalAlignment", "bottom", ...
             "HorizontalAlignment", "left");
    end
end

grid on;
box on;
xlim([startMin endMin]);
xlabel("Time [min]");
ylabel("Offset [m]");
title("Offset from estimated floor reference | " + rangeText);
legend("Location", "best");

applyNiceYLim(T.offset_from_estimated_floor_m(isMeasurement));

%% Print useful summary

fprintf("\n========== CSV SUMMARY ==========\n");
fprintf("File: %s\n", filename);
fprintf("Selected time range: %.2f to %.2f min\n", startMin, endMin);
fprintf("Rows total:          %d\n", height(T));
fprintf("Rows in range:       %d\n", sum(inTimeRange));
fprintf("Measurement rows:    %d\n", sum(isMeasurement));
fprintf("Calibration rows:    %d\n", sum(calRows));
fprintf("Test / actual rows:  %d\n", sum(testRows));
fprintf("Full duration:       %.2f min\n", max(T.time_min));

if any(isMeasurement)
    fprintf("\nPressure range:      %.2f to %.2f hPa\n", ...
        min(T.pressure_hPa(isMeasurement), [], "omitnan"), ...
        max(T.pressure_hPa(isMeasurement), [], "omitnan"));

    fprintf("Temperature range:   %.2f to %.2f °C\n", ...
        min(T.temperature_C(isMeasurement), [], "omitnan"), ...
        max(T.temperature_C(isMeasurement), [], "omitnan"));

    fprintf("Height range:        %.2f to %.2f m\n", ...
        min(T.relative_height_m(isMeasurement), [], "omitnan"), ...
        max(T.relative_height_m(isMeasurement), [], "omitnan"));

    fprintf("Offset range:        %.2f to %.2f m\n", ...
        min(T.offset_from_estimated_floor_m(isMeasurement), [], "omitnan"), ...
        max(T.offset_from_estimated_floor_m(isMeasurement), [], "omitnan"));
end

fprintf("=================================\n\n");

%% Optional save

savePlots = questdlg("Save pretty plots as PNG?", ...
                     "Save plots", ...
                     "Yes", "No", "No");

if strcmp(savePlots, "Yes")
    outputFolder = fullfile(path, "pretty_sensor_plots");

    if ~exist(outputFolder, "dir")
        mkdir(outputFolder);
    end

    safeRange = sprintf("_%.2f_to_%.2f_min", startMin, endMin);
    safeRange = strrep(safeRange, ".", "p");

    exportgraphics(fig, ...
        fullfile(outputFolder, "sensor_overview_pretty" + safeRange + ".png"), ...
        "Resolution", 250);

    exportgraphics(fig2, ...
        fullfile(outputFolder, "offset_detail_pretty" + safeRange + ".png"), ...
        "Resolution", 250);

    fprintf("Saved plots to:\n%s\n", outputFolder);
end

%% Local helper functions

function plotWithGaps(x, y, maxGap, varargin)
    x = x(:);
    y = y(:);

    valid = ~isnan(x) & ~isnan(y);
    x = x(valid);
    y = y(valid);

    if isempty(x)
        builtin("plot", NaN, NaN, varargin{:});
        return;
    end

    [x, idx] = sort(x);
    y = y(idx);

    gapIdx = find(diff(x) > maxGap);

    xPlot = x;
    yPlot = y;

    for k = numel(gapIdx):-1:1
        i = gapIdx(k);
        xPlot = [xPlot(1:i); NaN; xPlot(i+1:end)];
        yPlot = [yPlot(1:i); NaN; yPlot(i+1:end)];
    end

    builtin("plot", xPlot, yPlot, varargin{:});
end

function stairsWithGaps(x, y, maxGap, varargin)
    x = x(:);
    y = y(:);

    valid = ~isnan(x) & ~isnan(y);
    x = x(valid);
    y = y(valid);

    if isempty(x)
        stairs(NaN, NaN, varargin{:});
        return;
    end

    [x, idx] = sort(x);
    y = y(idx);

    gapIdx = find(diff(x) > maxGap);

    holdState = ishold;
    hold on;

    startIdx = 1;

    for k = 1:numel(gapIdx)
        stopIdx = gapIdx(k);
        stairs(x(startIdx:stopIdx), y(startIdx:stopIdx), varargin{:});
        startIdx = stopIdx + 1;
    end

    stairs(x(startIdx:end), y(startIdx:end), varargin{:});

    if ~holdState
        hold off;
    end
end

function applyNiceYLim(y)
    y = y(~isnan(y));

    if isempty(y)
        return;
    end

    ymin = min(y);
    ymax = max(y);

    if ymin == ymax
        padding = max(abs(ymin) * 0.05, 1);
    else
        padding = 0.08 * (ymax - ymin);
    end

    ylim([ymin - padding, ymax + padding]);
end