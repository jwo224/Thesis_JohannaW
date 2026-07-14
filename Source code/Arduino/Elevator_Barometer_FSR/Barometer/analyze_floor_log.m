%% analyze_floor_log.m
% Analysis and plotting for ESP32-C3 BMP390L floor detection CSV
%
% CSV columns:
% type,time_ms,temperature_C,pressure_Pa,pressure_hPa,relative_height_m,
% estimated_floor,offset_from_estimated_floor_m,actual_floor,correct,calibrated_floor

clear; clc; close all;

%% Load CSV

[file, path] = uigetfile("*.csv", "Select floor_log.csv");

if isequal(file, 0)
    error("No file selected.");
end

filename = fullfile(path, file);
T = readtable(filename);

%% Force numeric columns to numeric

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
    varName = numericVars(k);

    if ismember(varName, T.Properties.VariableNames)
        if iscell(T.(varName))
            T.(varName) = str2double(string(T.(varName)));
        elseif isstring(T.(varName))
            T.(varName) = str2double(T.(varName));
        elseif ischar(T.(varName))
            T.(varName) = str2double(string(T.(varName)));
        end
    end
end

%% Ignore negative values
% Negative pressure, temperature, relative height and offset values are ignored.
% They are converted to NaN, so they are not plotted or used in statistics.

ignoreNegativeVars = [
    "temperature_C"
    "pressure_Pa"
    "pressure_hPa"
    "relative_height_m"
    "offset_from_estimated_floor_m"
];

for k = 1:numel(ignoreNegativeVars)
    varName = ignoreNegativeVars(k);

    if ismember(varName, T.Properties.VariableNames)
        T.(varName)(T.(varName) < 0) = NaN;
    end
end

% Remove rows where time_ms is missing or invalid
T = T(~isnan(T.time_ms), :);

disp("Loaded file:");
disp(filename);

%% Basic cleanup

T.time_s = (T.time_ms - T.time_ms(1)) / 1000;
T.time_min = T.time_s / 60;

% Support old CSVs without type column
if ismember("type", T.Properties.VariableNames)
    rowType = string(T.type);
else
    rowType = strings(height(T), 1);
    rowType(:) = "read";
end

isRead = rowType == "read";
isTest = rowType == "test";
isCalibration = rowType == "calibration";
isMeasurement = isRead | isTest;

% Support missing columns
if ismember("actual_floor", T.Properties.VariableNames)
    hasActual = ~isnan(T.actual_floor);
else
    hasActual = false(height(T), 1);
end

if ismember("correct", T.Properties.VariableNames)
    hasCorrect = ~isnan(T.correct);
else
    hasCorrect = false(height(T), 1);
end

if ismember("calibrated_floor", T.Properties.VariableNames)
    hasCalFloor = ~isnan(T.calibrated_floor);
else
    hasCalFloor = false(height(T), 1);
end

calRows = isCalibration & hasCalFloor;

correctLogical = false(height(T), 1);
if ismember("correct", T.Properties.VariableNames)
    correctLogical(hasCorrect) = T.correct(hasCorrect) == 1;
end

%% Print general statistics

fprintf("\n========== GENERAL STATISTICS ==========\n");
fprintf("Samples total:              %d\n", height(T));
fprintf("Measurement samples:        %d\n", sum(isMeasurement));
fprintf("Calibration points:         %d\n", sum(calRows));
fprintf("Duration:                   %.2f min\n", max(T.time_min));
fprintf("Samples with actual floor:  %d\n", sum(hasActual));

if any(isMeasurement)
    fprintf("\nTemperature, measurement rows:\n");
    fprintf("  Mean: %.2f °C\n", mean(T.temperature_C(isMeasurement), "omitnan"));
    fprintf("  Min:  %.2f °C\n", min(T.temperature_C(isMeasurement), [], "omitnan"));
    fprintf("  Max:  %.2f °C\n", max(T.temperature_C(isMeasurement), [], "omitnan"));
    fprintf("  Std:  %.3f °C\n", std(T.temperature_C(isMeasurement), "omitnan"));

    fprintf("\nPressure, measurement rows:\n");
    fprintf("  Mean: %.2f hPa\n", mean(T.pressure_hPa(isMeasurement), "omitnan"));
    fprintf("  Min:  %.2f hPa\n", min(T.pressure_hPa(isMeasurement), [], "omitnan"));
    fprintf("  Max:  %.2f hPa\n", max(T.pressure_hPa(isMeasurement), [], "omitnan"));
    fprintf("  Std:  %.4f hPa\n", std(T.pressure_hPa(isMeasurement), "omitnan"));

    validPressureRows = isMeasurement & ~isnan(T.pressure_Pa);

    if any(validPressureRows)
        firstPressureIdx = find(validPressureRows, 1, "first");
        lastPressureIdx  = find(validPressureRows, 1, "last");

        pressureDriftPa = T.pressure_Pa(lastPressureIdx) - T.pressure_Pa(firstPressureIdx);
        fprintf("  Drift first to last valid measurement: %.2f Pa\n", pressureDriftPa);
    else
        fprintf("  Drift first to last valid measurement: unavailable\n");
    end

    fprintf("\nRelative height, measurement rows:\n");
    fprintf("  Mean: %.2f m\n", mean(T.relative_height_m(isMeasurement), "omitnan"));
    fprintf("  Min:  %.2f m\n", min(T.relative_height_m(isMeasurement), [], "omitnan"));
    fprintf("  Max:  %.2f m\n", max(T.relative_height_m(isMeasurement), [], "omitnan"));
    fprintf("  Std:  %.3f m\n", std(T.relative_height_m(isMeasurement), "omitnan"));

    fprintf("\nOffset from estimated floor, measurement rows:\n");
    fprintf("  Mean absolute offset: %.3f m\n", mean(abs(T.offset_from_estimated_floor_m(isMeasurement)), "omitnan"));
    fprintf("  Median absolute offset: %.3f m\n", median(abs(T.offset_from_estimated_floor_m(isMeasurement)), "omitnan"));
    fprintf("  Max absolute offset: %.3f m\n", max(abs(T.offset_from_estimated_floor_m(isMeasurement)), [], "omitnan"));
end

testedRows = hasActual & hasCorrect;

if any(testedRows)
    accuracy = mean(correctLogical(testedRows)) * 100;
    floorError = T.estimated_floor(testedRows) - T.actual_floor(testedRows);

    fprintf("\nFloor classification test:\n");
    fprintf("  Tested samples: %d\n", sum(testedRows));
    fprintf("  Correct samples: %d\n", sum(correctLogical(testedRows)));
    fprintf("  Accuracy: %.2f %%\n", accuracy);
    fprintf("  Mean floor error: %.3f floors\n", mean(floorError, "omitnan"));
    fprintf("  Mean absolute floor error: %.3f floors\n", mean(abs(floorError), "omitnan"));
    fprintf("  Max absolute floor error: %.0f floors\n", max(abs(floorError), [], "omitnan"));
else
    fprintf("\nNo actual floor test labels found. Accuracy statistics skipped.\n");
end

fprintf("========================================\n\n");

%% Calibration overview table

if any(calRows)
    calTable = T(calRows, :);
    calTable = sortrows(calTable, "calibrated_floor");

    fprintf("\n========== CALIBRATION POINTS ==========\n");
    disp(calTable(:, ["calibrated_floor", "time_s", "pressure_Pa", "pressure_hPa", "relative_height_m", "temperature_C"]));
end

%% Plot settings

gapLimitMin = 0.5;
% Break plotted lines when the time gap is larger than this.
% Example:
% 0.2 = stricter
% 0.5 = default
% 1.0 = only break after gaps longer than 1 minute

%% Plot 1: pressure and temperature

figure("Name", "Pressure and Temperature", "Color", "w");

subplot(2,1,1);

xPressure = T.time_min(isMeasurement);
yPressure = T.pressure_hPa(isMeasurement);

plotWithGaps(xPressure, yPressure, gapLimitMin, "LineWidth", 1.2, "DisplayName", "Measurement");
hold on;

validCalPressure = calRows & ~isnan(T.pressure_hPa);

if any(validCalPressure)
    scatter(T.time_min(validCalPressure), ...
            T.pressure_hPa(validCalPressure), ...
            70, "diamond", "filled", ...
            "DisplayName", "Calibration point");
end

grid on;
xlabel("Time [min]");
ylabel("Pressure [hPa]");
title("Pressure over time");
legend("Location", "best");

subplot(2,1,2);

xTemp = T.time_min(isMeasurement);
yTemp = T.temperature_C(isMeasurement);

plotWithGaps(xTemp, yTemp, gapLimitMin, "LineWidth", 1.2, "DisplayName", "Measurement");
hold on;

validCalTemp = calRows & ~isnan(T.temperature_C);

if any(validCalTemp)
    scatter(T.time_min(validCalTemp), ...
            T.temperature_C(validCalTemp), ...
            70, "diamond", "filled", ...
            "DisplayName", "Calibration point");
end

grid on;
xlabel("Time [min]");
ylabel("Temperature [°C]");
title("Temperature over time");
legend("Location", "best");

%% Plot 2: relative height with calibrated floor lines

figure("Name", "Relative Height With Calibrated Floors", "Color", "w");

xHeight = T.time_min(isMeasurement);
yHeight = T.relative_height_m(isMeasurement);

plotWithGaps(xHeight, yHeight, gapLimitMin, "LineWidth", 1.2, "DisplayName", "Measured height");
hold on;
grid on;

xlabel("Time [min]");
ylabel("Relative height [m]");
title("Relative height with calibrated floor references");

if any(calRows)
    calFloors = T.calibrated_floor(calRows);
    calHeights = T.relative_height_m(calRows);

    validCalHeightLocal = ~isnan(calFloors) & ~isnan(calHeights);
    calFloors = calFloors(validCalHeightLocal);
    calHeights = calHeights(validCalHeightLocal);

    uniqueFloors = sort(unique(calFloors));

    for k = 1:numel(uniqueFloors)
        f = uniqueFloors(k);

        localIdx = find(calFloors == f, 1, "last");
        h = calHeights(localIdx);

        yline(h, "--", "floor " + string(f), ...
            "LabelHorizontalAlignment", "left", ...
            "HandleVisibility", "off");
    end

    validCalHeight = calRows & ~isnan(T.relative_height_m);

    if any(validCalHeight)
        scatter(T.time_min(validCalHeight), ...
                T.relative_height_m(validCalHeight), ...
                90, "diamond", "filled", ...
                "DisplayName", "Calibration points");

        for i = find(validCalHeight)'
            label = " floor" + string(T.calibrated_floor(i));
            text(T.time_min(i), T.relative_height_m(i), label, ...
                "VerticalAlignment", "bottom", ...
                "HorizontalAlignment", "left");
        end
    end
end

validActualHeight = hasActual & ~isnan(T.relative_height_m);

if any(validActualHeight)
    scatter(T.time_min(validActualHeight), ...
            T.relative_height_m(validActualHeight), ...
            55, "filled", ...
            "DisplayName", "Manual test points");
end

legend("Location", "best");

%% Plot 3: estimated floor over time

figure("Name", "Estimated Floor", "Color", "w");

xFloor = T.time_min(isMeasurement);
yFloor = T.estimated_floor(isMeasurement);

stairsWithGaps(xFloor, yFloor, gapLimitMin, "LineWidth", 1.4, "DisplayName", "Estimated floor");
hold on;

validActualFloor = hasActual & ~isnan(T.actual_floor);

if any(validActualFloor)
    scatter(T.time_min(validActualFloor), ...
            T.actual_floor(validActualFloor), ...
            60, "filled", ...
            "DisplayName", "Actual floor input");
end

grid on;
xlabel("Time [min]");
ylabel("Floor");
title("Estimated floor over time");
legend("Location", "best");

allFloors = T.estimated_floor(isMeasurement);

if any(validActualFloor)
    allFloors = [allFloors; T.actual_floor(validActualFloor)];
end

allFloors = unique(allFloors(~isnan(allFloors)));

if ~isempty(allFloors)
    yticks(sort(allFloors));
end

%% Plot 4: offset from estimated floor with calibration markings

figure("Name", "Offset From Estimated Floor", "Color", "w");

xOffset = T.time_min(isMeasurement);
yOffset = T.offset_from_estimated_floor_m(isMeasurement);

plotWithGaps(xOffset, yOffset, gapLimitMin, "LineWidth", 1.2, "DisplayName", "Offset");
hold on;

yline(0, "--", "Perfect floor reference", "DisplayName", "Zero offset");

grid on;
xlabel("Time [min]");
ylabel("Offset [m]");
title("Offset from estimated floor reference height");

validActualOffset = hasActual & ~isnan(T.offset_from_estimated_floor_m);

if any(validActualOffset)
    scatter(T.time_min(validActualOffset), ...
            T.offset_from_estimated_floor_m(validActualOffset), ...
            60, "filled", ...
            "DisplayName", "Manual actual-floor test");
end

validCalOffset = calRows;

if any(validCalOffset)
    scatter(T.time_min(validCalOffset), ...
            zeros(sum(validCalOffset), 1), ...
            90, "diamond", "filled", ...
            "DisplayName", "Calibration points");

    for i = find(validCalOffset)'
        label = " floor" + string(T.calibrated_floor(i));
        text(T.time_min(i), 0, label, ...
            "VerticalAlignment", "bottom", ...
            "HorizontalAlignment", "left");
    end
end

validOffsetPlot = isMeasurement & ~isnan(T.offset_from_estimated_floor_m);

if any(validOffsetPlot)
    ylim padded;
end

legend("Location", "best");

%% Plot 5: estimated vs actual floor, only test points

if any(testedRows)
    validTestPlot = testedRows & ...
                    ~isnan(T.estimated_floor) & ...
                    ~isnan(T.actual_floor);

    if any(validTestPlot)
        figure("Name", "Estimated vs Actual Floor at Test Points", "Color", "w");

        plot(T.time_min(validTestPlot), ...
             T.estimated_floor(validTestPlot), ...
             "-o", "LineWidth", 1.2, ...
             "DisplayName", "Estimated floor");
        hold on;

        plot(T.time_min(validTestPlot), ...
             T.actual_floor(validTestPlot), ...
             "-x", "LineWidth", 1.2, ...
             "DisplayName", "Actual floor");

        grid on;
        xlabel("Time [min]");
        ylabel("Floor");
        title("Estimated vs actual floor at manual test points");
        legend("Location", "best");

        allFloors = unique([T.estimated_floor(validTestPlot); T.actual_floor(validTestPlot)]);
        allFloors = allFloors(~isnan(allFloors));

        if ~isempty(allFloors)
            yticks(sort(allFloors));
        end
    end
end

%% Plot 6: floor error when actual labels exist

if any(testedRows)
    validErrorRows = testedRows & ...
                     ~isnan(T.estimated_floor) & ...
                     ~isnan(T.actual_floor);

    if any(validErrorRows)
        floorError = T.estimated_floor(validErrorRows) - T.actual_floor(validErrorRows);

        figure("Name", "Floor Error", "Color", "w");

        stem(T.time_min(validErrorRows), floorError, "filled", "LineWidth", 1.2);
        hold on;
        yline(0, "--");
        grid on;

        xlabel("Time [min]");
        ylabel("Estimated - Actual floor");
        title("Floor estimation error at manual test points");
    end
end

%% Plot 7: confusion matrix

if any(testedRows)
    validConfusionRows = testedRows & ...
                         ~isnan(T.actual_floor) & ...
                         ~isnan(T.estimated_floor);

    if any(validConfusionRows)
        actual = T.actual_floor(validConfusionRows);
        estimated = T.estimated_floor(validConfusionRows);

        allClasses = sort(unique([actual; estimated]));
        C = confusionmat(actual, estimated, "Order", allClasses);

        figure("Name", "Confusion Matrix", "Color", "w");
        confusionchart(C, string(allClasses));
        title("Confusion matrix: actual floor vs estimated floor");
    end
end

%% Plot 8: relative height distribution by estimated floor

validBoxRows = isMeasurement & ...
               ~isnan(T.relative_height_m) & ...
               ~isnan(T.estimated_floor);

if any(validBoxRows)
    figure("Name", "Height Distribution by Estimated Floor", "Color", "w");

    boxplot(T.relative_height_m(validBoxRows), T.estimated_floor(validBoxRows));
    grid on;
    xlabel("Estimated floor");
    ylabel("Relative height [m]");
    title("Relative height distribution by estimated floor");
end

%% Per-floor statistics

fprintf("\n========== PER-FLOOR STATISTICS ==========\n");

validStatsRows = isMeasurement & ~isnan(T.estimated_floor);

if any(validStatsRows)
    G = findgroups(T.estimated_floor(validStatsRows));

    floorStats = table;
    floorStats.estimated_floor = splitapply(@(x) x(1), T.estimated_floor(validStatsRows), G);
    floorStats.n_samples = splitapply(@numel, T.estimated_floor(validStatsRows), G);

    floorStats.height_mean_m = splitapply(@(x) mean(x, "omitnan"), T.relative_height_m(validStatsRows), G);
    floorStats.height_std_m  = splitapply(@(x) std(x, "omitnan"),  T.relative_height_m(validStatsRows), G);
    floorStats.height_min_m  = splitapply(@(x) min(x, [], "omitnan"), T.relative_height_m(validStatsRows), G);
    floorStats.height_max_m  = splitapply(@(x) max(x, [], "omitnan"), T.relative_height_m(validStatsRows), G);

    floorStats.pressure_mean_hPa = splitapply(@(x) mean(x, "omitnan"), T.pressure_hPa(validStatsRows), G);
    floorStats.pressure_std_hPa  = splitapply(@(x) std(x, "omitnan"),  T.pressure_hPa(validStatsRows), G);
    floorStats.pressure_min_hPa  = splitapply(@(x) min(x, [], "omitnan"), T.pressure_hPa(validStatsRows), G);
    floorStats.pressure_max_hPa  = splitapply(@(x) max(x, [], "omitnan"), T.pressure_hPa(validStatsRows), G);

    floorStats.temp_mean_C = splitapply(@(x) mean(x, "omitnan"), T.temperature_C(validStatsRows), G);
    floorStats.temp_std_C  = splitapply(@(x) std(x, "omitnan"),  T.temperature_C(validStatsRows), G);
    floorStats.temp_min_C  = splitapply(@(x) min(x, [], "omitnan"), T.temperature_C(validStatsRows), G);
    floorStats.temp_max_C  = splitapply(@(x) max(x, [], "omitnan"), T.temperature_C(validStatsRows), G);

    disp(floorStats);
else
    disp("No valid measurement rows for per-floor statistics.");
end

%% Accuracy by actual floor

if any(testedRows)
    actualVals = T.actual_floor(testedRows);
    correctVals = correctLogical(testedRows);

    validAccRows = ~isnan(actualVals);

    if any(validAccRows)
        actualVals = actualVals(validAccRows);
        correctVals = correctVals(validAccRows);

        Gacc = findgroups(actualVals);

        accByFloor = table;
        accByFloor.actual_floor = splitapply(@(x) x(1), actualVals, Gacc);
        accByFloor.n_tests = splitapply(@numel, actualVals, Gacc);
        accByFloor.n_correct = splitapply(@sum, double(correctVals), Gacc);
        accByFloor.accuracy_percent = 100 * accByFloor.n_correct ./ accByFloor.n_tests;

        fprintf("\n========== ACCURACY BY ACTUAL FLOOR ==========\n");
        disp(accByFloor);
    end
end

%% Optional: save all figures as PNG

saveFigures = questdlg("Save all figures as PNG files?", ...
    "Save figures", ...
    "Yes", "No", "No");

if strcmp(saveFigures, "Yes")
    outputFolder = fullfile(path, "floor_log_plots");

    if ~exist(outputFolder, "dir")
        mkdir(outputFolder);
    end

    figs = findall(0, "Type", "figure");

    for i = 1:numel(figs)
        fig = figs(i);
        figName = string(fig.Name);

        if strlength(figName) == 0
            figName = "Figure_" + i;
        end

        safeName = regexprep(figName, "[^\w\d-]", "_");
        exportgraphics(fig, fullfile(outputFolder, safeName + ".png"), "Resolution", 200);
    end

    fprintf("\nSaved plots to:\n%s\n", outputFolder);
end

%% Local helper functions
% These must stay at the end of the script.

function plotWithGaps(x, y, gapLimit, varargin)
    % Plots x/y data but breaks the line when time gaps are large.

    x = x(:);
    y = y(:);

    valid = ~isnan(x) & ~isnan(y);
    x = x(valid);
    y = y(valid);

    if isempty(x)
        plot(NaN, NaN, varargin{:});
        return;
    end

    [x, sortIdx] = sort(x);
    y = y(sortIdx);

    gapIdx = find(diff(x) > gapLimit);

    xPlot = x;
    yPlot = y;

    for k = numel(gapIdx):-1:1
        idx = gapIdx(k);

        xPlot = [xPlot(1:idx); NaN; xPlot(idx+1:end)];
        yPlot = [yPlot(1:idx); NaN; yPlot(idx+1:end)];
    end

    plot(xPlot, yPlot, varargin{:});
end

function stairsWithGaps(x, y, gapLimit, varargin)
    % Creates a stairs plot but breaks the line when time gaps are large.

    x = x(:);
    y = y(:);

    valid = ~isnan(x) & ~isnan(y);
    x = x(valid);
    y = y(valid);

    if isempty(x)
        stairs(NaN, NaN, varargin{:});
        return;
    end

    [x, sortIdx] = sort(x);
    y = y(sortIdx);

    gapIdx = find(diff(x) > gapLimit);

    startIdx = 1;

    holdState = ishold;
    hold on;

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