function floor_gui_matlab
% MATLAB GUI for ESP32-C3 floor detection via Serial
% Robust serial read version

clc;

app = struct();
app.serialObj = [];
app.timerObj = [];
app.csvCapture = false;
app.csvLines = strings(0,1);
app.rxBuffer = "";

%% GUI

app.fig = uifigure( ...
    "Name", "ESP32-C3 Floor Detection GUI", ...
    "Position", [100 100 1100 700]);

mainGrid = uigridlayout(app.fig, [2 2]);
mainGrid.RowHeight = {60, "1x"};
mainGrid.ColumnWidth = {260, "1x"};
mainGrid.Padding = [10 10 10 10];

%% Connection panel

connectionPanel = uipanel(mainGrid, "Title", "Connection");
connectionPanel.Layout.Row = 1;
connectionPanel.Layout.Column = [1 2];

connectionGrid = uigridlayout(connectionPanel, [1 9]);
connectionGrid.ColumnWidth = {70, 150, 80, 90, 100, 100, 100, "1x", 120};

uilabel(connectionGrid, "Text", "COM Port:");

ports = getSerialPorts();
app.portDropDown = uidropdown(connectionGrid, ...
    "Items", ports, ...
    "Value", ports(1));

uibutton(connectionGrid, "push", ...
    "Text", "Refresh", ...
    "ButtonPushedFcn", @(src,event) refreshPorts());

uilabel(connectionGrid, "Text", "Baud:");

app.baudField = uieditfield(connectionGrid, "numeric", ...
    "Value", 115200, ...
    "Limits", [1 Inf], ...
    "RoundFractionalValues", "on");

uibutton(connectionGrid, "push", ...
    "Text", "Connect", ...
    "ButtonPushedFcn", @(src,event) connectSerial());

uibutton(connectionGrid, "push", ...
    "Text", "Disconnect", ...
    "ButtonPushedFcn", @(src,event) disconnectSerial());

uibutton(connectionGrid, "push", ...
    "Text", "Ping help", ...
    "ButtonPushedFcn", @(src,event) sendCommand("help"));

app.statusLabel = uilabel(connectionGrid, ...
    "Text", "Not connected");

%% Command panel

cmdPanel = uipanel(mainGrid, "Title", "Commands");
cmdPanel.Layout.Row = 2;
cmdPanel.Layout.Column = 1;

cmdGrid = uigridlayout(cmdPanel, [25 1]);
cmdGrid.RowHeight = repmat({28}, 1, 25);
cmdGrid.Padding = [10 10 10 10];

uilabel(cmdGrid, "Text", "Calibration", "FontWeight", "bold");

for f = 0:5
    addCommandButton(cmdGrid, sprintf("Set floor %d", f), sprintf("floor%d", f));
end

addCommandButton(cmdGrid, "End calibration", "end");
addCommandButton(cmdGrid, "Overview", "overview");
addCommandButton(cmdGrid, "Reset calibration", "resetcal");

uilabel(cmdGrid, "Text", "Testing", "FontWeight", "bold");

for f = 0:5
    addCommandButton(cmdGrid, sprintf("Actual floor %d", f), sprintf("actual%d", f));
end

uilabel(cmdGrid, "Text", "CSV / Log", "FontWeight", "bold");

uibutton(cmdGrid, "push", ...
    "Text", "Dump CSV", ...
    "ButtonPushedFcn", @(src,event) dumpCSV());

uibutton(cmdGrid, "push", ...
    "Text", "Save dumped CSV", ...
    "ButtonPushedFcn", @(src,event) saveCSV());

addCommandButton(cmdGrid, "Clear ESP log", "clearlog");
addCommandButton(cmdGrid, "Help", "help");

%% Serial monitor panel

rightPanel = uipanel(mainGrid, "Title", "Serial Monitor");
rightPanel.Layout.Row = 2;
rightPanel.Layout.Column = 2;

rightGrid = uigridlayout(rightPanel, [3 1]);
rightGrid.RowHeight = {"1x", 35, 35};
rightGrid.Padding = [10 10 10 10];

app.logArea = uitextarea(rightGrid, ...
    "Editable", "off", ...
    "Value", "Connect to the ESP32-C3, then click 'Ping help'.");

manualGrid = uigridlayout(rightGrid, [1 3]);
manualGrid.ColumnWidth = {120, "1x", 80};

uilabel(manualGrid, "Text", "Manual command:");

app.manualField = uieditfield(manualGrid, "text");

uibutton(manualGrid, "push", ...
    "Text", "Send", ...
    "ButtonPushedFcn", @(src,event) sendManual());

bottomGrid = uigridlayout(rightGrid, [1 3]);
bottomGrid.ColumnWidth = {150, 150, "1x"};

uibutton(bottomGrid, "push", ...
    "Text", "Clear GUI log", ...
    "ButtonPushedFcn", @(src,event) clearGUILog());

uibutton(bottomGrid, "push", ...
    "Text", "Save GUI log", ...
    "ButtonPushedFcn", @(src,event) saveGUILog());

app.csvStatusLabel = uilabel(bottomGrid, ...
    "Text", "CSV lines captured: 0");

app.fig.CloseRequestFcn = @(src,event) closeApp();

appendLog("[MATLAB] GUI ready.");

%% Button helper

    function addCommandButton(parent, label, command)
        cmdLocal = string(command);

        uibutton(parent, "push", ...
            "Text", label, ...
            "ButtonPushedFcn", @(src,event) sendCommand(cmdLocal));
    end

%% Serial port helpers

    function ports = getSerialPorts()
        try
            ports = serialportlist("available");
            if isempty(ports)
                ports = "No ports found";
            end
        catch
            ports = "No ports found";
        end
    end

    function refreshPorts()
        ports = getSerialPorts();
        app.portDropDown.Items = ports;
        app.portDropDown.Value = ports(1);
        appendLog("[MATLAB] Ports refreshed.");
    end

%% Connect / disconnect

    function connectSerial()
        port = string(app.portDropDown.Value);

        if port == "No ports found"
            uialert(app.fig, "No serial ports available.", "Connection error");
            return;
        end

        disconnectSerialSilent();

        try
            baud = app.baudField.Value;

            app.serialObj = serialport(port, baud, "Timeout", 0.05);
            configureTerminator(app.serialObj, "LF");
            flush(app.serialObj);

            app.rxBuffer = "";

            % Some ESP32-C3 boards need a short moment after opening USB CDC
            pause(1.5);

            app.timerObj = timer( ...
                "ExecutionMode", "fixedSpacing", ...
                "Period", 0.05, ...
                "TimerFcn", @(src,event) readSerialRaw());

            start(app.timerObj);

            app.statusLabel.Text = "Connected to " + port;
            appendLog("[MATLAB] Connected to " + port + " at " + baud + " baud.");
            appendLog("[MATLAB] Click 'Ping help'. If no ESP text appears, check COM port / USB CDC.");

        catch ME
            uialert(app.fig, ME.message, "Connection error");
        end
    end

    function disconnectSerial()
        disconnectSerialSilent();
        app.statusLabel.Text = "Disconnected";
        appendLog("[MATLAB] Disconnected.");
    end

    function disconnectSerialSilent()
        try
            if ~isempty(app.timerObj) && isvalid(app.timerObj)
                stop(app.timerObj);
                delete(app.timerObj);
            end
        catch
        end

        app.timerObj = [];

        try
            if ~isempty(app.serialObj)
                clear app.serialObj;
            end
        catch
        end

        app.serialObj = [];
    end

%% Robust serial reading

    function readSerialRaw()
        if isempty(app.serialObj)
            return;
        end

        try
            n = app.serialObj.NumBytesAvailable;

            if n <= 0
                return;
            end

            raw = read(app.serialObj, n, "char");
            raw = string(raw);

            app.rxBuffer = app.rxBuffer + raw;

            % Normalize line endings
            app.rxBuffer = replace(app.rxBuffer, sprintf('\r\n'), sprintf('\n'));
            app.rxBuffer = replace(app.rxBuffer, sprintf('\r'), sprintf('\n'));

            parts = split(app.rxBuffer, newline);

            % Process all complete lines except the last partial line
            if numel(parts) > 1
                for i = 1:numel(parts)-1
                    line = strtrim(parts(i));
                    handleIncomingLine(line);
                end

                app.rxBuffer = parts(end);
            end

        catch ME
            appendLog("[MATLAB] Serial read error: " + ME.message);
        end
    end

    function handleIncomingLine(line)
        if strlength(line) == 0
            return;
        end

        appendLog(line);

        if line == "========== CSV START =========="
            app.csvCapture = true;
            app.csvLines = strings(0,1);
            app.csvStatusLabel.Text = "CSV capture started";
            return;
        end

        if line == "========== CSV END =========="
            app.csvCapture = false;
            app.csvStatusLabel.Text = "CSV lines captured: " + numel(app.csvLines);
            appendLog("[MATLAB] CSV captured. Click 'Save dumped CSV'.");
            return;
        end

        if app.csvCapture
            if ~startsWith(line, "=")
                app.csvLines(end+1,1) = line;
                app.csvStatusLabel.Text = "CSV lines captured: " + numel(app.csvLines);
            end
        end
    end

%% Sending

    function sendCommand(cmd)
        if isempty(app.serialObj)
            uialert(app.fig, "Connect to the ESP32 first.", "Not connected");
            return;
        end

        try
            writeline(app.serialObj, string(cmd));
            appendLog("[MATLAB SENT] " + string(cmd));
        catch ME
            uialert(app.fig, ME.message, "Send error");
        end
    end

    function sendManual()
        cmd = string(strtrim(app.manualField.Value));

        if strlength(cmd) == 0
            return;
        end

        sendCommand(cmd);
        app.manualField.Value = "";
    end

%% CSV

    function dumpCSV()
        app.csvLines = strings(0,1);
        app.csvCapture = false;
        app.csvStatusLabel.Text = "CSV lines captured: 0";
        sendCommand("dumpcsv");
    end

    function saveCSV()
        if isempty(app.csvLines)
            uialert(app.fig, "No CSV captured yet. Click 'Dump CSV' first and wait until CSV END appears.", "No CSV");
            return;
        end

        [file, path] = uiputfile("floor_log.csv", "Save CSV log");

        if isequal(file, 0)
            return;
        end

        filename = fullfile(path, file);

        try
            fid = fopen(filename, "w");

            if fid < 0
                error("Could not open file for writing.");
            end

            for i = 1:numel(app.csvLines)
                fprintf(fid, "%s\n", app.csvLines(i));
            end

            fclose(fid);

            appendLog("[MATLAB] CSV saved to: " + filename);
            uialert(app.fig, "CSV saved successfully.", "Saved");

        catch ME
            try
                fclose(fid);
            catch
            end

            uialert(app.fig, ME.message, "Save error");
        end
    end

%% GUI log

    function appendLog(msg)
        if ~isfield(app, "logArea") || isempty(app.logArea)
            return;
        end

        old = string(app.logArea.Value);

        timestamp = string(datetime("now", "Format", "HH:mm:ss"));
        newLine = "[" + timestamp + "] " + string(msg);

        app.logArea.Value = [old; newLine];

        maxLines = 2000;

        if numel(app.logArea.Value) > maxLines
            app.logArea.Value = app.logArea.Value(end-maxLines+1:end);
        end

        drawnow limitrate;
    end

    function clearGUILog()
        app.logArea.Value = "";
    end

    function saveGUILog()
        [file, path] = uiputfile("serial_log.txt", "Save GUI serial log");

        if isequal(file, 0)
            return;
        end

        filename = fullfile(path, file);

        try
            lines = string(app.logArea.Value);
            writelines(lines, filename);
            uialert(app.fig, "GUI log saved successfully.", "Saved");
        catch ME
            uialert(app.fig, ME.message, "Save error");
        end
    end

%% Close

    function closeApp()
        disconnectSerialSilent();
        delete(app.fig);
    end

end