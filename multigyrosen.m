function varargout = multigyrosen(varargin)
% MULTIGYROSEN MATLAB code for multigyrosen.fig
%      MULTIGYROSEN, by itself, creates a new MULTIGYROSEN or raises the existing
%      singleton*.
%
%      H = MULTIGYROSEN returns the handle to a new MULTIGYROSEN or the handle to
%      the existing singleton*.
%
%      MULTIGYROSEN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MULTIGYROSEN.M with the given input arguments.
%
%      MULTIGYROSEN('Property','Value',...) creates a new MULTIGYROSEN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before multigyrosen_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to multigyrosen_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help multigyrosen

% Last Modified by GUIDE v2.5 26-Feb-2015 14:36:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @multigyrosen_OpeningFcn, ...
                   'gui_OutputFcn',  @multigyrosen_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT
end

% --- Executes just before multigyrosen is made visible.
function multigyrosen_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to multigyrosen (see VARARGIN)

% Choose default command line output for multigyrosen
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes multigyrosen wait for user response (see UIRESUME)
% uiwait(handles.multigyrosen);

global g_multigyrosen
g_multigyrosen = [];
setappdata(handles.multigyrosen, 'connected', 0);
setappdata(handles.multigyrosen, 'recording', 0);
setappdata(handles.multigyrosen, 'do_stop', 0);
setappdata(handles.multigyrosen, 'do_mark_event', 0);
set(handles.connect, 'String', 'Connect');
set(handles.start, 'String', 'Start');
set(handles.start, 'Enable', 'off');
set(handles.rezero, 'Enable', 'off');
set(handles.event, 'Enable', 'off');
set(handles.elap, 'String', '');
set(handles.line, 'String', '');
set(handles.line2, 'String', '');
set(handles.status, 'String', 'Click Connect to begin');
set(handles.event, 'String', 'Event #1');
set(handles.debug1, 'String', '');
set(handles.hint_forceclose, 'Visible', 'off');
set(handles.led_recording, 'ForeGroundColor', [0 0.1 0]);
set(handles.led_stopped, 'ForeGroundColor', [1 0 0]);
set(handles.led_sync, 'ForeGroundColor', [0.1 0.1 0]);
set(handles.led_synckid, 'ForeGroundColor', [0.1 0.1 0]);
set(handles.led_error, 'ForegroundColor', [0.1 0 0]);
set(handles.led_warning, 'ForegroundColor', [0.1 0.1 0]);
%set([handles.led_recording handles.led_stopped handles.led_sync handles.led_error handles.led_warning], 'Visible', 'off');
cla(handles.ax_gyr1);
cla(handles.ax_gyr2);
cla(handles.ax_gyr3);
cla(handles.ax_gon1);
cla(handles.ax_gon2);
set([handles.ax_gyr1 handles.ax_gyr2 handles.ax_gyr3 handles.ax_gon1 handles.ax_gon2], 'NextPlot', 'add');
ph_gyr1x = plot(handles.ax_gyr1, 0, 1, 'r-');
ph_gyr1y = plot(handles.ax_gyr1, 0, 2, 'g-');
ph_gyr1z = plot(handles.ax_gyr1, 0, 3, 'b-');
ph_gyr1m = plot(handles.ax_gyr1, 0, 4, 'm-', 'LineWidth', 2);
ph_gyr2x = plot(handles.ax_gyr2, 0, 1, 'r-');
ph_gyr2y = plot(handles.ax_gyr2, 0, 2, 'g-');
ph_gyr2z = plot(handles.ax_gyr2, 0, 3, 'b-');
ph_gyr2m = plot(handles.ax_gyr2, 0, 4, 'm-', 'LineWidth', 2);
ph_gyr3x = plot(handles.ax_gyr3, 0, 1, 'r-');
ph_gyr3y = plot(handles.ax_gyr3, 0, 2, 'g-');
ph_gyr3z = plot(handles.ax_gyr3, 0, 3, 'b-');
ph_gyr3m = plot(handles.ax_gyr3, 0, 4, 'm-', 'LineWidth', 2);
ph_gon1a = plot(handles.ax_gon1, 0, 1, 'r-');
ph_gon1b = plot(handles.ax_gon1, 0, 2, 'b-');
ph_gon1m = plot(handles.ax_gon1, 0, 4, 'm-', 'LineWidth', 2);
ph_gon2a = plot(handles.ax_gon2, 0, 1, 'r-');
ph_gon2b = plot(handles.ax_gon2, 0, 2, 'b-');
ph_gon2m = plot(handles.ax_gon2, 0, 4, 'm-', 'LineWidth', 2);
setappdata(handles.multigyrosen, 'plothandles', [ph_gyr1x ph_gyr1y ph_gyr1z ph_gyr2x ph_gyr2y ph_gyr2z ph_gyr3x ph_gyr3y ph_gyr3z ph_gon1a ph_gon1b ph_gon2a ph_gon2b ph_gyr1m ph_gyr2m ph_gyr3m ph_gon1m ph_gon2m]);
set([handles.sw_gyr1x handles.sw_gyr1y handles.sw_gyr1z handles.sw_gyr1m], 'Value', 1);
set([handles.sw_gyr2x handles.sw_gyr2y handles.sw_gyr2z handles.sw_gyr2m], 'Value', 1);
set([handles.sw_gyr3x handles.sw_gyr3y handles.sw_gyr3z handles.sw_gyr3m], 'Value', 1);
set([handles.sw_gon1a handles.sw_gon1b handles.sw_gon1m], 'Value', 1);
set([handles.sw_gon2a handles.sw_gon2b handles.sw_gon2m], 'Value', 1);
set(handles.cal_sensor, 'String', {'Gyro 1', 'Gyro 2', 'Gyro 3', 'Gonio 1', 'Gonio 2'}.');
set(handles.cal_model, 'String', {'Linear regression', 'Quadratic regression', 'Cubic regression'}.');
set(handles.cal_formula, 'String', '');
set(handles.cal_modelparms, 'String', '');
set(handles.errled_sen1, 'Visible', 'off');
set(handles.errled_sen2, 'Visible', 'off');
set(handles.errled_sen3, 'Visible', 'off');
set(handles.errled_sen4, 'Visible', 'off');
set(handles.errled_sen5, 'Visible', 'off');
CalibFile = [gettmpdir() filesep 'multigyrosen_calibrationdata.mat'];
if exist(CalibFile, 'file')
    load(CalibFile, 'CalibrationData');
    setappdata(handles.multigyrosen, 'CalibrationData', CalibrationData);
    cal_sensor_Callback(handles.cal_sensor, [], handles);
else
    setappdata(handles.multigyrosen, 'CalibrationData', {});
    populate_pointlist(handles, []);
end
set([handles.zerotext_gyr1x handles.zerotext_gyr1y handles.zerotext_gyr1z handles.zerotext_gyr2x handles.zerotext_gyr2y handles.zerotext_gyr2z handles.zerotext_gyr3x handles.zerotext_gyr3y handles.zerotext_gyr3z handles.zerotext_gon1a handles.zerotext_gon1b handles.zerotext_gon2a handles.zerotext_gon2b], 'Visible', 'off');
g_multigyrosen.SubjectID = '';
g_multigyrosen.SessionNum = '';
setmatlabtitle('MULTI GYRO');
end


% --- Outputs from this function are returned to the command line.
function varargout = multigyrosen_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

end


% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global g_multigyrosen

Recording = getappdata(handles.multigyrosen, 'recording');
cereal = getappdata(handles.multigyrosen, 'cereal');
if g_multigyrosen.KidAttached
    cerealkid = getappdata(handles.multigyrosen, 'cerealkid');
end
ChanNames = {    'Time'    'Sync'    'Gyro1x'    'Gyro1y'    'Gyro1z'    'Gyro2x'    'Gyro2y'    'Gyro2z'    'Gyro3x'    'Gyro3y'    'Gyro3z'    'Gonio1a'    'Gonio1b'    'Gonio2a'    'Gonio2b'    };
ChanNamesKid = {    'Time'    'Sync'    'Gyro3x'    'Gyro3y'    'Gyro3z'    };
datestring = datestr(clock, 'yyyymmdd_HHMMSS_FFF');
try
    InputBufferSize = cereal.InputBufferSize;
catch
    connect_Callback(handles.connect, [], handles);
    return
end

if ~Recording
    % If not recording
    Recording = 1;
    setappdata(handles.multigyrosen, 'recording', 1);
    set(handles.connect, 'Enable', 'off');
    %set(handles.subjectid, 'Enable', 'off');
    set(hObject, 'Enable', 'off');
    setappdata(handles.multigyrosen, 'do_mark_event', 0);
    set(handles.event, 'String', 'Event #1');
    set(handles.event, 'Enable', 'off');
    PATTERN = 'Time=(\d+) Syn=(\d+) G1x=(-?\d+) G1y=(-?\d+) G1z=(-?\d+) G2x=(-?\d+) G2y=(-?\d+) G2z=(-?\d+) G3x=(-?\d+) G3y=(-?\d+) G3z=(-?\d+) G4a=(-?\d+) G4b=(-?\d+) G5a=(-?\d+) G5b=(-?\d+)';
    RawData = zeros(120*60*512, 15, 'int32');
    RawDataKid = zeros(120*60*512, 5, 'int32');
    if g_multigyrosen.KidAttached
        PATTERN_Kid = 'Time=(\d+) Syn=(\d+) G1x=(-?\d+) G1y=(-?\d+) G1z=(-?\d+)';
    end
    EventData = false(120*60*512, 1);
    
    % Extra stuff
    GyroVel = zeros(120*60*512, 9, 'double');
    GyroPos = zeros(120*60*512, 9, 'double');
    GonioPos = zeros(120*60*512, 4, 'double');
    Time = zeros(120*60*512, 1, 'double');
    Angle = zeros(120*60*512, 5, 'double');
    
    g_multigyrosen.GyroPos = GyroPos;
    g_multigyrosen.GonioPos = GonioPos;
    g_multigyrosen.Time = Time;
    
    
    %
    k = 0;
    kk = 0;
    k_processed = 0;
    set(handles.status, 'String', 'Preparing to start recording');
    flushinput(cereal);
    if g_multigyrosen.KidAttached
        flushinput(cerealkid);
    end
    set(handles.line, 'String', '');
    set(handles.line2, 'String', '');
    try
        fwrite(cereal, 'g');
    catch
        set(handles.status, 'String', 'Arduino #1 is not responding to commands. Check electrical connection to gyro sensors and then reconnect.');
        set(handles.led_error, 'ForegroundColor', [1 0 0]);
        setappdata(handles.multigyrosen, 'recording', 0);
        Recording = 0;
        set(handles.connect, 'Enable', 'on');
        %set(handles.subjectid, 'Enable', 'on');
        set(hObject, 'Enable', 'on');
        return
    end
    if g_multigyrosen.KidAttached
        try
            fwrite(cerealkid, 'g');
        catch
            set(handles.status, 'String', 'Arduino #2 is not responding to commands. Check electrical connection to gyro sensors and then reconnect.');
            set(handles.led_error, 'ForegroundColor', [1 0 0]);
            setappdata(handles.multigyrosen, 'recording', 0);
            Recording = 0;
            set(handles.connect, 'Enable', 'on');
            %set(handles.subjectid, 'Enable', 'on');
            set(hObject, 'Enable', 'on');
            return
        end
    end
    
    tsent = tic;
    testgetsuccess = 0;
    testgetsuccess_kid = 0;
    datareceived = 0;
    datareceived_kid = 0;
    while ~(testgetsuccess && (testgetsuccess_kid || ~g_multigyrosen.KidAttached)) && toc(tsent) < 5
        if ~testgetsuccess && cereal.BytesAvailable
            datareceived = 1;
            LINE = fgetl(cereal);
            set(handles.line, 'String', LINE);
            b = regexp(LINE, PATTERN, 'tokens', 'once');
            if ~isempty(b)
                if str2double(b{1}) > 134217727
                    set(handles.status, 'String', 'Please reconnect before proceeding.');
                    setappdata(handles.multigyrosen, 'recording', 0);
                    Recording = 0;
                    set(handles.led_error, 'ForegroundColor', [1 0 0]);
                    set(handles.connect, 'Enable', 'on');
                    testgetsuccess = 0;
                    return
                else
                    testgetsuccess = 1;
                end
                
            end
        end
        
        if g_multigyrosen.KidAttached && ~testgetsuccess_kid && cerealkid.BytesAvailable
            datareceived_kid = 1;
            LINE_Kid = fgetl(cerealkid);
            set(handles.line2, 'String', LINE_Kid);
            b = regexp(LINE_Kid, PATTERN_Kid, 'tokens', 'once');
            if ~isempty(b)
                if str2double(b{1}) > 7200000
                    set(handles.status, 'String', 'Please reconnect before proceeding.');
                    setappdata(handles.multigyrosen, 'recording', 0);
                    Recording = 0;
                    set(handles.led_error, 'ForegroundColor', [1 0 0]);
                    testgetsuccess_kid = 0;
                    return
                else
                    testgetsuccess_kid = 1;
                end
                
            end
            
        end
        
        pause(0.05);
    end
    
    if ~datareceived
        set(handles.status, 'String', 'Arduino #1 is not responding to commands. Check electrical connection to gyro sensors and then reconnect.');
        set(handles.led_error, 'ForegroundColor', [1 0 0]);
        setappdata(handles.multigyrosen, 'recording', 0);
        Recording = 0;
        set(handles.connect, 'Enable', 'on');
        %set(handles.subjectid, 'Enable', 'on');
        set(hObject, 'Enable', 'on');
        return
    elseif ~testgetsuccess
        set(handles.status, 'String', 'Arduino #1 failed test pattern.');
        set(handles.led_error, 'ForegroundColor', [1 0 0]);
        setappdata(handles.multigyrosen, 'recording', 0);
        Recording = 0;
        set(handles.connect, 'Enable', 'on');
        %set(handles.subjectid, 'Enable', 'on');
        set(hObject, 'Enable', 'on');
        return
    end
    
    if g_multigyrosen.KidAttached
        if ~datareceived_kid
            set(handles.status, 'String', 'Arduino #2 is not responding to commands. Check electrical connection to gyro sensors and then reconnect.');
            set(handles.led_error, 'ForegroundColor', [1 0 0]);
            setappdata(handles.multigyrosen, 'recording', 0);
            Recording = 0;
            set(handles.connect, 'Enable', 'on');
            %set(handles.subjectid, 'Enable', 'on');
            set(hObject, 'Enable', 'on');
            return
        elseif ~testgetsuccess_kid
            set(handles.status, 'String', 'Arduino #2 failed test pattern.');
            set(handles.led_error, 'ForegroundColor', [1 0 0]);
            setappdata(handles.multigyrosen, 'recording', 0);
            Recording = 0;
            set(handles.connect, 'Enable', 'on');
            %set(handles.subjectid, 'Enable', 'on');
            set(hObject, 'Enable', 'on');
            return
        end
    end
    
    % Get the sensitivity level
    GyroGainFlat = g_multigyrosen.GyroGainFlat;
    
    
    set(handles.led_recording, 'ForeGroundColor', [0 1.0 0]);
    set(handles.led_stopped, 'ForeGroundColor', [0.1 0 0]);
    set(handles.led_error, 'ForegroundColor', [0.1 0 0]);
    set(handles.led_warning, 'ForegroundColor', [0.1 0.1 0]);
    fwrite(cereal, 'c');
    if g_multigyrosen.KidAttached
        fwrite(cerealkid, 'c');
    end
    % For Arduino #2, only the latest line is fetched
    tpong = tic;
    stopsent = 0;
    % Time=19403 Syn=0 G1x=145 G1y=-99 G1z=42 G2x=-177 G2y=-633 G2z=-16 G3x=-7168 G3y=15 G3z=-90 G4a=258 G4b=315 G5a=222 G5b=222
    
    tstart = tic;
    tdatain = tic;
    tdatain_kid = tic;
    set(handles.status, 'String', sprintf('Recording..\nRecord ID: %s', datestring));
    set(hObject, 'String', 'Stop');
    set(handles.connect, 'Enable', 'off');
    set(handles.rezero, 'Enable', 'off');
    %set(handles.subjectid, 'Enable', 'off');
    set([handles.zerotext_gyr1x handles.zerotext_gyr1y handles.zerotext_gyr1z handles.zerotext_gyr2x handles.zerotext_gyr2y handles.zerotext_gyr2z handles.zerotext_gyr3x handles.zerotext_gyr3y handles.zerotext_gyr3z handles.zerotext_gon1a handles.zerotext_gon1b handles.zerotext_gon2a handles.zerotext_gon2b], 'Visible', 'off');
    setappdata(handles.multigyrosen, 'do_mark_event', 0);
    set(handles.event, 'String', 'Event #1');
    set(handles.event, 'Enable', 'on');
    
    %donotmovedisplayed = 0;
    sensorzeroed = 1;
    buttonenabled = 0;
    recordtimewarned = 0;
    syncledon = 0;
    synckidledon = 0;
    firsttms = -1;
    elap = 0;
    ZeroMean = g_multigyrosen.ZeroMean;
    ZeroStd = g_multigyrosen.ZeroStd;
    %ZeroMean = zeros(1,13); % Mean | G1xyz G2xyz G3xyz Gonio1ab Gonio2ab
    phs = getappdata(handles.multigyrosen, 'plothandles');
    set(phs, 'XData', [], 'YData', []);
    set(handles.status, 'String', sprintf('Recording..\nRecord ID: %s', datestring));
    
    % main loop
    fprintf('Entering main loop..\n');
    
%     cereal.BytesAvailableFcn = @cereal_Callback;
    if g_multigyrosen.KidAttached
        cerealkid.BytesAvailableFcn = @cerealkid_Callback;
    end
    
    
    while 1
        datareceived = 0;
        datareceived_kid = 0;
        if ~stopsent
            if toc(tstart) > 119*60 || getappdata(handles.multigyrosen, 'do_stop')
                set(handles.status, 'String', 'Sending stop signal');
                set(hObject, 'Enable', 'off');
                try 
                    fwrite(cereal, 's');
                end
                if g_multigyrosen.KidAttached
                    try
                        fwrite(cerealkid, 's');
                    end
                end
                stopsent = 1;
            elseif ~recordtimewarned && toc(tstart) > 114*60
                recordtimewarned = 1;
                set(handles.status, 'String', 'Warning: Recording capacity is reaching limit');
                set(handles.led_warning, 'ForegroundColor', [1 1 0]);
            end
        end
        if toc(tpong) > 5
            try
                fwrite(cereal, 'n');
            end
            if g_multigyrosen.KidAttached
                try
                    fwrite(cerealkid, 'n');
                end
            end
            tpong = tic;
        end

        
        
% % Moved to cereal callback functions
        BytesAvailable = cereal.BytesAvailable;
        if BytesAvailable
            tdatain = tic;
            LINE = fgetl(cereal);
            set(handles.line, 'String', LINE);
            b = regexp(LINE, PATTERN, 'tokens', 'once');
            if ~isempty(b)
                k = k + 1;
                RawData(k,:) = str2double(b);
                datareceived = 1;
            else
                fprintf('LINE=%s', LINE);
            end
        end
%         
%         if g_multigyrosen.KidAttached
%             BytesAvailableKid = cerealkid.BytesAvailable;
%             if BytesAvailableKid
%                 while cerealkid.BytesAvailable
%                     tdatain_kid = tic;
%                     LINE_Kid = fgetl(cerealkid);
%                     set(handles.line2, 'String', LINE_Kid);
%                     b = regexp(LINE_Kid, PATTERN_Kid, 'tokens', 'once');
%                     if ~isempty(b)
%                         kk = kk + 1;
%                         RawDataKid(kk,:) = str2double(b);
%                         datareceived_kid = 1;
%                     else
%                         fprintf('LINE=%s', LINE);
%                     end
%                 end
%                 fwrite(cerealkid, 'g');
%             end
%         end
        
        
%         if k <= k_processed
%             pause(0.001);
%             continue
%         end
        
        if toc(tdatain) > 2
            set(handles.status, 'String', 'No incoming data from Arduino #1. Connection lost? Stopping.');
            fprintf('No incoming data from Arduino #1. Connection lost? Stopping.\n');
            if ~getappdata(handles.multigyrosen, 'do_stop')
                set(handles.led_error, 'ForegroundColor', [1 0 0]);
            end
            drawnow
            try fwrite(cereal, 's'); end
            try fwrite(cerealkid, 's'); end
            setappdata(handles.multigyrosen, 'do_stop', 0);
            break
        end
        
        
        if g_multigyrosen.KidAttached && toc(tdatain_kid) > 2
            set(handles.status, 'String', 'No incoming data from Arduino #2. Connection lost? Stopping.');
            fprintf('No incoming data from Arduino #2. Connection lost? Stopping.\n');
            if ~getappdata(handles.multigyrosen, 'do_stop')
                set(handles.led_error, 'ForegroundColor', [1 0 0]);
            end
            drawnow
            try fwrite(cereal, 's'); end
            try fwrite(cerealkid, 's'); end
            setappdata(handles.multigyrosen, 'do_stop', 0);
            break
        end
        
        
        
        if firsttms >= 0
            elap = (double(RawData(k,1)) - firsttms) / 1000.0;
            set(handles.elap, 'String', sprintf('%.2f ', elap));
        elseif k
            firsttms = double(RawData(1,1));
        end
        if getappdata(handles.multigyrosen, 'do_mark_event')
            setappdata(handles.multigyrosen, 'do_mark_event', 0);
            EventData(k) = 1;
            set(handles.event, 'String', sprintf('Event #%i',nnz(EventData)+1));
        end
        if ~buttonenabled && toc(tstart) > 1
            buttonenabled = 1;
            set(hObject, 'Enable', 'on');
        end
        
        
        % Extra stuff
        if (k && BytesAvailable < InputBufferSize / 2) && (~g_multigyrosen.KidAttached || (kk && BytesAvailableKid < InputBufferSize / 2))
            
            Time(k) = elap;
            
            GyroVel(k,:) = GyroGainFlat.*(double(RawData(k,3:11)) - ZeroMean(1:9));
            if g_multigyrosen.KidAttached
                GyroVel(k,7:9) = GyroGainFlat(7:9).*(double(RawDataKid(kk,3:5)) - ZeroMean(7:9));
            end
            
            if sensorzeroed && k > 1
                GonioPos(k,:) = (double(RawData(k,12:15)) - ZeroMean(10:13));
                GyroPos(k,:) = GyroPos(k-1,:) + GyroVel(k,:) * (Time(k) - Time(k-1));
            else
                GonioPos(k,:) = 0;
                GyroPos(k,:) = 0;
            end
            
            g_multigyrosen.GyroPos(k,:) = GyroPos(k,:);
            g_multigyrosen.GonioPos(k,:) = GonioPos(k,:);
            g_multigyrosen.Time(k,:) = Time(k,:);
            g_multigyrosen.k = k;
            Angle(k,:) = get_angles(handles);
            
            % Plot stuff
            
            %set(handles.debug1, 'String', [num2str(BytesAvailable) ' ' num2str(BytesAvailableKid)]);
            
            if mod(k,4) == 0 && BytesAvailable < 128 && (~g_multigyrosen.KidAttached || BytesAvailableKid < 128)
                
                
                gyr1x = get(handles.sw_gyr1x, 'Value');
                gyr1y = get(handles.sw_gyr1y, 'Value');
                gyr1z = get(handles.sw_gyr1z, 'Value');
                gyr2x = get(handles.sw_gyr2x, 'Value');
                gyr2y = get(handles.sw_gyr2y, 'Value');
                gyr2z = get(handles.sw_gyr2z, 'Value');
                gyr3x = get(handles.sw_gyr3x, 'Value');
                gyr3y = get(handles.sw_gyr3y, 'Value');
                gyr3z = get(handles.sw_gyr3z, 'Value');
                gon1a = get(handles.sw_gon1a, 'Value');
                gon1b = get(handles.sw_gon1b, 'Value');
                gon2a = get(handles.sw_gon2a, 'Value');
                gon2b = get(handles.sw_gon2b, 'Value');
                gyr1m = get(handles.sw_gyr1m, 'Value');
                gyr2m = get(handles.sw_gyr2m, 'Value');
                gyr3m = get(handles.sw_gyr3m, 'Value');
                gon1m = get(handles.sw_gon1m, 'Value');
                gon2m = get(handles.sw_gon2m, 'Value');
                
                if gyr1x || gyr1y || gyr1z || gyr2x || gyr2y || gyr2z || gyr3x || gyr3y || gyr3z || gon1a || gon1b || gon2a || gon2b || gyr1m || gyr2m || gyr3m || gon1m || gon2m
                    k1 = find(Time >= Time(k)-10, 1, 'first');
                    %[k1 k Time(k1) Time(k)]
                    
                    if ~isempty(k1) && ~isnan(k1) && Time(k) > Time(k1)
                        
                        if gyr1x || gyr1y || gyr1z || gyr1m
                            if gyr1x, set(phs(1), 'XData', Time(k1:k,1), 'YData', GyroPos(k1:k,1)); end
                            if gyr1y, set(phs(2), 'XData', Time(k1:k,1), 'YData', GyroPos(k1:k,2)); end
                            if gyr1z, set(phs(3), 'XData', Time(k1:k,1), 'YData', GyroPos(k1:k,3)); end
                            if gyr1m, set(phs(14), 'XData', Time(k1:k,1), 'YData', Angle(k1:k,1)); end
                            if k1 > 1
                                set(handles.ax_gyr1, 'XLim', Time([k1 k]));
                            else
                                set(handles.ax_gyr1, 'XLim', Time(k) + [-10 0]);
                            end
                        end
                        if gyr2x || gyr2y || gyr2z || gyr2m
                            if gyr2x, set(phs(4), 'XData', Time(k1:k,1), 'YData', GyroPos(k1:k,4)); end
                            if gyr2y, set(phs(5), 'XData', Time(k1:k,1), 'YData', GyroPos(k1:k,5)); end
                            if gyr2z, set(phs(6), 'XData', Time(k1:k,1), 'YData', GyroPos(k1:k,6)); end
                            if gyr2m, set(phs(15), 'XData', Time(k1:k,1), 'YData', Angle(k1:k,2)); end
                            if k1 > 1
                                set(handles.ax_gyr2, 'XLim', Time([k1 k]));
                            else
                                set(handles.ax_gyr2, 'XLim', Time(k) + [-10 0]);
                            end
                        end
                        if gyr3x || gyr3y || gyr3z || gyr3m
                            if gyr3x, set(phs(7), 'XData', Time(k1:k,1), 'YData', GyroPos(k1:k,7)); end
                            if gyr3y, set(phs(8), 'XData', Time(k1:k,1), 'YData', GyroPos(k1:k,8)); end
                            if gyr3z, set(phs(9), 'XData', Time(k1:k,1), 'YData', GyroPos(k1:k,9)); end
                            if gyr3m, set(phs(16), 'XData', Time(k1:k,1), 'YData', Angle(k1:k,3)); end
                            if k1 > 1
                                set(handles.ax_gyr3, 'XLim', Time([k1 k]));
                            else
                                set(handles.ax_gyr3, 'XLim', Time(k) + [-10 0]);
                            end
                        end
                        if gon1a || gon1b || gon1m
                            if gon1a, set(phs(10), 'XData', Time(k1:k,1), 'YData', GonioPos(k1:k,1)); end
                            if gon1b, set(phs(11), 'XData', Time(k1:k,1), 'YData', GonioPos(k1:k,2)); end
                            if gon1m, set(phs(17), 'XData', Time(k1:k,1), 'YData', Angle(k1:k,4)); end
                            if k1 > 1
                                set(handles.ax_gon1, 'XLim', Time([k1 k]));
                            else
                                set(handles.ax_gon1, 'XLim', Time(k) + [-10 0]);
                            end
                        end
                        if gon2a || gon2b || gon2m
                            if gon2a, set(phs(12), 'XData', Time(k1:k,1), 'YData', GonioPos(k1:k,3)); end
                            if gon2b, set(phs(13), 'XData', Time(k1:k,1), 'YData', GonioPos(k1:k,4)); end
                            if gon2m, set(phs(18), 'XData', Time(k1:k,1), 'YData', Angle(k1:k,5)); end
                            if k1 > 1
                                set(handles.ax_gon2, 'XLim', Time([k1 k]));
                            else
                                set(handles.ax_gon2, 'XLim', Time(k) + [-10 0]);
                            end
                        end
                    end
                end
                
                if ~syncledon && RawData(k,2)
                    set(handles.led_sync, 'ForeGroundColor', [1 1 0]);
                    syncledon = 1;
                elseif syncledon && ~RawData(k,2)
                    set(handles.led_sync, 'ForeGroundColor', [0.1 0.1 0]);
                    syncledon = 0;
                end
                if g_multigyrosen.KidAttached
                    if ~synckidledon && RawDataKid(kk,2)
                        set(handles.led_synckid, 'ForeGroundColor', [1 1 0]);
                        synckidledon = 1;
                    elseif synckidledon && ~RawDataKid(kk,2)
                        set(handles.led_synckid, 'ForeGroundColor', [0.1 0.1 0]);
                        synckidledon = 0;
                    end
                end
                drawnow
            end
        end
        
        k_processed = k;
        
    end
    % When loop exits, the recording ended
    LoopExitRoutine();
    
else
    % If recording
    setappdata(handles.multigyrosen, 'do_stop', 1);
    set(hObject, 'String', 'Stopping');
    set(hObject, 'Enable', 'off');
    set(handles.sessionnum, 'Enable', 'off');
    set(handles.subjectid, 'Enable', 'off');
    drawnow
end

    function cereal_Callback(cereal, eventdata)
        if Recording
            LINE = fgetl(cereal);
            BytesAvailable = cereal.BytesAvailable;
            tdatain = tic;
            set(handles.line, 'String', LINE);
            b1 = regexp(LINE, PATTERN, 'tokens', 'once');
            if ~isempty(b1)
                k = k + 1;
                RawData(k,:) = str2double(b1);
                datareceived = 1;
            elseif ~isempty(regexp(LINE, 'Device has entered sleep mode. To wake up, enter any valid command.', 'match', 'once'))
                try
                    LoopExitRoutine();
                catch
                    Filename1 = sprintf('%s%smultigyrosen_%s.mat', gettmpdir(), filesep, datestring);
                    save(Filename1);
                    set(handles.status, 'String', 'Emergency save: Data saved in temp dir');
                end
                
            else
                fprintf('LINE=%s', LINE);
            end
        end
    end

    function cerealkid_Callback(cerealkid, eventdata)
        if Recording
            LINE_Kid = fgetl(cerealkid);
            BytesAvailableKid = cerealkid.BytesAvailable;
            tdatain_kid = tic;
            set(handles.line2, 'String', LINE_Kid);
            b2 = regexp(LINE_Kid, PATTERN_Kid, 'tokens', 'once');
            if ~isempty(b2)
                kk = kk + 1;
                RawDataKid(kk,:) = str2double(b2);
                datareceived_kid = 1;
            elseif ~isempty(regexp(LINE_Kid, 'Device has entered sleep mode. To wake up, enter any valid command.', 'match', 'once'))
                try
                    LoopExitRoutine();
                catch
                    Filename1 = sprintf('%s%smultigyrosen_%s.mat', gettmpdir(), filesep, datestring);
                    save(Filename1);
                    set(handles.status, 'String', 'Emergency save: Data saved in temp dir');
                end
            else
                fprintf('LINE_Kid=%s', LINE_Kid);
            end
        end
    end

    function LoopExitRoutine ()
        set(handles.led_recording, 'ForeGroundColor', [0 0.1 0]);
        set(handles.led_stopped, 'ForeGroundColor', [1 0 0]);
        drawnow
        
        RawData = RawData(1:k,:);
        RawDataKid = RawDataKid(1:kk,:);
        EventData = EventData(1:k,:);
        assignin('base', 'MultiGyroSen_RawData', RawData);
        assignin('base', 'MultiGyroSen_RawDataKid', RawDataKid);
        assignin('base', 'MultiGyroSen_EventData', EventData);
        assignin('base', 'MultiGyroSen_ChanNames', ChanNames);
        assignin('base', 'MultiGyroSen_ChanNamesKid', ChanNamesKid);
        assignin('base', 'MultiGyroSen_Date', datestring);
        set(handles.status, 'String', 'Saving data');
        drawnow
        
        try
            % Compact variables first
            Angle = Angle(1:k,:);
            GonioPos = GonioPos(1:k,:);
            GyroPos = GyroPos(1:k,:);
            GyroVel = GyroVel(1:k,:);
            Time = Time(1:k,:);
            g_multigyrosen.GyroPos = g_multigyrosen.GyroPos(1:k,:);
            g_multigyrosen.GonioPos = g_multigyrosen.GonioPos(1:k,:);
            g_multigyrosen.Time = g_multigyrosen.Time(1:k,:);
        end
        
        CalibrationData = getappdata(handles.multigyrosen, 'CalibrationData');
        
        try
            Filename1 = sprintf('%s%smultigyrosen_%s.mat', gettmpdir(), filesep, datestring);
            save(Filename1);
            set(handles.status, 'String', 'Data saved in temp dir');
        end
        
        try
            CalibFile = [gettmpdir() filesep 'multigyrosen_calibrationdata.mat'];
            save(CalibFile, 'CalibrationData');
        end
        
        SubjectID = g_multigyrosen.SubjectID;
        SessionNum = g_multigyrosen.SessionNum;
        if ~isempty(SubjectID) && ~isempty(SessionNum)
            try
                matname2 = sprintf('%s-%sarduinomultigyrosen.mat', SubjectID, SessionNum);
                if g_multigyrosen.KidAttached
                    matname3 = sprintf('%s-%sarduinomultigyrosenkid.mat', SubjectID, SessionNum);
                    Filename3 = sprintf('%s%s%s', getdesktopdir(), filesep, matname3);
                end
                Filename2 = sprintf('%s%s%s', getdesktopdir(), filesep, matname2);
                if exist(Filename2, 'file')
                    matname2 = regexprep(matname2, 'arduinomultigyrosen\.mat$', sprintf('arduinomultigyrosen-%s.mat', datestring));
                    Filename2 = sprintf('%s%s%s', getdesktopdir(), filesep, matname2);
                    if g_multigyrosen.KidAttached
                        matname3 = regexprep(matname3, 'arduinomultigyrosenkid\.mat$', sprintf('arduinomultigyrosenkid-%s.mat', datestring));
                        Filename3 = sprintf('%s%s%s', getdesktopdir(), filesep, matname3);
                    end
                end
                save(Filename2, 'RawData', 'EventData', 'ChanNames', 'datestring', 'CalibrationData', 'ZeroMean', 'ZeroStd', 'GyroGainFlat', 'GyroPos', 'Angle', 'GonioPos', 'GyroVel');
                if g_multigyrosen.KidAttached
                    save(Filename3, 'RawDataKid', 'ChanNamesKid', 'datestring', 'CalibrationData', 'ZeroMean', 'ZeroStd', 'GyroGainFlat');
                end
                if g_multigyrosen.KidAttached
                    set(handles.status, 'String', sprintf('Data saved in desktop as\n%s and %s', matname2, matname3));
                else
                    set(handles.status, 'String', sprintf('Data saved in desktop as\n%s', matname2));
                end
                check_valid_savename(handles);
            catch
                set(handles.status, 'String', 'Error saving data. Still available in base workspace.');
                set(handles.led_error, 'ForegroundColor', [1 0 0]);
            end
        end
        
        set(handles.connect, 'Enable', 'on');
        set(handles.sessionnum, 'Enable', 'on');
        set(handles.subjectid, 'Enable', 'on');
        set(handles.rezero, 'Enable', 'on');
        set(hObject, 'Enable', 'on');
        set(hObject, 'String', 'Start');
        setappdata(handles.multigyrosen, 'recording', 0);
        Recording = 0;
        drawnow
    end

end


function zero_sensors (handles)
global g_multigyrosen
PATTERN = 'Time=(\d+) Syn=(\d+) G1x=(-?\d+) G1y=(-?\d+) G1z=(-?\d+) G2x=(-?\d+) G2y=(-?\d+) G2z=(-?\d+) G3x=(-?\d+) G3y=(-?\d+) G3z=(-?\d+) G4a=(-?\d+) G4b=(-?\d+) G5a=(-?\d+) G5b=(-?\d+)';
PATTERN_Kid = 'Time=(\d+) Syn=(\d+) G1x=(-?\d+) G1y=(-?\d+) G1z=(-?\d+)';
Recording = getappdata(handles.multigyrosen, 'recording');
if Recording
    return
end
cereal = getappdata(handles.multigyrosen, 'cereal');
if g_multigyrosen.KidAttached
    cerealkid = getappdata(handles.multigyrosen, 'cerealkid');
end
%datestring = datestr(clock, 'yyyymmdd_HHMMSS_FFF');
ZeroingTime = 3;
kzero = 5;
GyroGain = [0 0 0];
%InputBufferSize = cereal.InputBufferSize;

% Get the sensitivity level
flushinput(cereal);
if g_multigyrosen.KidAttached
    flushinput(cerealkid);
end
set(handles.line, 'String', '');
set(handles.line2, 'String', '');
fwrite(cereal, 'd');
if g_multigyrosen.KidAttached
    fwrite(cerealkid, 'd');
end
tsent = tic;
testgetsuccess = 0;
while ~testgetsuccess && toc(tsent) < 5
    if cereal.BytesAvailable
        LINE = fgetl(cereal);
        set(handles.line, 'String', LINE);
        b = regexp(LINE, '^GYRO_DEGPERDIGIT: G1=(\d+(\.\d+)?) G2=(\d+(\.\d+)?) G3=(\d+(\.\d+)?)', 'tokens', 'once');
        if ~isempty(b)
            testgetsuccess = 1;
            GyroGain(1) = str2double(b{1});
            GyroGain(2) = str2double(b{2});
            GyroGain(3) = str2double(b{3});
            break
        end
    end
    pause(0.05);
end
if ~testgetsuccess
    set(handles.status, 'String', 'Unable to obtain gyro gains.');
    set(handles.led_error, 'ForegroundColor', [1 0 0]);
    setappdata(handles.multigyrosen, 'recording', 0);
    return
else
    set(handles.status, 'String', sprintf('Gyro gains %g %g %g', GyroGain(1:3)));
end

if g_multigyrosen.KidAttached
    tsent = tic;
    testgetsuccesskid = 0;
    while ~testgetsuccesskid && toc(tsent) < 5
        if cerealkid.BytesAvailable
            LINE_Kid = fgetl(cerealkid);
            set(handles.line2, 'String', LINE_Kid);
            b = regexp(LINE_Kid, '^GYRO_DEGPERDIGIT: G1=(\d+(\.\d+)?)', 'tokens', 'once');
            if ~isempty(b)
                testgetsuccesskid = 1;
                GyroGain(3) = str2double(b{1});
                break
            end
        end
        pause(0.05);
    end
    if ~testgetsuccesskid
        set(handles.status, 'String', 'Unable to obtain gyro gains for kid.');
        set(handles.led_error, 'ForegroundColor', [1 0 0]);
        setappdata(handles.multigyrosen, 'recording', 0);
        return
    else
        set(handles.status, 'String', sprintf('Gyro gains %g %g %g', GyroGain(1:3)));
    end
end


GyroGainFlat = reshape([1 1 1]' * GyroGain,1,[]);
g_multigyrosen.GyroGain = GyroGain;
g_multigyrosen.GyroGainFlat = GyroGainFlat;

ZeroData = nan(5+ceil(ZeroingTime*100),15);
ZeroDataKid = nan(5+ceil(ZeroingTime*100),5);
% main loop
set(handles.status, 'String', 'Zeroing.. Do not move sensors.');
k = 0;
kk = 0;
tstart = tic;
while 1
    fwrite(cereal, 'g');
    if g_multigyrosen.KidAttached
        fwrite(cerealkid, 'g');
    end
    
    if cereal.BytesAvailable
        LINE = fgetl(cereal);
        set(handles.line, 'String', LINE);
        b = regexp(LINE, PATTERN, 'tokens', 'once');
        if ~isempty(b)
            k = k + 1;
            ZeroData(k,:) = str2double(b);
            if k >= 205 || toc(tstart) >= ZeroingTime
                break
            end
        end
    end
    
    if g_multigyrosen.KidAttached && cerealkid.BytesAvailable
        LINE_Kid = fgetl(cerealkid);
        set(handles.line2, 'String', LINE_Kid);
        b = regexp(LINE_Kid, PATTERN_Kid, 'tokens', 'once');
        if ~isempty(b)
            kk = kk + 1;
            ZeroDataKid(kk,:) = str2double(b);
            if (k >= 205 && kk >= 205) || toc(tstart) >= ZeroingTime
                break
            end
        end
    end
    
    pause(0.01);
end

ZeroMean = [ % Mean Std | G1xyz G2xyz G3xyz Gonio1ab Gonio2ab
    nanmean(double(ZeroData(kzero:k,3:5)))  nanmean(double(ZeroData(kzero:k,6:8)))  nanmean(double(ZeroData(kzero:k,9:11)))  nanmean(double(ZeroData(kzero:k,12:13)))  nanmean(double(ZeroData(kzero:k,14:15)))
    ];
ZeroStd = [ % Mean Std | G1xyz G2xyz G3xyz Gonio1ab Gonio2ab
    nanstd(double(ZeroData(kzero:k,3:5)))  nanstd(double(ZeroData(kzero:k,6:8)))  nanstd(double(ZeroData(kzero:k,9:11)))  nanstd(double(ZeroData(kzero:k,12:13)))  nanstd(double(ZeroData(kzero:k,14:15)))
    ];

if g_multigyrosen.KidAttached
ZeroMean(7:9) = [ 
    nanmean(double(ZeroDataKid(kzero:kk,3:5)))
    ];
ZeroStd(7:9) = [
    nanstd(double(ZeroDataKid(kzero:kk,3:5)))
    ];
end

set(handles.status, 'String', 'Sensors zeroed. Zero values shown in subplots.');
g_multigyrosen.ZeroMean = ZeroMean;
g_multigyrosen.ZeroStd = ZeroStd;

set([handles.zerotext_gyr1x handles.zerotext_gyr1y handles.zerotext_gyr1z handles.zerotext_gyr2x handles.zerotext_gyr2y handles.zerotext_gyr2z handles.zerotext_gyr3x handles.zerotext_gyr3y handles.zerotext_gyr3z handles.zerotext_gon1a handles.zerotext_gon1b handles.zerotext_gon2a handles.zerotext_gon2b], 'Visible', 'on');

set(handles.zerotext_gyr1x, 'String', sprintf('mu=%9.3f\nsd=%9.3f', ZeroMean(1), ZeroStd(1)));
set(handles.zerotext_gyr1y, 'String', sprintf('mu=%9.3f\nsd=%9.3f', ZeroMean(2), ZeroStd(2)));
set(handles.zerotext_gyr1z, 'String', sprintf('mu=%9.3f\nsd=%9.3f', ZeroMean(3), ZeroStd(3)));

set(handles.zerotext_gyr2x, 'String', sprintf('mu=%9.3f\nsd=%9.3f', ZeroMean(4), ZeroStd(4)));
set(handles.zerotext_gyr2y, 'String', sprintf('mu=%9.3f\nsd=%9.3f', ZeroMean(5), ZeroStd(5)));
set(handles.zerotext_gyr2z, 'String', sprintf('mu=%9.3f\nsd=%9.3f', ZeroMean(6), ZeroStd(6)));

set(handles.zerotext_gyr3x, 'String', sprintf('mu=%9.3f\nsd=%9.3f', ZeroMean(7), ZeroStd(7)));
set(handles.zerotext_gyr3y, 'String', sprintf('mu=%9.3f\nsd=%9.3f', ZeroMean(8), ZeroStd(8)));
set(handles.zerotext_gyr3z, 'String', sprintf('mu=%9.3f\nsd=%9.3f', ZeroMean(9), ZeroStd(9)));

set(handles.zerotext_gon1a, 'String', sprintf('mu=%9.3f\nsd=%9.3f', ZeroMean(10), ZeroStd(10)));
set(handles.zerotext_gon1b, 'String', sprintf('mu=%9.3f\nsd=%9.3f', ZeroMean(11), ZeroStd(11)));

set(handles.zerotext_gon2a, 'String', sprintf('mu=%9.3f\nsd=%9.3f', ZeroMean(12), ZeroStd(12)));
set(handles.zerotext_gon2b, 'String', sprintf('mu=%9.3f\nsd=%9.3f', ZeroMean(13), ZeroStd(13)));

end










function Angle = get_angles (handles)
global g_multigyrosen
persistent N_sensor
if isempty(N_sensor)
    N_sensor = length(cellstr(get(handles.cal_sensor, 'String')));
end
Angle = nan(1,N_sensor);
if ~isfield(g_multigyrosen, 'ModelData')
    return
end
for s = 1:N_sensor
    if length(g_multigyrosen.ModelData) >= s
        if isfield(g_multigyrosen.ModelData{s}, 'id') && g_multigyrosen.ModelData{s}.id
            
            
            
            switch g_multigyrosen.ModelData{s}.name
                case 'Linear regression'
                    B = g_multigyrosen.ModelData{s}.B;
                    k = g_multigyrosen.k;
                    if s >= 1 && s <= 3
                        GyroData = g_multigyrosen.GyroPos(k,(s-1)*3+1:(s-1)*3+3);
                        Angle(s) = [1 GyroData] * B;
                    else
                        GonioData = g_multigyrosen.GonioPos(k,(s-4)*2+1:(s-4)*2+2);
                        Angle(s) = [1 GonioData] * B;
                    end
                case 'Quadratic regression'
                    B = g_multigyrosen.ModelData{s}.B;
                    k = g_multigyrosen.k;
                    if s >= 1 && s <= 3
                        GyroData = g_multigyrosen.GyroPos(k,(s-1)*3+1:(s-1)*3+3);
                        Angle(s) = [1 GyroData GyroData.^2] * B;
                    else
                        GonioData = g_multigyrosen.GonioPos(k,(s-4)*2+1:(s-4)*2+2);
                        Angle(s) = [1 GonioData GonioData.^2] * B;
                    end
                case 'Cubic regression'
                    B = g_multigyrosen.ModelData{s}.B;
                    k = g_multigyrosen.k;
                    if s >= 1 && s <= 3
                        GyroData = g_multigyrosen.GyroPos(k,(s-1)*3+1:(s-1)*3+3);
                        Angle(s) = [1 GyroData GyroData.^2 GyroData.^3] * B;
                    else
                        GonioData = g_multigyrosen.GonioPos(k,(s-4)*2+1:(s-4)*2+2);
                        Angle(s) = [1 GonioData GonioData.^2 GonioData.^3] * B;
                    end
            end
        end
        
    end
end

end

% --- Executes on button press in connect.
function connect_Callback(hObject, eventdata, handles)
% hObject    handle to connect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global g_multigyrosen
Connected = getappdata(handles.multigyrosen, 'connected');
if ~Connected
    % If not connected
    set(hObject, 'Enable', 'off');
    set(hObject, 'String', 'Connecting');
    set(handles.led_error, 'ForegroundColor', [0.1 0 0]);
    set(handles.led_warning, 'ForegroundColor', [0.1 0.1 0]);
    set(handles.status, 'String', sprintf('Auto-detecting Arduino #1'));
    set([handles.errled_sen1 handles.errled_sen2 handles.errled_sen3 handles.errled_sen4 handles.errled_sen5], 'Visible', 'off');
    drawnow
    if isfield(g_multigyrosen, 'COM_Port') && ~isempty(g_multigyrosen.COM_Port)
        COM_Port = g_multigyrosen.COM_Port;
        if isfield(g_multigyrosen, 'COM_Port_Kid') && ~isempty(g_multigyrosen.COM_Port_Kid)
            COM_Port_Kid = g_multigyrosen.COM_Port_Kid;
        else
            COM_Port_Kid = '';
        end
    else
        COM_Port = findarduino_ck ('MULTIGYROSEN', [], [], [], 1);
        set(handles.status, 'String', sprintf('Auto-detecting Arduino #2')); drawnow
        COM_Port_Kid = findarduino_ck ('MULTIGYROSEN_KID', [], [], [], 1);
        g_multigyrosen.COM_Port = COM_Port;
        g_multigyrosen.COM_Port_Kid = COM_Port_Kid;
    end
    if ~isempty(COM_Port)
        set(handles.status, 'String', sprintf('Connecting to Arduino on %s', COM_Port));
        try
            cereal = serial(COM_Port);
            cereal.BaudRate = 115200;
            cereal.InputBufferSize = 1048576;
            fopen(cereal);
            pause(1.0);
            found = 0;
            found_kid = 0;
            g_multigyrosen.KidAttached = 0;
            timeout = 5;
            tfind = tic;
            while ~found && toc(tfind) < timeout
                fwrite(cereal, '?');
                while cereal.BytesAvailable
                    LINE = fgetl(cereal);
                    set(handles.line, 'String', LINE);
                    if ~isempty(regexp(LINE, 'MULTIGYROSEN', 'match', 'once'))
                        found = 1;
                        break
                    end
                end
                pause(0.1);
            end
        catch
            found = 0;
        end
        
        if ~isempty(COM_Port_Kid)
            set(handles.status, 'String', sprintf('Connecting to Arduino on %s', COM_Port_Kid));
            try
                cerealkid = serial(COM_Port_Kid);
                cerealkid.BaudRate = 115200;
                cerealkid.InputBufferSize = 1048576;
                fopen(cerealkid);
                pause(1.0);
                found_kid = 0;
                timeout = 5;
                tfind = tic;
                while ~found_kid && toc(tfind) < timeout
                    fwrite(cerealkid, '?');
                    while cerealkid.BytesAvailable
                        LINE_Kid = fgetl(cerealkid);
                        set(handles.line2, 'String', LINE_Kid);
                        if ~isempty(regexp(LINE_Kid, 'MULTIGYROSEN_KID', 'match', 'once'))
                            found_kid = 1;
                            break
                        end
                    end
                    pause(0.1);
                end
            catch
                found_kid = 0;
            end
        else
            COM_Port_Kid = 'any COM ports';
            found_kid = 0;
        end
        
    else
        COM_Port = 'any COM ports';
        found = 0;
    end
    drawnow
    
    if found
        setappdata(handles.multigyrosen, 'connected', 1);
        setappdata(handles.multigyrosen, 'cereal', cereal);
        if found_kid
            setappdata(handles.multigyrosen, 'cerealkid', cerealkid);
        end
        
        sensorok = [0 0 0 0 0];
        fwrite(cereal, 'q');
        retry = 0;
        while 1
            if cereal.BytesAvailable
                LINE = fgetl(cereal);
                b = regexp(LINE, '^Sensor Status: G1=(\d) G2=(\d) G3=(\d) G4=(\d) G5=(\d)', 'tokens', 'once');
                if ~isempty(b)
                    sensorok = str2double(b);
                    break
                end
            end
            retry = retry + 1;
            if mod(retry,10)
                fwrite(cereal, 'q');
            end
            pause(0.05);
        end
        
        if found_kid
            g_multigyrosen.KidAttached = 1;
            fwrite(cerealkid, 'q');
            retry = 0;
            while 1
                if cerealkid.BytesAvailable
                    LINE_Kid = fgetl(cerealkid);
                    b = regexp(LINE_Kid, '^Sensor Status: G1=(\d)', 'tokens', 'once');
                    if ~isempty(b)
                        sensorok(3) = str2double(b);
                        break
                    end
                end
                retry = retry + 1;
                if mod(retry,10)
                    fwrite(cerealkid, 'q');
                end
                pause(0.05);
            end
        end
        
        try g_multigyrosen = rmfield(g_multigyrosen, 'k'); end
        try g_multigyrosen = rmfield(g_multigyrosen, 'GonioPos'); end
        try g_multigyrosen = rmfield(g_multigyrosen, 'GyroPos'); end
        try g_multigyrosen = rmfield(g_multigyrosen, 'Time'); end
        
        
        
        if ~isfield(g_multigyrosen, 'ZeroMean')
            zero_sensors(handles);
        end
        
        
        if min(sensorok) == 0
            if sensorok(1), set(handles.errled_sen1, 'Visible', 'off'); else set(handles.errled_sen1, 'Visible', 'on'); end
            if sensorok(2), set(handles.errled_sen2, 'Visible', 'off'); else set(handles.errled_sen2, 'Visible', 'on'); end
            if sensorok(3), set(handles.errled_sen3, 'Visible', 'off'); else set(handles.errled_sen3, 'Visible', 'on'); end
            if sensorok(4), set(handles.errled_sen4, 'Visible', 'off'); else set(handles.errled_sen4, 'Visible', 'on'); end
            if sensorok(5), set(handles.errled_sen5, 'Visible', 'off'); else set(handles.errled_sen5, 'Visible', 'on'); end
            set(handles.status, 'String', 'Connected. Some sensors are not functional. Click Start to begin recording.');
        else
            set(handles.status, 'String', 'Connected. Click Start to begin recording.');
        end
        set(hObject, 'Enable', 'on');
        set(hObject, 'String', 'Disconnect');
        set(handles.start, 'Enable', 'on');
        set(handles.rezero, 'Enable', 'on');
        set(handles.sessionnum, 'Enable', 'on');
        set(handles.subjectid, 'Enable', 'on');
        set(handles.line, 'String', '');
        set(handles.line2, 'String', '');
        set(handles.elap, 'String', '');
        setappdata(handles.multigyrosen, 'do_mark_event', 0);
        set(handles.event, 'String', 'Event #1');
        set(handles.event, 'Enable', 'off');
        drawnow
    else
        try
            fclose(cereal);
        end
        try
            fclose(cerealkid);
        end
        setappdata(handles.multigyrosen, 'connected', 0);
        set(handles.status, 'String', sprintf('Arduino not found in %s', COM_Port));
        set(handles.led_error, 'ForegroundColor', [1 0 0]);
        set(hObject, 'Enable', 'on');
        set(hObject, 'String', 'Connect');
        set(handles.start, 'Enable', 'off');
        set(handles.rezero, 'Enable', 'off');
        set(handles.line, 'String', '');
        set(handles.line2, 'String', '');
        set(handles.elap, 'String', '');
        setappdata(handles.multigyrosen, 'do_mark_event', 0);
        set(handles.event, 'String', 'Event #1');
        set(handles.event, 'Enable', 'off');
        g_multigyrosen.COM_Port = '';
        try g_multigyrosen = rmfield(g_multigyrosen, 'k'); end
        try g_multigyrosen = rmfield(g_multigyrosen, 'GonioPos'); end
        try g_multigyrosen = rmfield(g_multigyrosen, 'GyroPos'); end
        try g_multigyrosen = rmfield(g_multigyrosen, 'Time'); end
        drawnow
        return
    end
else
    % If already connected
    try
        cereal = getappdata(handles.multigyrosen, 'cereal');
        fclose(cereal);
    end
    try
        cerealkid = getappdata(handles.multigyrosen, 'cerealkid');
        fclose(cerealkid);
    end
    setappdata(handles.multigyrosen, 'connected', 0);
    set(handles.status, 'String', 'Disconnected');
    set(handles.led_error, 'ForegroundColor', [0.1 0 0]);
    set(handles.led_warning, 'ForegroundColor', [0.1 0.1 0]);
    set(hObject, 'Enable', 'on');
    set(handles.start, 'Enable', 'off');
    set(handles.rezero, 'Enable', 'off');
    set(handles.sessionnum, 'Enable', 'on');
    set(handles.subjectid, 'Enable', 'on');
    set(hObject, 'String', 'Connect');
    set(handles.line, 'String', '');
    set(handles.line2, 'String', '');
    set(handles.elap, 'String', '');
    setappdata(handles.multigyrosen, 'do_mark_event', 0);
    set(handles.event, 'String', 'Event #1');
    set(handles.event, 'Enable', 'off');
    try g_multigyrosen = rmfield(g_multigyrosen, 'k'); end
    try g_multigyrosen = rmfield(g_multigyrosen, 'GonioPos'); end
    try g_multigyrosen = rmfield(g_multigyrosen, 'GyroPos'); end
    try g_multigyrosen = rmfield(g_multigyrosen, 'Time'); end
    drawnow
end

end

% --- Executes when user attempts to close multigyrosen.
function multigyrosen_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to multigyrosen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure

persistent clicked
if isempty(clicked)
    clicked = [];
end
clicked = [now clicked];
if length(clicked) >= 4 && abs(sum(diff(clicked(1:3)))*86400) < 2
    delete(hObject);
end

if getappdata(handles.multigyrosen, 'recording')
    set(handles.hint_forceclose, 'Visible', 'on');
    drawnow
    return
end
if getappdata(handles.multigyrosen, 'connected')
    connect_Callback(handles.connect, [], handles);
end
delete(hObject);

end

% --- Executes on button press in event.
function event_Callback(hObject, eventdata, handles)
% hObject    handle to event (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
setappdata(handles.multigyrosen, 'do_mark_event', 1);

end

% --- Executes on button press in sw_allplots.
function sw_allplots_Callback(hObject, eventdata, handles)
% hObject    handle to sw_allplots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
gyr1x = get(handles.sw_gyr1x, 'Value');
gyr1y = get(handles.sw_gyr1y, 'Value');
gyr1z = get(handles.sw_gyr1z, 'Value');
gyr2x = get(handles.sw_gyr2x, 'Value');
gyr2y = get(handles.sw_gyr2y, 'Value');
gyr2z = get(handles.sw_gyr2z, 'Value');
gyr3x = get(handles.sw_gyr3x, 'Value');
gyr3y = get(handles.sw_gyr3y, 'Value');
gyr3z = get(handles.sw_gyr3z, 'Value');
gon1a = get(handles.sw_gon1a, 'Value');
gon1b = get(handles.sw_gon1b, 'Value');
gon2a = get(handles.sw_gon2a, 'Value');
gon2b = get(handles.sw_gon2b, 'Value');
gyr1m = get(handles.sw_gyr1m, 'Value');
gyr2m = get(handles.sw_gyr2m, 'Value');
gyr3m = get(handles.sw_gyr3m, 'Value');
gon1m = get(handles.sw_gon1m, 'Value');
gon2m = get(handles.sw_gon2m, 'Value');

if gyr1x || gyr1y || gyr1z || gyr2x || gyr2y || gyr2z || gyr3x || gyr3y || gyr3z || gon1a || gon1b || gon2a || gon2b || gyr1m || gyr2m || gyr3m || gon1m || gon2m
    set(handles.sw_gyr1x, 'Value', 0);
    set(handles.sw_gyr1y, 'Value', 0);
    set(handles.sw_gyr1z, 'Value', 0);
    set(handles.sw_gyr2x, 'Value', 0);
    set(handles.sw_gyr2y, 'Value', 0);
    set(handles.sw_gyr2z, 'Value', 0);
    set(handles.sw_gyr3x, 'Value', 0);
    set(handles.sw_gyr3y, 'Value', 0);
    set(handles.sw_gyr3z, 'Value', 0);
    set(handles.sw_gon1a, 'Value', 0);
    set(handles.sw_gon1b, 'Value', 0);
    set(handles.sw_gon2a, 'Value', 0);
    set(handles.sw_gon2b, 'Value', 0);
    set(handles.sw_gyr1m, 'Value', 0);
    set(handles.sw_gyr2m, 'Value', 0);
    set(handles.sw_gyr3m, 'Value', 0);
    set(handles.sw_gon1m, 'Value', 0);
    set(handles.sw_gon2m, 'Value', 0);
else
    set(handles.sw_gyr1x, 'Value', 1);
    set(handles.sw_gyr1y, 'Value', 1);
    set(handles.sw_gyr1z, 'Value', 1);
    set(handles.sw_gyr2x, 'Value', 1);
    set(handles.sw_gyr2y, 'Value', 1);
    set(handles.sw_gyr2z, 'Value', 1);
    set(handles.sw_gyr3x, 'Value', 1);
    set(handles.sw_gyr3y, 'Value', 1);
    set(handles.sw_gyr3z, 'Value', 1);
    set(handles.sw_gon1a, 'Value', 1);
    set(handles.sw_gon1b, 'Value', 1);
    set(handles.sw_gon2a, 'Value', 1);
    set(handles.sw_gon2b, 'Value', 1);
    set(handles.sw_gyr1m, 'Value', 1);
    set(handles.sw_gyr2m, 'Value', 1);
    set(handles.sw_gyr3m, 'Value', 1);
    set(handles.sw_gon1m, 'Value', 1);
    set(handles.sw_gon2m, 'Value', 1);
end

end

% --- Executes on button press in sw_gyr1x.
function sw_gyr1x_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gyr1x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gyr1x

end

% --- Executes on button press in sw_gyr2x.
function sw_gyr2x_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gyr2x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gyr2x

end

% --- Executes on button press in sw_gyr3x.
function sw_gyr3x_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gyr3x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gyr3x

end

% --- Executes on button press in sw_gon1a.
function sw_gon1a_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gon1a (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gon1a

end

% --- Executes on button press in sw_gon2a.
function sw_gon2a_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gon2a (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gon2a

end

% --- Executes on button press in sw_gyr1y.
function sw_gyr1y_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gyr1y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gyr1y

end

% --- Executes on button press in sw_gyr1z.
function sw_gyr1z_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gyr1z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gyr1z

end

% --- Executes on button press in sw_gyr2y.
function sw_gyr2y_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gyr2y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gyr2y

end

% --- Executes on button press in sw_gyr2z.
function sw_gyr2z_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gyr2z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gyr2z

end

% --- Executes on button press in sw_gyr3y.
function sw_gyr3y_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gyr3y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gyr3y

end

% --- Executes on button press in sw_gyr3z.
function sw_gyr3z_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gyr3z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gyr3z

end

% --- Executes on button press in sw_gon1b.
function sw_gon1b_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gon1b (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gon1b

end

% --- Executes on button press in sw_gon2b.
function sw_gon2b_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gon2b (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gon2b

end

% --- Executes on button press in cal_sub10.
function cal_sub10_Callback(hObject, eventdata, handles)
% hObject    handle to cal_sub10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
val = str2double(get(handles.cal_angle, 'String'));
if ~isnan(val)
    val = val - 10;
else
    val = 0;
end
set(handles.cal_angle, 'String', num2str(val));

end

% --- Executes on button press in cal_add10.
function cal_add10_Callback(hObject, eventdata, handles)
% hObject    handle to cal_add10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
val = str2double(get(handles.cal_angle, 'String'));
if ~isnan(val)
    val = val + 10;
else
    val = 0;
end
set(handles.cal_angle, 'String', num2str(val));

end


function cal_angle_Callback(hObject, eventdata, handles)
% hObject    handle to cal_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cal_angle as text
%        str2double(get(hObject,'String')) returns contents of cal_angle as a double
val = str2double(get(hObject, 'String'));
if isnan(val)
    val = 0;
end
set(handles.cal_angle, 'String', num2str(val));

end

% --- Executes during object creation, after setting all properties.
function cal_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cal_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

% --- Executes on button press in cal_mark.
function cal_mark_Callback(hObject, eventdata, handles)
% hObject    handle to cal_mark (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global g_multigyrosen
angle = str2double(get(handles.cal_angle, 'String'));
if ~isnan(angle) && isfield(g_multigyrosen, 'k') && g_multigyrosen.k > 4
    CalibrationData = getappdata(handles.multigyrosen, 'CalibrationData');
    sensor_ind = get(handles.cal_sensor, 'Value');
    if length(CalibrationData) >= sensor_ind
        SensorCal = CalibrationData{sensor_ind};
    end
    tmp = cellstr(get(handles.cal_sensor,'String'));
    SensorCal.name = tmp{sensor_ind};
    k = g_multigyrosen.k;
    switch SensorCal.name
        case 'Gyro 1'
            sensordata = mean(g_multigyrosen.GyroPos(k-4:k,1:3));
        case 'Gyro 2'
            sensordata = mean(g_multigyrosen.GyroPos(k-4:k,4:6));
        case 'Gyro 3'
            sensordata = mean(g_multigyrosen.GyroPos(k-4:k,7:9));
        case 'Gonio 1'
            sensordata = mean(g_multigyrosen.GonioPos(k-4:k,1:2));
        case 'Gonio 2'
            sensordata = mean(g_multigyrosen.GonioPos(k-4:k,3:4));
    end
    
    if ~isfield(SensorCal, 'data')
        SensorCal.data = [];
    end
    
    if isempty(SensorCal.data) || ~ismember(k,SensorCal.data(:,2))
        SensorCal.data = sortrows([
            SensorCal.data
            angle k sensordata
            ]);
    end
    CalibrationData{sensor_ind} = SensorCal;
    setappdata(handles.multigyrosen, 'CalibrationData', CalibrationData);
    populate_pointlist(handles, SensorCal, k);
    
end

end

function populate_pointlist(handles, SensorCal, k)
global g_multigyrosen
if ~exist('k','var')
    k = [];
end
if isempty(SensorCal) || ~isfield(SensorCal, 'data') || isempty(SensorCal.data)
    set(handles.cal_pointlist, 'String', 'No calibration points');
    set(handles.cal_pointlist, 'Value', 1);
    set(handles.cal_n, 'String', num2str(0));
    set(handles.cal_model, 'Value', 1);
else
    data = SensorCal.data;
    if size(data,2) == 4
        data(:,5) = nan(size(data,1),1);
    end
    listdata = string_to_cell(sprintf('%+4.0f°    (%.2f, %.2f, %.2f) (k=%i)\n', data(:,[1 3 4 5 2]).'),sprintf('\n')).';
    set(handles.cal_pointlist, 'String', listdata);
    if ~isempty(k)
        [~, ind] = min(abs(data(:,2) - k));
        set(handles.cal_pointlist, 'Value', ind);
    else
        set(handles.cal_pointlist, 'Value', 1);
    end
    set(handles.cal_n, 'String', num2str(size(listdata,1)));
    set(handles.cal_model, 'Value', 1);
    if isfield(g_multigyrosen, 'ModelData')
        sensor_id = get(handles.cal_sensor, 'Value');
        if length(g_multigyrosen.ModelData) >= sensor_id
            if isfield(g_multigyrosen.ModelData{sensor_id}, 'id') && g_multigyrosen.ModelData{sensor_id}.id
                set(handles.cal_model, 'Value', g_multigyrosen.ModelData{sensor_id}.id);
            end
        end
    end
    
end
cal_model_Callback(handles.cal_model, [], handles);

end


function update_rsq (handles)
global g_multigyrosen
sensor_id = get(handles.cal_sensor, 'Value');
if isfield(g_multigyrosen,'ModelData') && length(g_multigyrosen.ModelData) >= sensor_id && isfield(g_multigyrosen.ModelData{sensor_id},'ACOD') && isfield(g_multigyrosen.ModelData{sensor_id},'id') && g_multigyrosen.ModelData{sensor_id}.id ~= 0
    Rsq = g_multigyrosen.ModelData{sensor_id}.ACOD;
    set(handles.cal_rsq, 'String', num2str(Rsq));
    
    if isfield(g_multigyrosen.ModelData{sensor_id},'B') && ~isempty(g_multigyrosen.ModelData{sensor_id}.B)
        B = g_multigyrosen.ModelData{sensor_id}.B;
        str = sprintf('Parameter:\n');
        for i = 1:length(B)
            str = [str sprintf('b%i= %+.3g', i-1, B(i))];
            if i ~= length(B)
                str = [str sprintf('\n')];
            end
        end
        set(handles.cal_modelparms, 'String', str, 'FontSize', 9);
    else
        B = [];
        set(handles.cal_modelparms, 'String', 'Unknown.');
    end
    
    
    
else
    set(handles.cal_rsq, 'String', 'NaN');
    set(handles.cal_modelparms, 'String', 'Enter more points for this model.');
end

end

% --- Executes on selection change in cal_pointlist.
function cal_pointlist_Callback(hObject, eventdata, handles)
% hObject    handle to cal_pointlist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns cal_pointlist contents as cell array
%        contents{get(hObject,'Value')} returns selected item from cal_pointlist

end

% --- Executes during object creation, after setting all properties.
function cal_pointlist_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cal_pointlist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

% --- Executes on button press in cal_del.
function cal_del_Callback(hObject, eventdata, handles)
% hObject    handle to cal_del (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
CalibrationData = getappdata(handles.multigyrosen, 'CalibrationData');
sensor_ind = get(handles.cal_sensor, 'Value');
list_ind = get(handles.cal_pointlist, 'Value');
if length(CalibrationData) >= sensor_ind
    SensorCal = CalibrationData{sensor_ind};
    if size(SensorCal.data,1) >= list_ind
        k = SensorCal.data(list_ind,2);
        SensorCal.data = SensorCal.data([1:list_ind-1,list_ind+1:end],:);
        CalibrationData{sensor_ind} = SensorCal;
        setappdata(handles.multigyrosen, 'CalibrationData', CalibrationData);
        populate_pointlist(handles, SensorCal, k);
    end
end

end

% --- Executes on button press in cal_clear.
function cal_clear_Callback(hObject, eventdata, handles)
% hObject    handle to cal_clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
CalibrationData = getappdata(handles.multigyrosen, 'CalibrationData');
sensor_ind = get(handles.cal_sensor, 'Value');
CalibrationData{sensor_ind} = [];
setappdata(handles.multigyrosen, 'CalibrationData', CalibrationData);
populate_pointlist(handles, []);

end

% --- Executes on selection change in cal_model.
function cal_model_Callback(hObject, eventdata, handles)
% hObject    handle to cal_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns cal_model contents as cell array
%        contents{get(hObject,'Value')} returns selected item from cal_model

global g_multigyrosen
contents = cellstr(get(hObject,'String'));
model_name = contents{get(hObject,'Value')};
model_id = get(hObject,'Value');

CalibrationData = getappdata(handles.multigyrosen, 'CalibrationData');

s = get(handles.cal_sensor, 'Value');
g_multigyrosen.ModelData{s}.id = 0;
g_multigyrosen.ModelData{s}.name = 'none';
set(handles.cal_formula, 'String', '');

switch model_name
    case 'Linear regression'
        % theta = b0 + b1 phi_x + b2 phi_y + b3 phi_z
        set(handles.cal_formula, 'String', 'q = b0 + b1*x + b2*y + b3*z');
        if length(CalibrationData) >= s
            SensorCal = CalibrationData{s};
            if isfield(SensorCal, 'data') && size(SensorCal.data,1) > 2
                if size(SensorCal.data,2) == 5
                    X = SensorCal.data(:,3:5);
                    Y = SensorCal.data(:,1);
                    [B, ~, ~, ~, ~, COD, ACOD] = mvlsq(X, Y, 1, 1);
                    g_multigyrosen.ModelData{s}.id = model_id;
                    g_multigyrosen.ModelData{s}.name = model_name;
                    g_multigyrosen.ModelData{s}.B = B;
                    g_multigyrosen.ModelData{s}.COD = COD;
                    g_multigyrosen.ModelData{s}.ACOD = ACOD;
                    %fprintf('setting sensor %i to model %i\n', s, model_id);
                elseif size(SensorCal.data,2) == 4
                    X = SensorCal.data(:,3:4);
                    Y = SensorCal.data(:,1);
                    [B, ~, ~, ~, ~, COD, ACOD] = mvlsq(X, Y, 1, 1);
                    g_multigyrosen.ModelData{s}.id = model_id;
                    g_multigyrosen.ModelData{s}.name = model_name;
                    g_multigyrosen.ModelData{s}.B = B;
                    g_multigyrosen.ModelData{s}.COD = COD;
                    g_multigyrosen.ModelData{s}.ACOD = ACOD;
                    %fprintf('setting sensor %i to model %i\n', s, model_id);
                end
            end
        end
        
    case 'Quadratic regression'
        % theta = b0 + b1 phi_x + b2 phi_y + b3 phi_z + b4 phi_x^2 + b5 phi_y^2 + b6 phi_z^2
        set(handles.cal_formula, 'String', 'q = b0 + b1*x + b2*y + b3*z + b4*x² + b5*y² + b6*z²');
        if length(CalibrationData) >= s
            SensorCal = CalibrationData{s};
            if isfield(SensorCal, 'data') && size(SensorCal.data,1) > 3
                if size(SensorCal.data,2) == 5
                    X = [SensorCal.data(:,3:5) SensorCal.data(:,3:5).^2];
                    Y = SensorCal.data(:,1);
                    [B, ~, ~, ~, ~, COD, ACOD] = mvlsq(X, Y, 1, 1);
                    g_multigyrosen.ModelData{s}.id = model_id;
                    g_multigyrosen.ModelData{s}.name = model_name;
                    g_multigyrosen.ModelData{s}.B = B;
                    g_multigyrosen.ModelData{s}.COD = COD;
                    g_multigyrosen.ModelData{s}.ACOD = ACOD;
                    fprintf('setting sensor %i to model %i\n', s, model_id);
                elseif size(SensorCal.data,2) == 4
                    X = [SensorCal.data(:,3:4) SensorCal.data(:,3:4).^2];
                    Y = SensorCal.data(:,1);
                    [B, ~, ~, ~, ~, COD, ACOD] = mvlsq(X, Y, 1, 1);
                    g_multigyrosen.ModelData{s}.id = model_id;
                    g_multigyrosen.ModelData{s}.name = model_name;
                    g_multigyrosen.ModelData{s}.B = B;
                    g_multigyrosen.ModelData{s}.COD = COD;
                    g_multigyrosen.ModelData{s}.ACOD = ACOD;
                    fprintf('setting sensor %i to model %i\n', s, model_id);
                end
            end
        end
    case 'Cubic regression'
        % theta = b0 + b1 phi_x + b2 phi_y + b3 phi_z + b4 phi_x^2 + b5 phi_y^2 + b6 phi_z^2 + b7 phi_x^3 + b8 phi_y^3 + b9 phi_z^3
        set(handles.cal_formula, 'String', 'q = b0 + b1*x + b2*y + b3*z + b4*x² + b5*y² + b6*z² + b7*x³ + b8*y³ + b9*z³');
        if length(CalibrationData) >= s
            SensorCal = CalibrationData{s};
            if isfield(SensorCal, 'data') && size(SensorCal.data,1) > 4
                if size(SensorCal.data,2) == 5
                    X = [SensorCal.data(:,3:5) SensorCal.data(:,3:5).^2 SensorCal.data(:,3:5).^3];
                    Y = SensorCal.data(:,1);
                    [B, ~, ~, ~, ~, COD, ACOD] = mvlsq(X, Y, 1, 1);
                    g_multigyrosen.ModelData{s}.id = model_id;
                    g_multigyrosen.ModelData{s}.name = model_name;
                    g_multigyrosen.ModelData{s}.B = B;
                    g_multigyrosen.ModelData{s}.COD = COD;
                    g_multigyrosen.ModelData{s}.ACOD = ACOD;
                    fprintf('setting sensor %i to model %i\n', s, model_id);
                elseif size(SensorCal.data,2) == 4
                    X = [SensorCal.data(:,3:4) SensorCal.data(:,3:4).^2 SensorCal.data(:,3:4).^3];
                    Y = SensorCal.data(:,1);
                    [B, ~, ~, ~, ~, COD, ACOD] = mvlsq(X, Y, 1, 1);
                    g_multigyrosen.ModelData{s}.id = model_id;
                    g_multigyrosen.ModelData{s}.name = model_name;
                    g_multigyrosen.ModelData{s}.B = B;
                    g_multigyrosen.ModelData{s}.COD = COD;
                    g_multigyrosen.ModelData{s}.ACOD = ACOD;
                    fprintf('setting sensor %i to model %i\n', s, model_id);
                end
            end
        end
end

update_rsq(handles);

N_sensor = length(cellstr(get(handles.cal_sensor, 'String')));
for s = 1:N_sensor
    if isfield(g_multigyrosen, 'ModelData') && length(g_multigyrosen.ModelData) >= s && isfield(g_multigyrosen.ModelData{s},'id') && g_multigyrosen.ModelData{s}.id
        set(handles.(['led_sen' num2str(s)]), 'BackgroundColor', [0 0.5 0]);
    else
        set(handles.(['led_sen' num2str(s)]), 'BackgroundColor', [1 0 0]);
    end
end

end

% --- Executes during object creation, after setting all properties.
function cal_model_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cal_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

% --- Executes on selection change in cal_sensor.
function cal_sensor_Callback(hObject, eventdata, handles)
% hObject    handle to cal_sensor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns cal_sensor contents as cell array
%        contents{get(hObject,'Value')} returns selected item from cal_sensor
CalibrationData = getappdata(handles.multigyrosen, 'CalibrationData');
sensor_ind = get(hObject, 'Value');
if length(CalibrationData) >= sensor_ind
    SensorCal = CalibrationData{sensor_ind};
else
    SensorCal = [];
end
populate_pointlist(handles, SensorCal);

end

% --- Executes during object creation, after setting all properties.
function cal_sensor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cal_sensor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

% --- Executes on button press in cal_clearall.
function cal_clearall_Callback(hObject, eventdata, handles)
% hObject    handle to cal_clearall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
setappdata(handles.multigyrosen, 'CalibrationData', {});
populate_pointlist(handles, []);

end

% --- Executes on button press in sw_gyr1m.
function sw_gyr1m_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gyr1m (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gyr1m

end

% --- Executes on button press in sw_gyr2m.
function sw_gyr2m_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gyr2m (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gyr2m

end

% --- Executes on button press in sw_gyr3m.
function sw_gyr3m_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gyr3m (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gyr3m

end

% --- Executes on button press in sw_gon1m.
function sw_gon1m_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gon1m (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gon1m

end

% --- Executes on button press in sw_gon2m.
function sw_gon2m_Callback(hObject, eventdata, handles)
% hObject    handle to sw_gon2m (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sw_gon2m

end

% --- Executes on button press in led_sen1.
function led_sen1_Callback(hObject, eventdata, handles)
% hObject    handle to led_sen1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.cal_sensor, 'Value', 1);
cal_sensor_Callback(handles.cal_sensor, [], handles);

end

% --- Executes on button press in led_sen2.
function led_sen2_Callback(hObject, eventdata, handles)
% hObject    handle to led_sen2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.cal_sensor, 'Value', 2);
cal_sensor_Callback(handles.cal_sensor, [], handles);

end

% --- Executes on button press in led_sen3.
function led_sen3_Callback(hObject, eventdata, handles)
% hObject    handle to led_sen3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.cal_sensor, 'Value', 3);
cal_sensor_Callback(handles.cal_sensor, [], handles);

end

% --- Executes on button press in led_sen4.
function led_sen4_Callback(hObject, eventdata, handles)
% hObject    handle to led_sen4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.cal_sensor, 'Value', 4);
cal_sensor_Callback(handles.cal_sensor, [], handles);

end

% --- Executes on button press in led_sen5.
function led_sen5_Callback(hObject, eventdata, handles)
% hObject    handle to led_sen5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.cal_sensor, 'Value', 5);
cal_sensor_Callback(handles.cal_sensor, [], handles);


end


% --- Executes on button press in rezero.
function rezero_Callback(hObject, eventdata, handles)
% hObject    handle to rezero (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Connected = getappdata(handles.multigyrosen, 'connected');
Recording = getappdata(handles.multigyrosen, 'recording');
if Connected && ~Recording
    set(hObject, 'Enable', 'off'); 
    drawnow
    zero_sensors(handles);
    set(hObject, 'Enable', 'on');
end

end


function subjectid_Callback(hObject, eventdata, handles)
% hObject    handle to subjectid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of subjectid as text
%        str2double(get(hObject,'String')) returns contents of subjectid as a double
global g_multigyrosen
Connected = getappdata(handles.multigyrosen, 'connected');
Recording = getappdata(handles.multigyrosen, 'recording');
sid = get(hObject,'String');
if Recording
    SubjectID = getsubjectid(sid);
    g_multigyrosen.SubjectID = SubjectID;
    set(hObject, 'String', SubjectID);
    check_valid_savename(handles);
    %SubjectID = g_multigyrosen.SubjectID;
    %set(hObject, 'String', SubjectID);
    return
end
DisableList = [handles.sessionnum    handles.subjectid    handles.rezero    handles.start    handles.connect];
try
    SubjectID = getsubjectid(sid);
    if ~isempty(regexp(SubjectID, '^SJ\d{4,}$', 'match'))
        set(hObject, 'String', SubjectID);
        CurrentlyEnabled = ~cellfun(@isempty,regexp(get(DisableList, 'Enable'), '^on$', 'once'));
        CurrentStatusText = get(handles.status, 'String');
        set(DisableList, 'Enable', 'off');
        set(handles.status, 'String', 'Verify Subject ID in the other window');
        drawnow
        verify_consent(SubjectID, 1, 1, 1);
        set(DisableList(CurrentlyEnabled), 'Enable', 'on');
        set(handles.status, 'String', CurrentStatusText);
        drawnow
    end
catch
    SubjectID = '';
end
if isempty(regexp(SubjectID, '^SJ\d{4,}$', 'match'))
    SubjectID = '';
end
g_multigyrosen.SubjectID = SubjectID;
set(hObject, 'String', SubjectID);
check_valid_savename(handles);

end

% --- Executes during object creation, after setting all properties.
function subjectid_CreateFcn(hObject, eventdata, handles)
% hObject    handle to subjectid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end


function sessionnum_Callback(hObject, eventdata, handles)
% hObject    handle to sessionnum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sessionnum as text
%        str2double(get(hObject,'String')) returns contents of sessionnum as a double
global g_multigyrosen
sn = floor(str2double(get(hObject,'String')));
if ~isnan(sn)
    SessionNum = sprintf('%02i', sn);
else
    SessionNum = '';
end
g_multigyrosen.SessionNum = SessionNum;
set(hObject, 'String', SessionNum);
check_valid_savename(handles);

end

% --- Executes during object creation, after setting all properties.
function sessionnum_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sessionnum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end


function check_valid_savename (handles)
global g_multigyrosen
SubjectID = g_multigyrosen.SubjectID;
SessionNum = g_multigyrosen.SessionNum;

set([handles.warn_willnotsave handles.warn_nosavewhy], 'Visible', 'off');
if isempty(SessionNum)
    set(handles.sessionnum, 'String', '');
    set([handles.warn_willnotsave handles.warn_nosavewhy], 'Visible', 'on');
    set(handles.warn_nosavewhy, 'String', 'Missing Session #');
end
if isempty(SubjectID)
    set(handles.subjectid, 'String', '');
    set([handles.warn_willnotsave handles.warn_nosavewhy], 'Visible', 'on');
    set(handles.warn_nosavewhy, 'String', 'Missing Subject ID');
end
if ~isempty(SubjectID) && ~isempty(SessionNum)
    matname = sprintf('%s-%sarduinomultigyrosen.mat', SubjectID, SessionNum);
    Filename2 = sprintf('%s%s%s', getdesktopdir(), filesep, matname);
    if exist(Filename2, 'file')
        %set(handles.sessionnum, 'String', '');
        set(handles.warn_nosavewhy, 'Visible', 'on');
        set(handles.warn_nosavewhy, 'String', 'Saved session exists.');
    end
end

end

