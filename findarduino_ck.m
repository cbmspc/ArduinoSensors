%% findarduino_ck - Function to find serial port used in ArduinoGUI
%See ArduinoGUI.m to see where findarduino_ck is used when pressing the
%start button.

%This function searches up to 19 serial ports on the computer for the ports
%that have a response/are plugged into something that is on.

%Created by Po T. Wang and edited by Christine E. King on 11/03/2011
function cerealstring = findarduino_ck (IDstring, OpenTimeout, UpToCOM, ReceiveTimeout, verbosity)

persistent LastSuccessfulFindPair

try %#ok<*TRYNC>
    % This force close all opened serial connections from MATLAB
    a = instrfind('Status', 'open'); %Try to close all serial ports
    for i = 1:length(a)
        fclose(a(i)); %Force close all open serial connections
        delete(a(i));
    end
end

if ~exist('IDstring','var') || isempty(IDstring)
    IDstring = 'ARDUINO_UNO_KINESIOLOGY';
    %IDstring = 'ARDUINO_MEGA_RASCALTHEROBOT';
end

if ~exist('OpenTimeout','var') || isempty(OpenTimeout)
    OpenTimeout = 2;
end

if ~exist('UpToCOM','var') || isempty(UpToCOM)
    UpToCOM = 19;
end

if length(UpToCOM) > 1
    COMtoSearch = UpToCOM(:).';
else
    COMtoSearch = 1:UpToCOM;
    COMtoSearch = COMtoSearch(randperm(length(COMtoSearch)));
end

if ~isempty(LastSuccessfulFindPair)
    [~, ia] = intersect(cell2mat(LastSuccessfulFindPair(:,2)), COMtoSearch);
    ja = ~cellfun(@isempty,regexp(LastSuccessfulFindPair(ia,1), ['^' IDstring '[' 13 10 ']{0,2}' '$'], 'match', 'once'));
    LastSuccessfulFind = LastSuccessfulFindPair(ia(ja),2);
    if ~isempty(LastSuccessfulFind) && isnumeric(LastSuccessfulFind{1})
        LastSuccessfulFind = cat(2,LastSuccessfulFind{:});
        COMtoSearch = [LastSuccessfulFind, setdiff(COMtoSearch, LastSuccessfulFind)];
    end
end

if ~exist('verbosity','var') || isempty(verbosity)
    verbosity = 0;
end

if ~exist('ReceiveTimeout','var') || isempty(ReceiveTimeout)
    ReceiveTimeout = 3;
end

found = 0;
cerealstring = ''; %For ArduinoGUI, make sure there's nothing if there's no serial port connected
for p = COMtoSearch %Searches up to UpToCOM serial ports
    OpenOK = 0;
    ReceiveOK = 0;
    
    if verbosity
        fprintf('\n');
        fprintf('Testing connection to COM%i .. ', p);
    end
    cereal = serial(['COM' num2str(p)], 'BaudRate', 115200,'InputBufferSize',1048576,'Timeout',OpenTimeout); %#ok<TNMLP>
    try %Try to open serial port
        fopen(cereal);
        OpenOK = 1;
        if verbosity
            fprintf('[opened] ');
        end
    catch %#ok<CTCH>
        if verbosity
            fprintf('[cannot open] ');
        end
    end
    
    if OpenOK
        pause(ReceiveTimeout);
        if cereal.BytesAvailable > 0
            ReceiveOK = 1;
            if verbosity
                fprintf('[receive ok] ');
            end
        else
            ReceiveOK = 0;
            if verbosity
                fprintf('[receive failed] ');
            end
        end
    end
    
    if ReceiveOK
        try
            flushinput(cereal);
        end
        if verbosity
            fprintf('[sending IDN?] ');
        end
        fprintf(cereal, '*IDN?');
        if verbosity
            fprintf('[sent IDN?] ');
        end
        pause(ReceiveTimeout);
        while cereal.BytesAvailable
            if verbosity
                fprintf('[trying to read] ');
            end
            idn = fscanf(cereal);
            if verbosity
                fprintf('\n');
                fprintf('COM%i response: %s', p, idn);
                fprintf('\n');
            end
            if ~isempty(regexp(idn, ['^' IDstring '[' 13 10 ']{0,2}' '$'], 'match', 'once'))
                cerealstring = ['COM' num2str(p)];
                LastSuccessfulFindPair = uniquerows([LastSuccessfulFindPair; {IDstring} {p}]);
                fprintf('Found an Arduino %s device on COM%i\n', IDstring, p);
                found = 1;
                break;
            end
        end
        if found
            break
        end
    end
    fclose(cereal); %Always close serial port so it can be opened again
    delete(cereal);
    if verbosity
        fprintf('Closed COM%i', p);
        fprintf('\n');
    end
end
try
    fclose(cereal); %Always close serial port so it can be opened again
    delete(cereal);
end

%{
        fprintf(cereal, '*IDN?');
        pause(1.000);
        if cereal.BytesAvailable %If we have a serial port open, display that there was a response from that port
            idn = fscanf(cereal); %Figure out number of serial port that's open
            fprintf('COM%i response: %s', p, idn);
            cerealstring = ['COM' num2str(p)];
            fclose(cereal); %Close serial port so it doesn't open while still doing stuff
            delete(cereal);
            break
        else
            fprintf('COM%i no response\n', p); %If no serial port response, display which ports on not responding
        end
    else
        %fprintf('COM%i not valid serial\n', p);
%}