%% findarduino_ck_blue - Function to open Bluetooth Connection for Arduino
%See ArduinoLaserWiiGUI.m to see where findarduino_ck_blue is used when pressing the
%start button.

%This function opens the bluetooth connection given the known remote name.
%(see Bluetooth Devices in Control Panel for names of devices to connect to)

%Created by Christine E King on 2013-01-28

function [bluestring, blue] = findarduino_ck_blue2 (RemoteName, IDstring, ReceiveTimeout, verbosity)
fprintf('findarduino_ck_blue2:\n');
bluestring = '';
if ~exist('RemoteName', 'var') || isempty(RemoteName)
    RemoteName = 'RN42-0156';
end

name.RemoteName = [];
retry = 3;
while retry && isempty(name.RemoteName)
    fprintf('  Finding Bluetooth instrumentation named %s (%i attempts left)\n', RemoteName, retry);
    name = instrhwinfo('Bluetooth', RemoteName);
    retry = retry - 1;
end
if isempty(name.RemoteName)
    error('%s not found via Bluetooth connection. Verify that Bluetooth is present and paired.', RemoteName);
end

fprintf('  Creating Bluetooth object\n');
blue = Bluetooth(RemoteName, str2double(cell2mat(name.Channels)));

if ~exist('IDstring','var') || isempty(IDstring)
    IDstring = 'MEGA_LASER_WMPG';
end

if ~exist('verbosity','var') || isempty(verbosity)
    verbosity = 0;
end

if ~exist('ReceiveTimeout','var') || isempty(ReceiveTimeout)
    ReceiveTimeout = 1;
end

if length(blue.Channel) == 1
    OpenOK = 0;
    ReceiveOK = 0;
    
    if verbosity
        fprintf('\n');
        fprintf('Connecting to Bluetooth serial..\n');
    end
    try %Try to open serial port
        blue.InputBufferSize = 1048576;
        fopen(blue);
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
        if blue.Status == 'open' %#ok<STCMP>
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
        flushinput(blue);
        if verbosity
            fprintf('[sending IDN?] ');
        end
        fprintf(blue, '*IDN?');
        if verbosity
            fprintf('[sent IDN?] ');
        end
        pause(ReceiveTimeout);
        if verbosity
            fprintf('[trying to read] ');
        end
        idn = fscanf(blue);
        if verbosity
            fprintf('\n');
            fprintf('Bluetooth response: %s', idn);
            fprintf('\n');
        end
        if ~isempty(regexp(idn, IDstring, 'match', 'once'))
            fprintf('Found an Arduino device on Bluetooth\n');
            bluestring = ['Bluetooth ID: ' num2str(blue.RemoteID)];
        end
    end
end

if ReceiveOK == 0
    try
        fclose(blue); %Always close serial port so it can be opened again
        delete(blue);
    catch %#ok<CTCH>
        fprintf('Unable to close Bluetooth connection..\n');
    end
end
