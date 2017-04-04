% Dummy code for public use
function X = getsubjectid (X)

if ~exist('X','var')
    X = inputdlg('Enter Subject ID: ','getsubjectid',1);
    if isempty(X)
        X = '';
    end
    X = X{1};
end

if isempty(X)
    X = '';
    return
end

X = sanitizename(X);


function outputname = sanitizename (inputname)
% Change to all lower case
% Remove duplicate spacebars
% Replace spacebar with +
outputname = lower(inputname);
% Remove illegal chars, space at beginning or end
outputname = regexprep(outputname, '^\s+|\s+$|[\x00-\x1F]', '');

% Replace space with + sign (also de-duplicates them)
outputname = regexprep(outputname, ' {1,}', '+');
