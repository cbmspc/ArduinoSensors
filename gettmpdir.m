function D = gettmpdir ()
if ispc
    D = getenv('tmp');
else
    D = '/tmp';
end
