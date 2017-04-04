function DesktopDir = getdesktopdir ()
% Gets Desktop's path
if ispc
    DesktopDir = [getenv('USERPROFILE') '\Desktop'];
else
    DesktopDir = [getenv('HOME') '/Desktop'];
end
