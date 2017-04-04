function setmatlabtitle (title)
try
    jDesktop = com.mathworks.mde.desk.MLDesktop.getInstance;
    jDesktop.getMainFrame.setTitle(title);
catch
    warning('Failed to set MATLAB title');
end
