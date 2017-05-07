function ret = mbed_getTargets(mbedversion)

if isequal(mbedversion, 'mbed-os 5')
    pathstr = mbed_getTargetRootPath();
    newpath = fullfile(pathstr,'targets','mbed-os','tools');
    oldpath=cd(newpath);
    [~,cmdout]=system('python project.py -S targets');
    cmdout=regexprep(cmdout,'\n\n','');
    ret=regexprep(cmdout,'\n','|');
    ret=ret(1:end-1);
    cd(oldpath);
else
    pathstr = mbed_getTargetRootPath();
    targetsfiles = ls(fullfile(pathstr,'targets','*.zip'));
    ret=[];
    for i=1:size(targetsfiles,1)
        [~,name,~] = fileparts(targetsfiles(i,:));
        ret = [ret '|' name]; %#ok<AGROW>
    end
    ret = ret(2:end);
end

end
