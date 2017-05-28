function [ret1, ret2, count]= mbed_getTargets(mbedversion)

if isequal(mbedversion, 'mbed-os 5')
    pathstr = mbed_getTargetRootPath();
    newpath = fullfile(pathstr,'targets','mbed-os','tools');
    oldpath=cd(newpath);
    [~,cmdout]=system('python project.py -S targets');
    ret2=regexprep(cmdout,'\n\n','');
    ret1=regexprep(ret2,'\n','|');
    ret1=ret1(1:end-1);
    count=size(strfind(ret1,'|'),2)+1;
    cd(oldpath);
else
    pathstr = mbed_getTargetRootPath();
    targetsfiles = ls(fullfile(pathstr,'targets','*.zip'));
    ret=[];
    for i=1:size(targetsfiles,1)
        [~,name,~] = fileparts(targetsfiles(i,:));
        ret = [ret '|' name]; %#ok<AGROW>
    end
    ret1 = ret(2:end);
    ret2='';
    count=0;
end

end
