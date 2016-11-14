function ret = mbed_getTargets()
pathstr = mbed_getTargetRootPath();
targetsfiles = ls(fullfile(pathstr,'targets','*.zip'));
ret=[];
for i=1:size(targetsfiles,1)
    %ret(i)
    [~,name,~] = fileparts(targetsfiles(i,:));
    ret = [ret '|' name]; %#ok<AGROW>
end
ret = ret(2:end);
end
