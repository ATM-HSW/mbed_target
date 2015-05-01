function ret = mbed_getDownloadMethod()
pathstr = mbed_getTargetRootPath();
targetsfiles = ls(fullfile(pathstr,'targets_flash','*.bat'));
ret=['|mbed'];
for i=1:size(targetsfiles,1)
    %ret(i)
    [~,name,~] = fileparts(targetsfiles(i,:));
    ret = [ret '|' name]; %#ok<AGROW>
end
ret = ret(2:end);
end
