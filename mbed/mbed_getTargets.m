function ret = mbed_getTargets_old()
pathstr = mbed_getTargetRootPath();
targetsfiles = ls(fullfile(pathstr,'targets','*.zip'));
ret=[];
for i=1:size(targetsfiles,1)
    %ret(i)
    [~,name,~] = fileparts(targetsfiles(i,:));
    ret = [ret '|' name]; %#ok<AGROW>
end
ret = ret(2:end);

function ret = mbed_getTargets(mbedversion)

  if isequal(mbedversion, 'mbed-os 5')
    [a,b]=system('mbed export -T 2>:null');
    b=regexprep(b,'\n\n','');
    ret=regexprep(b,'\n','|');
    ret=ret(1:end-1);
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
  disp(ret);
>>>>>>> integration of pip installation of mbed-cli and mbed-ls
end
