function ret = mbed_getTargets()
disp 'mbed_getTargets called';
[pathstr,name,ext] = fileparts(getMbedTargetPath());
%ret = pathstr;
targetsfiles = ls(fullfile(pathstr,'targets','*.zip'));
ret=[];
for i=1:size(targetsfiles,1)
    %ret(i)
    [pathstr,name,ext] = fileparts(targetsfiles(i,:));
    ret = [ret '|' name];
end
ret = ret(2:end);
end

function ret = getMbedTargetPath()
  ret = '';
  a=strsplit(path,';');
  b=regexp(a,'\\mbed_\w*\\mbed');
  %b=regexp(a,'\\mbed_\w*\\blocks');
  c = find(cellfun(@func, b));
  if size(c)==1
      ret = char(a(c));
  end
end

function ret = func(item)
  ret = 0;
  if ~isempty(item)
      ret = item;
  end
end