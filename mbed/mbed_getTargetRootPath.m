function ret = mbed_getTargetRootPath()
  ret = '';
  a=strsplit(path,{';', '\n'},'CollapseDelimiters',false, 'DelimiterType','RegularExpression');
  b=regexp(a,'\\mbed_target\\blocks\\slx');
  c = find(cellfun(@func, b));
  if size(c,2)==1
      ret = fileparts(fileparts(char(a(c))));
  elseif size(c,2)==0
      error('can not extract mbed target path: did not find mbed_target path in Matlab path');
  elseif size(c,2)>1
      error('can not extract mbed target path: found more than one mbed_target path in Matlab path');
  end
end

function ret = func(item)
  ret = 0;
  if ~isempty(item)
      ret = item;
  end
end