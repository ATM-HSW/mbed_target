function ret = mbed_getTargetRootPath()
  ret = '';
  a=strsplit(path,{';', '\n'},'CollapseDelimiters',false, 'DelimiterType','RegularExpression');
  %b=regexp(a,'\\mbed_\w*\\mbed');
  b=regexp(a,'\\mbed_\w*\\blocks\\slx');
  c = find(cellfun(@func, b));
  if size(c)==1
      ret = fileparts(fileparts(char(a(c))));
  else
      error('can not extract mbed target path');
  end
end

function ret = func(item)
  ret = 0;
  if ~isempty(item)
      ret = item;
  end
end