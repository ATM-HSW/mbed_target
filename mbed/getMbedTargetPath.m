function ret = getMbedTargetPath()
  ret = '';
  a=strsplit(path,';');
  b=regexp(a,'\\mbed_\w*\\mbed');
  %b=regexp(a,'\\mbed_\w*\\blocks');
  c = find(cellfun(@func, b));
  if size(c)==1
      ret = fileparts(char(a(c)));
  end
end

function ret = func(item)
  ret = 0;
  if ~isempty(item)
      ret = item;
  end
end