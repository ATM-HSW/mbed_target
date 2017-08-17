%  MbedTarget Simulink target
%  Copyright (c) 2014-2017 Dr.O.Hagendorf , HS Wismar
%
%  Licensed under the Apache License, Version 2.0 (the "License");
%  you may not use this file except in compliance with the License.
%  You may obtain a copy of the License at
%
%      http://www.apache.org/licenses/LICENSE-2.0
%
%  Unless required by applicable law or agreed to in writing, software
%  distributed under the License is distributed on an "AS IS" BASIS,
%  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%  See the License for the specific language governing permissions and
%  limitations under the License.

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