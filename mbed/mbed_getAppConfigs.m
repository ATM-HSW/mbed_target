%  MbedTarget Simulink target
%  Copyright (c) 2014-2018 Dr.O.Hagendorf , HS Wismar
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

function ret = mbed_getAppConfigs()

  pathstr = mbed_getTargetRootPath();
  targetsfiles = dir(fullfile(pathstr,'targets','appconfig'));
  ret='none';
  for i=1:size(targetsfiles,1)
      if strcmp(targetsfiles(i).name, '.')
          continue;
      end
      if strcmp(targetsfiles(i).name, '..')
          continue;
      end
      if targetsfiles(i).isdir==0
          continue;
      end
      ret = [ret '|' targetsfiles(i).name]; %#ok<AGROW>
  end

end
