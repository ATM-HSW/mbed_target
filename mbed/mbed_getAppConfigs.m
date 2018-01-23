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

function ret = mbed_getAppConfigs()

  pathstr = mbed_getTargetRootPath();
  targetsfiles = ls(fullfile(pathstr,'targets','appconfig'));
  ret=['none'];
  for i=3:size(targetsfiles,1)
      [~,name,~] = fileparts(targetsfiles(i,:));
      ret = [ret '|' name]; %#ok<AGROW>
  end

end
