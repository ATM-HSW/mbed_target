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

function [ret1, ret2, count]= mbed_getIDEs(filtered)

    pathstr = mbed_getTargetRootPath();
    newpath = fullfile(pathstr,'targets','mbed-os','tools');
    oldpath=cd(newpath);
    [~,cmdout]=system('python project.py -S ides');
    ret2=regexprep(cmdout,'\n\n','');
    ret1=regexprep(ret2,'\n','|');
    ret1=ret1(1:end-1);
    if filtered
        ret1=replace(ret1,'|gcc_arm|','|');
        ret2=regexprep(ret2,'\<gcc_arm\>\n','');
    end
    ret1=['none|' ret1];
    ret2=['none' 13 ret2];
    count=size(strfind(ret1,'|'),2)+1;
    cd(oldpath);

end
