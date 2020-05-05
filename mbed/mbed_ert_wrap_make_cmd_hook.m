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

function makeCmd = mbed_ert_wrap_make_cmd_hook(args)

    if ispc
        %disp('ispc');
        args.make = 'make.exe';
        % create a batch file to call make with following steps:
        %  - copy generated make file (modelname.mk) to Makefile
        %  - call make.exe
        [~,modelName,~] = fileparts( which (bdroot));
        lCodeGenFolder = Simulink.fileGenControl('getConfig').CodeGenFolder;
        buildAreaSrcFolder = fullfile(lCodeGenFolder, [modelName '_slprj']);
        makeexepath = fullfile(mbed_getTargetRootPath(), 'buildtools', 'bin');
        makeexefullpath = fullfile(makeexepath, 'make.exe');
        buildAreaDstFolder = mbed_getTargetDestFolder();
        copy1 = ['copy "' fullfile(buildAreaSrcFolder, [args.modelName '.mk']) '" "' fullfile(buildAreaDstFolder, 'BUILD', 'Makefile') '"'];
        copy2 = ['copy "' fullfile(buildAreaDstFolder, 'target_tools.mk') '" "' fullfile(buildAreaDstFolder, 'BUILD', 'target_tools.mk') '"'];
        copy3 = ['copy "' fullfile(buildAreaSrcFolder, [args.modelName '.bat']) '" "' buildAreaDstFolder '"'];
        cd1   = [buildAreaDstFolder(1:2)];
        cd2   = ['cd "' buildAreaDstFolder '"'];
        set1  = ['set PATH=' makeexepath ';%PATH%'];
        gcc_path = ['PATH=..\..\..\buildtools\gcc-arm-none-eabi-6-2017-q2\bin\'];
        param = get_param(modelName, 'ObjectParameters');
        if isfield(param,'MakeJobs')
            jobs = ['-j ' get_param(gcs,'MakeJobs')];
        else
            jobs = '';
        end
        CR=sprintf('\n');
        args.makeCmd = ['@echo off & ' CR copy1 ' & ' CR copy2 ' & ' CR copy3 ' & ' CR cd1 ' & ' CR cd2 ' & ' CR set1 ' & ' CR makeexefullpath ' ' jobs ' -C BUILD ' gcc_path];
        args.verbose = 1;
        makeCmd = setup_for_default(args);
    else
        %disp('no ispc');
%         makeCmd = fullfile(matlabroot,'bin',lower(computer),'gmake');
%         pcMakeCmd = '%MBED_ROOT%/hardware/tools/avr/utils/bin/make';
%         len=length(pcMakeCmd);
%         assert(strncmp(args.makeCmd, pcMakeCmd, len), 'makeCmd must default to PC Make command');
%         makeCmd = [makeCmd args.makeCmd(len+1:end)];
    end
end
