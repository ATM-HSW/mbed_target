function makeInfo=rtwmakecfg()
%RTWMAKECFG adds include and source directories to the generated makefiles.
%   For details refer to documentation on the rtwmakecfg API.

%   Copyright 1994-2011 The MathWorks, Inc.
%             2014 Dr. Olaf Hagendorf, HS Wismar

% Add the folder where this file resides to the include path
blocks_inc_path = fileparts(mfilename('fullpath'));
makeInfo.includePath = {blocks_inc_path};
