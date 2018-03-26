function infos = wmicGet(classOrAlias, properties, wqlKeyWord, clauses)
%wmicGet Computer and operating system information on Windows.
%   wmicGet returns computer and operating system informations on Windows
%   platforms. It uses the Windows Management Instrumentation Command-line
%   (WMIC).
%
%   wmicGet(CLASS) returns properties of the WMI class. CLASS is a string
%   that begins with "Win32_" or "CIM_". See:
%   http://msdn.microsoft.com/en-us/library/aa394388%28v=vs.85%29.aspx
%   http://msdn.microsoft.com/en-us/library/aa386179%28v=vs.85%29.aspx
%
%   wmicGet(CLASS, PROPS) returns the properties PROPS of the WMI class.
%   PROPS is a string (one property) or a cell array of strings (multiple 
%   properties).
%
%   wmicGet(ALIAS,...) uses alias instead of WMI class name. See:
%   http://technet.microsoft.com/en-us/library/cc736307%28v=ws.10%29.aspx
%
%   infos = wmicGet(...) returns a string or a cell array of strings if
%   only one property is requested. Otherwise, it returns a structure whose
%   field names match the names of the WMI properties.
%
%   wmicGet(CLASS, PROPS, CLAUSE, EXPRESSION) uses WQL to perform more
%   advanced queries. CLAUSE must be 'where'. EXPRESSION is a Mx4 cell
%   array with one WQL expression per line. Each line of EXPRESSION must be
%   of the form: {WQL_KEYWORD PROP VALUE BOOL}. WQL_KEYWORD determines how
%   to combine successive WQL expressions and must be 'and' or 'or'. PROP
%   is a WMI class property. VALUE is the value to check. BOOL is a boolean
%   that determines if the comparison operator is "is" or "is not". See:
%   http://msdn.microsoft.com/en-us/library/aa394606%28v=vs.85%29.aspx
%
%   Examples
%
%   To get informations about CPU(s):
%       infos = wmicGet('cpu');
%       infos = wmicGet('win32_processor');
%
%   To get specific informations about CPU(s):
%       infos = wmicGet('cpu', {'currentclockspeed' 'maxclockspeed'});
%       infos = wmicGet('win32_processor', 'name');
%
%   To get size and free space of disks with drive letter C: or D:
%       infos = wmicGet('win32_logicaldisk', {'DeviceID' 'FreeSpace' 'Size'}, ...
%               'Where',{'' 'DeviceID' 'C:' true ; 'or' 'DeviceID' 'D:' true})
%
%   To get model name for connected USB disk(s):
%       infos = wmicGet('Win32_DiskDrive', 'Model', ...
%               'where', {'' 'InterfaceType' 'USB' true ; 'and' 'Size' 'null'  false});
%
%   More informations about WMIC:
%   http://technet.microsoft.com/en-us/library/cc779482.aspx
%

%   Author: Jerome Briot
%   http://briot-jerome.developpez.com/
%   http://www.mathworks.com/matlabcentral/fileexchange/authors/21984
%   Contact: dutmatlab#at#yahoo#dot#fr
%   Version: 1.0 - 16 Sep 2014
%
%   Tested with MATLAB from R2008a to R2014a on Windows XP, Vista, 7 and 8.
%

if verLessThan('matlab', '7.13')
    error(nargchk(1,4,nargin));
else
    narginchk(1, 4);
end

if ~ispc
    error('WMIC only available on Windows.')
end

if isWmicNotAvailable
    error('WMIC not available on this machine.')
end

if strncmpi(classOrAlias, 'Win32_', 6)
    p = 'path ';
else
    p = '';
end

whereClause = '';

if nargin==1 || (nargin==2 && isempty(properties))
    cmd = sprintf('wmic %s%s get /value <NUL | find "="', p, classOrAlias);
    properties = [];
elseif nargin==2
    properties = cellstr(properties);
    idx = cellfun(@isempty, properties);
    properties(idx) = [];
    if verLessThan('matlab', '8.1')
        cmd = sprintf('wmic %s%s get %s /value <NUL | find "="', p, classOrAlias, strjoin2(properties, ','));
    else
        cmd = sprintf('wmic %s%s get %s /value <NUL | find "="', p, classOrAlias, strjoin(properties, ','));
    end
elseif nargin==3
    error('1, 2 or 4 input arguments only. See: help wmicGet');
elseif nargin==4
    
    if ~strcmpi(wqlKeyWord, 'where')
        error('3rd input must be ''where''. See: help wmicGet');
    end
    
    if ~iscell(clauses)
        error('4th argument must be  a cell array. See: help wmicGet')
    end
    
    cmd = sprintf('wmic %s%s', p, classOrAlias);
    
    for nClauses = 1:size(clauses,1)
        
        if nClauses>1
            if strcmp(clauses{nClauses,1},'&') || strcmpi(clauses{nClauses,1},'and')
                wqlKey{1} = 'and';
            elseif strcmp(clauses{nClauses,1},'|') || strcmpi(clauses{nClauses,1},'or')
                wqlKey{1} = 'or';
            else
                error('In 4th argument, WQL key must be ''&'', ''|'', ''and'' or ''or''');
            end
        end
        
        if clauses{nClauses,4}
            wqlKey{2} = '=';
        else
            wqlKey{2} = '!=';
        end
        
        if ~strcmpi(clauses{nClauses,3}, 'null')
            clauses{nClauses,3} = ['''' clauses{nClauses,3} ''''];
        end
        
        if nClauses==1
            whereClause = sprintf('%s%s%s', clauses{nClauses,2}, ...
                wqlKey{2}, clauses{nClauses,3});
        else
            whereClause = sprintf(' %s %s %s%s%s ',whereClause, wqlKey{1}, clauses{nClauses,2}, ...
                wqlKey{2}, clauses{nClauses,3});
        end
    end
    
    properties = cellstr(properties);
    idx = cellfun(@isempty, properties);
    properties(idx) = [];
    
    if isempty(properties)
        cmd = sprintf('%s where "%s" get /value <NUL | find "="', cmd, whereClause);
    else
        properties = cellstr(properties);
        if verLessThan('matlab', '8.1')
            cmd = sprintf('%s where "%s" get %s /value <NUL | find "="',cmd, whereClause, strjoin2(properties, ','));
        else
            cmd = sprintf('%s where "%s" get %s /value <NUL | find "="',cmd, whereClause, strjoin(properties, ','));
        end
    end
    
end

[status, output] = dos(cmd);

% WMI Return Codes
% http://msdn.microsoft.com/en-us/library/aa394574%28v=vs.85%29.aspx
if status==3
    error('Unable to run WMIC: %s (%d)', output, status)
elseif status==2
    warning('WMIC: %s (%d)', output, status)
    infos = [];
    return
elseif status==1
    warning('WMIC: %s (%d)\n%s', output, status, cmd)
    infos = [];
    return
end

X = textscan(output, '%s', 'delimiter', '\n');

nProps = numel(properties);
properties = strtok(X{1}, '=');

if nProps == 1
    nProps = numel(properties);
    if nProps==1
        infos = strrep(X{1}{1}, [properties{1} '='], '');
    else
        for n = 1:nProps
            infos{n} = strrep(X{1}{n}, [properties{1} '='], '');
        end
    end
else
    infos = getProperties(X, properties);
end

function infos = getProperties(X, properties)

for n = 1:numel(properties)
    
    idx = strncmp(X{1}, [properties{n} '='], numel(properties{n})+1);
    
    if sum(idx)==1
        infos.(properties{n}) = strrep(X{1}{idx}, [properties{n} '='], '');
    else
        idx = find(idx);
        for k = 1:numel(idx)
            infos(k).(properties{n}) = strrep(X{1}{idx(k)}, [properties{n} '='], '');
        end
    end
    
end

function status = isWmicNotAvailable

[noneed,status] = evalc('dos(''wmic /? <NUL'')');

function str = strjoin2(prop, sep)

if numel(prop)==1
    str = prop{1};
else
    str = strcat(prop{1}, sep);
    for n = 2:numel(prop)
        str = strcat(str, prop{n}, sep);
    end
    str(end) = [];    
end
