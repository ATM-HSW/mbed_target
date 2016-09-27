fi=dir(fullfile(mbed_getTargetRootPath,'blocks','mex','sourcen','*.c'));

for i=1:size(fi,1)
    [pathstr,name,ext] = fileparts(fi(i).name);
    try
    fi1=dir(fullfile(mbed_getTargetRootPath,'blocks','mex','sourcen',[name '.c']));
    fi2=dir(fullfile(mbed_getTargetRootPath,'blocks','mex',[name '.mexw64']));
    if isempty(fi2)
        disp(['compile ' name '.c']);
        mex( '-outdir', fullfile(mbed_getTargetRootPath,'blocks','mex'),  fullfile(mbed_getTargetRootPath,'blocks','mex','sourcen',[name '.c']))
    elseif fi1.datenum>fi2.datenum
        disp(['compile ' name '.c']);
        mex( '-outdir', fullfile(mbed_getTargetRootPath,'blocks','mex'), fullfile(mbed_getTargetRootPath,'blocks','mex','sourcen',[name '.c']))
    end
    catch E
        disp (E.message);
    end
    disp ''
end
