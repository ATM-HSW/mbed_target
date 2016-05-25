fi=dir('*.c');

for i=1:size(fi,1)
    [pathstr,name,ext] = fileparts(fi(i).name);
    try
    fi1=dir([name '.c']);
    fi2=dir([name '.mexw64']);
    if isempty(fi2)
        disp(['compile ' name '.c']);
        mex( [name '.c'])
    elseif fi1.datenum>fi2.datenum
        disp(['compile ' name '.c']);
        mex( [name '.c'])
    end
    catch E
        disp (E.message);
    end
    disp ''
end
