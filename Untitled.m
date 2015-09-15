cfg={'8*uint8' '4*int8' '1*bool'};
str = '';

for i=1:size(cfg,2)
    res = strsplit(cfg{i}, '*');
    if isempty(str)
        str = strjoin({['''' res{2} ''''] res{1}});
    else
        str = strjoin({str ['''' res{2} ''''] res{1}});
    end
end
str = strjoin({'{' str '}'});

disp(str);