% function ret = strmulticompare1(dataline)
% 
% commands = {'on', 'off'};
% ret = uint8(0);
% 
% for i=1:size(commands,2)
%     if contains(char(commands{i}), char(dataline(1:size(commands{i},2))))
%         ret = uint8(i);
%         return;
%     end
% end


function ret = strmulticompare(dataline)

commands = {'on', 'off'};
ret = uint8(0);

for i=1:size(commands,2)
    ok=uint8(1);
    for j=1:size(commands{i},2)
        if commands{i}(j)~=dataline(j)
            ok=uint8(0);
            break;
        end
    end
    if ok==uint8(1)
      ret = uint8(i);
      return;
    end
end
