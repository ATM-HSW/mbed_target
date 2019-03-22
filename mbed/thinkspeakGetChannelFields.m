%import matlab.net.*
%import matlab.net.http.*

function [number, fnames] = thinkspeakGetChannelFields(channel, readkey)
import matlab.net.*
import matlab.net.http.*

number = [];
fnames = {};

r = RequestMessage;
uri = URI(['https://api.thingspeak.com/channels/' channel '/fields/2.json?api_key=' readkey '&results=0']);
resp = send(r,uri);
status = resp.StatusCode;

if status ~= 200
    disp(['error ' num2str(status)]);
    return;
end

names = fieldnames(resp.Body.Data.channel);
idx=1;
for i=(1:size(names,1))
    if startsWith(names(i), 'field')
      temp1 = names(i);
      temp1 = temp1{1};
      temp2 = getfield(resp.Body.Data.channel,temp1);
      % disp([ temp1 ' - ' temp2]);
      number = [number ; str2num(temp1(6))];
      fnames = {fnames{:}  {temp2}};
      idx = idx + 1;
    end
end

end