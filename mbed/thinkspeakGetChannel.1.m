import matlab.net.*
import matlab.net.http.*
r = RequestMessage;
uri = URI('https://api.thingspeak.com/channels/719728/fields/2.json?api_key=2KY0I4OUX7V3U5LT&results=0');
resp = send(r,uri);
status = resp.StatusCode