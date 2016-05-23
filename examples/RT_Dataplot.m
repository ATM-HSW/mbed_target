%%real time data plot from a serial port 
% This matlab script is for ploting a graph by accessing serial port data in
% real time. Change the com values and all variable values accroding to
% your requirements. Dont forget to add terminator in to your serial device program.
% This script can be modified to be used on any platform by changing the
% serialPort variable. 
% Author: Moidu thavot.

%%Clear all variables

clear all;
%%Variables (Edit yourself)

SerialPort='com11'; %serial port
MaxDeviation = 3;%Maximum Allowable Change from one value to next 
TimeInterval=0.2;%time interval between each input.
loop=120;%count values
%%Set up the serial port object

s = serial(SerialPort,'BaudRate',9600,'Terminator','LF/CR')
fopen(s);



time =now;
voltage = 0;
%% Set up the figure 
figureHandle = figure('NumberTitle','off',...
    'Name','Voltage Characteristics',...
    'Color',[0 0 0],'Visible','off');

% Set axes
axesHandle = axes('Parent',figureHandle,...
    'YGrid','on',...
    'YColor',[0.9725 0.9725 0.9725],...
    'XGrid','on',...
    'XColor',[0.9725 0.9725 0.9725],...
    'Color',[0 0 0]);

hold on;

plotHandle = plot(axesHandle,time,voltage,'Marker','.','LineWidth',1,'Color',[0 1 0]);

xlim(axesHandle,[min(time) max(time+0.001)]);

% Create xlabel
xlabel('Time','FontWeight','bold','FontSize',14,'Color',[1 1 0]);

% Create ylabel
ylabel('Value','FontWeight','bold','FontSize',14,'Color',[1 1 0]);

% Create title
title('Real Time Data','FontSize',15,'Color',[1 1 0]);




%% Initializing variables

voltage(1)=0;
time(1)=0;
count = 2;
k=1;
while ~isequal(count,loop)
    %%Re creating Serial port before timeout
    k=k+1;  
    if k==25
        fclose(s);
        delete(s);
        clear s;        
        s = serial(SerialPort,'BaudRate',9600,'Terminator','LF/CR');
        fopen(s)
        k=0;
    end
    
    %%Serial data accessing 
    v = fscanf(s,'%f')
    voltage(count) = v;
     
    time(count) = count;
    set(plotHandle,'YData',voltage,'XData',time);
    set(figureHandle,'Visible','on');
    datetick('x','mm/DD HH:MM');
    
    pause(TimeInterval);
    count = count +1;
end



%% Clean up the serial port
fclose(s);
delete(s);
clear s;