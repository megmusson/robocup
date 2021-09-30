close all
clear
clc
%% Setting up parameters
mega = serialport("COM7", 9600);
samples = 500;
eachRead  = 1;
i = 1;

%% Set up code
configureTerminator(mega,"CR/LF")
flush(mega);
mega.UserData = struct("Data",[],"Raw", [], "Count",1);
readline(mega);
disp("delaying");
pause(1);
disp("done");
%% Loop code
% while loop to save data
while (mega.UserData.Count <= samples)
    
    data = readline(mega);
    temp = str2double(regexp(data,'\d*','match')');
    mega.UserData.Data(end+1) = temp(1);
    mega.UserData.Raw(end+1) = temp(2);
    mega.UserData.Count = mega.UserData.Count + 1;

end

saved1 = mega.UserData.Data(40:end);
saved2 = mega.UserData.Raw(40:end);

hold on
plot(saved1);
plot(saved2);

clear mega