close all
clear
clc
%% Setting up parameters
mega = serialport("COM21", 9600);
samples = 1000;
eachRead  = 1;
i = 1;

%% Set up code
configureTerminator(mega,"CR/LF")
flush(mega);
mega.UserData = struct("Data",[],"Count",1);

%% Loop code
% while loop to save data
while (mega.UserData.Count <= samples)
    
    data = readline(mega);
    mega.UserData.Data(end+1) = str2double(data);
    mega.UserData.Count = mega.UserData.Count + 1;

end
saved = mega.UserData.Data(1:end);
plot(saved);
clear mega