# Setting up parameters
import serial
import syslog
import time

port = "COM8"
mega = serial.Serial(port,9600,timeout=5)

samples = 5000
eachRead  = 1
saved = zeros(samples,2)
i = 1

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

plot(mega.UserData.Data(2:end));
clear mega


port = '/dev/ttyS0'

i = 0

while (i < 4):
    # Serial write section

    setTempCar1 = 63
    setTempCar2 = 37
    ard.flush()
    setTemp1 = str(setTempCar1)
    setTemp2 = str(setTempCar2)
    print ("Python value sent: ")
    print (setTemp1)
    ard.write(setTemp1)
    time.sleep(4)

    # Serial read section
    msg = ard.readline()
    print ("Message from arduino: ")
    print (msg)
    i = i + 1
else:
    print "Exiting"
exit()