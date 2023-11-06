clc;
clear;
close all;

measurement = readmatrix("measure.csv");
t = measurement(:,1);
y = measurement(:,2) - mean(measurement(:,2));
offset = measurement(1,1);
fs = 300; % sample freqency in Hz

figure(1)
plot(t - offset, y);
xlabel('time (s)')
ylabel('(V)')
title('Time domain EMG measurement data')

%% FFT 
[f, ym] = myfft(t,y);
figure(2)
plot(f, ym)
xlabel("frequency (Hz)")
ylabel("magnitude")
title("frequency domain EMG measurement data")


%% Butterworth high pass filter for 10 Hz
[b, a] = butter(1, 10/(fs/2), 'high');
figure(3)
freqz(b,a)
filtered10 = filter(b, a, y);

%% Butterworth bandstop filter for 50Hz
[b, a] = butter(1, [48 52]/max(f), 'stop');
figure(4)
freqz(b,a)
filtered50 = filter(b, a, filtered10);

%% Butterworth bandstop filter for 50Hz
[b, a] = butter(1, [98 100]/max(f), 'stop');
figure(5)
freqz(b,a)
filtered100 = filter(b, a, filtered50);

[f_filtered, ym_filtered] = myfft(t,filtered100);

figure(6)
plot(f, ym)
hold on
plot(f_filtered, ym_filtered)
xlabel("frequency (Hz)")
ylabel("magnitude")
title("Original data and filtered data in frequency domain")
legend('original EMG', 'filtered EMG')

figure(7)
plot(t,y)
hold on
plot(t,filtered100)
xlabel("time (s)")
ylabel("(V)")
title("Original data and filtered data in frequency domain")
legend('original EMG', 'filtered EMG')


