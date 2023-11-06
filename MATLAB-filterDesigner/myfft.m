function [ f, ym, yp ] = myfft( t, x )
%MYFFT Computes the FFT according to Shiavi (basically double sided DFT).
%For this to happen, we need to divide the fft by the lenght of the signal
%array (i.e. length of x).
%This function strips the second half of the FFT result (redundant part).

% t: time array
% x: signal value array
% f: frquency array
% ym: magnitude of fft(x)
% yp: phase of fft(x)

s = ceil(length(x)/2);                      % round the length up to a multiple of 2
y = fft(x) / length(x);                     % scale the fft magnitudes
y = y(1:s);                                 % take only the first half (discard the second half since it is merely a mirror of the first half and so provides no additional information. but pay attention: the magnitude is not scaled so the spectral power is not preserved in this step)
ym = abs(y);                                % for the magnitude take the absolute value
yp = angle(y);                              % get the phase
f = 0:(s-1);                                % generate the length of the frequency vector
fspacing = 1 / ( length(t) * (t(2)-t(1)));  % calculate the frequency spacing
f = fspacing .* f;                          % generate the frequency vector using the afore generated length and the calculated frequency spacing

end

