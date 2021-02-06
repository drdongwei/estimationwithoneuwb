
y=xt(2,:);
fs = 25;
N = length(y);
y_fft=fft(y);
P2_y_fft =abs(y_fft/N);
P1_y_fft = P2_y_fft(1:N/2+1);
P1_y_fft(2:end-1) = 2*P1_y_fft(2:end-1);
f = fs*(0:N/2)/N;
figure(1)
plot(f,P1_y_fft)
xlabel('f (Hz)')

[b,a] = butter(4,0.6,'high');

yy = filtfilt(b,a,y);
figure(2)
plot(y,'r')
hold on
plot(yy,'b')
grid on
y=yy;
N = length(y);
y_fft=fft(y);
P2_y_fft =abs(y_fft/N);
P1_y_fft = P2_y_fft(1:N/2+1);
P1_y_fft(2:end-1) = 2*P1_y_fft(2:end-1);
f = fs*(0:N/2)/N;
figure(3)
plot(f,P1_y_fft)
xlabel('f (Hz)')
