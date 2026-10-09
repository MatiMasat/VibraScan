% EJEMPLO_FFT  FFT de una captura de VibraScan con cargar_captura.m
%
% Ejecutar desde la carpeta matlab/ del repo (o agregarla al path con addpath).
% Cambiar la ruta de la captura por la que se quiera analizar.

cap = cargar_captura(fullfile('..', 'pc', 'capturas', 'captura_20261008_221540'));

% Segmento mas largo: la FFT no debe cruzar un hueco ni un cambio de ODR/rango
segs = unique(cap.seg);
cuantas = histc(cap.seg, segs);
[~, k] = max(cuantas);
dentro = (cap.seg == segs(k));

x = cap.az(dentro);          % eje Z, en g
x = x - mean(x);             % sin la gravedad (si no, un pico enorme en 0 Hz)
fs = cap.fs;                 % tasa MEDIDA (ver cargar_captura)
N = numel(x);

% FFT con ventana de Hann periodica (la misma que usa pc/vibrascan_espectro.py)
w = hann(N, 'periodic');
X = fft(x .* w);
nf = floor(N / 2) + 1;
f = (0:nf - 1)' * fs / N;
A = 2 * abs(X(1:nf)) / sum(w);    % amplitud de pico de cada tono, en g

figure;
plot(f, A * 1000);
grid on;
xlabel('Frecuencia [Hz]');
ylabel('Amplitud [mg]');
title(sprintf('Eje Z - fs medida %.2f Hz - %d muestras - %s', fs, N, cap.nota), 'Interpreter', 'none');

% Densidad espectral (Welch) con los parametros de pc/espectro.py: tramos de
% 4096, Hann periodica, 50 %. Nota: pwelch no resta la media de cada tramo
% (Python si); la diferencia esta solo en los primeros bins.
% [P, F] = pwelch(x, hann(4096, 'periodic'), 2048, 4096, fs, 'onesided');
% figure; semilogy(F, P); xlabel('Frecuencia [Hz]'); ylabel('PSD [g^2/Hz]'); grid on;
