clear all
close all
clc
pkg load control

P = zpk([],[-1 -5],1); % 制御対象

% 開ループステップ応答を求める
[time, response] = step(P);

% ステップ応答の最大傾きを求める
[max_slope, max_slope_index] = max(diff(response)./diff(time));

% 最大傾きの接線を求める
tangent_line = max_slope * (time(2:end) - time(max_slope_index)) + response(max_slope_index);

% 最大傾きの接線をプロット
figure;
plot(time, response, 'b', time(2:end), tangent_line, 'r--');
title('Open Loop Step Response with Tangent Line');
xlabel('Time');
ylabel('Response');
legend('Open Loop Step Response', 'Tangent Line');
print("open-loop-tangent.png","-dpng");

% 最大傾き R と時間軸の交点 L を求める
L = interp1(tangent_line, time(2:end), 0); % 時間軸との交点
R = max_slope; % 最大傾き

disp(['最大傾き R: ', num2str(R)]);
disp(['時間軸の交点 L: ', num2str(L)]);
print("op-res.png","-dpng");

% PIDゲインの導出　計算した値を下3つに代入
Kp = 0.01092773;
Ti = 0.37936;
Td = 0.09484;

Ki = Kp/Ti;
Kd = Td*Kp;

% PID制御による時間応答
C = pid(Kp, Ki, Kd);
L = C*P;
GG = L/(1+L);
G = minreal(GG);

figure;
step(G, 'r', P, 'b'); % Gのステップ応答を赤色、Pのステップ応答を青色でプロット
legend('G', 'P'); % 凡例を設定

