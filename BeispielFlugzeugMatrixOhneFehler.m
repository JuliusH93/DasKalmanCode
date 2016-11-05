clear all;

% Init values 

initVeloX = 280;    % 280 m/s
initX = 4000;       % 4000m in x

measuredX = [4000, 4260, 4550, 4860, 5110, 5000, 5250, 5300, 5100, 5300, 5032, 5600, 5300]; % In m
measuredVeloX = [280, 282, 285, 286, 290, 300, 310, 303, 304, 320, 330, 325, 352];  % In m/s
measurementMatrix = [measuredX ; measuredVeloX]
currentXMatrix = [1:size(measuredX,2); 1:size(measuredVeloX,2)]

initAccel = 2       % m/s^2
deltaT = 1          % 1s
deltaX = 25         % 25m   -> Observation error
deltaVeloX = 6      % 6 m/s -> Observation error
initPredErrorX = 20;
initPredVeloX = 5;

% Some convertion matrices
A = [1, deltaT; 0, 1]
B = [0.5 * deltaT.^2; deltaT]
C = [1, 0; 0, 1]
I = [1, 0; 0, 1]
H = [1, 0; 0, 1]

% 1) Calculate predicted state -> X_k_p = A X_k-1 + B u_k + w_k, w_k == 0
currentXMatrix(:,1) = A * measurementMatrix(:,1) + B * initAccel + 0
    
 % 2) Calculate initial process covariance matrix
currentP = [initPredErrorX^2, initPredErrorX * initPredVeloX; initPredErrorX * initPredVeloX, initPredVeloX^2] % P_k-1
currentP(1,2) = 0
currentP(2,1) = 0   % no dependency between the two values in this example!

for n = 2:size(measuredX,2)
    % 3) Calculate predicted process covariance matrix 
    % -> P_k_p = A P_k-1 AT + Q_k, Q_k == 0
    predictedP = A * currentP * transpose(A) + 0
    predictedP(1,2) = 0
    predictedP(2,1) = 0 % no dependency between the two values in this example!
    
    % 4) Calculate kalman gain -> K = P_k_p HT / H P_k_p HT + R, R = suared
    % errors on diagonal line
    K = (predictedP * transpose(H)) / (H * predictedP * transpose(H) + [deltaX^2, 0; 0, deltaVeloX^2])
    
    % 5) Import new observation -> Y_k = C Y_k_m + Z_k -> Z_k = 0
    newObservation = C * measurementMatrix(:,n) + 0
    
    % 6) Calculate current state -> X_k = K_k_p + K[Y_k - H X_k_p]
    currentXMatrix(:,n) = currentXMatrix(:,n-1) + K*(newObservation - H * currentXMatrix(:,n-1))
    
    % 7) Update process covariance matrix
    currentP = (I - K * H) * predictedP 
end

measurementMatrix
currentXMatrix

figure;
plot(measurementMatrix(1,:));
hold on;
plot(currentXMatrix(1,:));
hold off;

title('Position plot');
legend('Measurement','Prediction');

figure;
plot(measurementMatrix(2,:));
hold on;
plot(currentXMatrix(2,:));
hold off;

title('Velocity plot');
legend('Measurement','Prediction');


