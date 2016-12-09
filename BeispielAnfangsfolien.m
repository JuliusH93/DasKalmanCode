clear all;

%measurements = [89; 68; 60; 72; 74; 75; 73; 76; 71; 70; 72; 75; 72; 71; 70; 70; 72; 71; 50; 80; 40; 70; 77; 70; 60; 73; 71; 71; 75; 70; 72; 56; 64; 66; 73; 47];
measurements = [20.1,20.2,20.2,20.3,20.2,20.3,20.2,20.1,19.8,20.4,20.2,20.4,20.5,20,20,20.2,20.4,20.2,20.4,20.5,20.2,20.5,20,20,20,20,20,20,20,20,20,20,21,20,20,23,20,22,21,20,20,20,19,18,21,20,22,22,21,22,23,22,22,20,20,20,20,20,20,20.3,20,20,20,20,20,20,20,20,20,20,20]';
measurements(1:length(measurements)) = measurements(1:length(measurements)) + 80;
estimates = (1:size(measurements,1));
estimates(1) = 100;                              % initial estimate
errors = (1:size(measurements,1));
errors(1) = 2;
measErr = 4;
kalmanGain = (1:size(measurements,1));

for n = 2:size(measurements,1)
    kalmanGain(n) = errors(n-1) / (errors(n-1) + measErr);   % Calculate kalman gain
    
    estimates(n) = estimates(n-1) + kalmanGain(n)*(measurements(n-1) - estimates(n-1));
    %errors(n) = (measErr * errors(n-1)) / (measErr + errors(n-1));
    errors(n) = (1-kalmanGain(n)) * errors(n-1);
end

plot(measurements, 'LineWidth',1.5);
hold on;
plot(estimates, 'LineWidth', 1.5);
hold on;
%plot(errors);

legend('Messwerte','Vorhersage'); 
xlabel('t'); ylabel('Geschwindigkeit');
prettyPlotEx(7,5,'measurement.png');
