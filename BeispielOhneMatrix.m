clear all;

measurements = [100; 68; 60; 72; 74; 75; 73; 76; 71; 70; 72; 75; 72; 71; 70; 70; 72; 71; 50; 80; 40; 70; 77; 70; 60; 73; 71; 71; 75; 70; 72; 56; 64; 66; 73; 47];
estimates = (1:size(measurements,1));
estimates(1) = 75;                              % initial estimate
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

legend('Measurements','Estimates'); 
xlabel('t'); ylabel('Values');
prettyPlotEx(7,5,'measurement.png');
