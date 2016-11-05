clear all;

measurements = [100; 68; 60; 72; 74; 75; 73; 76; 71; 70; 72; 75; 72; 71; 70; 70; 72; 71; 50; 80; 40; 70; 77; 70; 60; 73; 71; 71; 75; 76; 73; 70; 73; 80; 72; 60; 73; 50; 71; 40; 50; 30; 42; 45; 52; 42; 33; 33; 33; 30; 20; 32; 25; 32; 32; 23; 33; 30; 30; 20; 32; 25; 32; 32; 23; 33; 32];
estimates = (1:size(measurements,1));
estimates(1) = 75;                              % initial estimate
errors = (1:size(measurements,1));
errors(1) = 2;
measErr = 4;

for n = 2:size(measurements,1)
    kalmanGain = errors(n-1) / (errors(n-1) + measErr);   % Calculate kalman gain
    
    estimates(n) = estimates(n-1) + kalmanGain*(measurements(n) - estimates(n-1));
    %errors(n) = (measErr * errors(n-1)) / (measErr + errors(n-1));
    errors(n) = (1-kalmanGain) * errors(n-1);
end

plot(measurements);
hold on;
plot(estimates);
hold on;
plot(errors);

legend('Measurements','Estimates', 'Errors'); 
xlabel('t'); ylabel('Values');
